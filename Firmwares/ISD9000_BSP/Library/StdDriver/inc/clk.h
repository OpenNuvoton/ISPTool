/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 1$
 * $Date: 14/10/22 2:35p $
 * @brief    ISD9000 series CLK driver header file
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

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup ISD9000_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCON constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_DPDWAKEUP_PINOSC10K   0      /*!< DPD wake up from WAKEUP pin and OSC 16k */  
#define CLK_DPDWAKEUP_PIN         1      /*!< DPD wake up from WAKEUP pin */
#define CLK_DPDWAKEUP_OSC10K      2      /*!< DPD wake up from OSC 16k */
#define CLK_DPDWAKEUP_POR         3      /*!< DPD wake up from POR event trigger */

#define CLK_DPDWAKETIME_12ms    (0x1ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (12.8 ms) */
#define CLK_DPDWAKETIME_25ms    (0x2ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (25.6 ms) */
#define CLK_DPDWAKETIME_50ms    (0x4ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (51.2 ms) */
#define CLK_DPDWAKETIME_100ms   (0x8ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (102.4 ms) */
	
/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.  (Write-protection)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_HIRC         (0x0UL<<CLK_CLKSEL0_HCLKSEL_Pos)	/*!< Setting clock source as HIRC  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LXT          (0x1UL<<CLK_CLKSEL0_HCLKSEL_Pos)	/*!< Setting clock source as LXT  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LIRC         (0x2UL<<CLK_CLKSEL0_HCLKSEL_Pos)	/*!< Setting clock source as LIRC  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x0UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as LIRC 10KHz  \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_LXT           (0x1UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as LXT 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  (0x2UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as HCLK/2048  \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_HIRC          (0x3UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_ADCSEL_HCLK          (0x0UL<<CLK_CLKSEL1_ADCSEL_Pos)       /*!< Setting ADC clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_ADCSEL_HIRC          (0x2UL<<CLK_CLKSEL1_ADCSEL_Pos)       /*!< Setting ADC clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMR0SEL_HCLK         (0x0UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as LXT 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external trigger  \hideinitializer */    
#define CLK_CLKSEL1_TMR0SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMR1SEL_HCLK         (0x0UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as LXT 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external trigger   \hideinitializer */    
#define CLK_CLKSEL1_TMR1SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMR2SEL_HCLK         (0x0UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_LIRC         (0x2UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMRFSEL_LXT_DIV32           (0x0UL<<CLK_CLKSEL1_TMRFSEL_Pos)      /*!< Setting Timer F clock source as LXT/32  \hideinitializer */
#define CLK_CLKSEL1_TMRFSEL_LXT_DIV128          (0x1UL<<CLK_CLKSEL1_TMRFSEL_Pos)      /*!< Setting Timer F clock source as LXT/128  \hideinitializer */
#define CLK_CLKSEL1_TMRFSEL_LIRC_DIV32          (0x2UL<<CLK_CLKSEL1_TMRFSEL_Pos)      /*!< Setting Timer F clock source as LIRC/32768  \hideinitializer */
#define CLK_CLKSEL1_TMRFSEL_LIRC_DIV128         (0x3UL<<CLK_CLKSEL1_TMRFSEL_Pos)      /*!< Setting Timer F clock source as LIRC/32768/4  \hideinitializer */    
#define CLK_CLKSEL1_TMRFSEL_HIRC_DIV32768       (0x6UL<<CLK_CLKSEL1_TMRFSEL_Pos)      /*!< Setting Timer F clock source as HIRC/32768  \hideinitializer */
#define CLK_CLKSEL1_TMRFSEL_HIRC_DIV32768_DIV4  (0x7UL<<CLK_CLKSEL1_TMRFSEL_Pos)      /*!< Setting Timer F clock source as HIRC/32768/4  \hideinitializer */    

#define CLK_CLKSEL1_RTCSEL_LIRC          (0x0UL<<CLK_CLKSEL1_RTCSEL_Pos)   /*!< Setting RTC clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_RTCSEL_LXT           (0x1UL<<CLK_CLKSEL1_RTCSEL_Pos)   /*!< Setting RTC clock source as LXT  \hideinitializer */

#define CLK_CLKSEL1_PWM0SEL_HCLK         (0x0UL<<CLK_CLKSEL1_PWM0SEL_Pos)   /*!< Setting PWM0 and PWM clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM0SEL_LXT          (0x1UL<<CLK_CLKSEL1_PWM0SEL_Pos)   /*!< Setting PWM0 and PWM clock source as LXT  \hideinitializer */
#define CLK_CLKSEL1_PWM0SEL_LIRC         (0x2UL<<CLK_CLKSEL1_PWM0SEL_Pos)   /*!< Setting PWM0 and PWM clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_PWM0SEL_HIRC         (0x3UL<<CLK_CLKSEL1_PWM0SEL_Pos)   /*!< Setting PWM0 and PWM clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_PWM1SEL_HCLK         (0x0UL<<CLK_CLKSEL1_PWM1SEL_Pos)   /*!< Setting PWM1 and PWM clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM1SEL_LXT          (0x1UL<<CLK_CLKSEL1_PWM1SEL_Pos)   /*!< Setting PWM1 and PWM clock source as LXT  \hideinitializer */
#define CLK_CLKSEL1_PWM1SEL_LIRC         (0x2UL<<CLK_CLKSEL1_PWM1SEL_Pos)   /*!< Setting PWM1 and PWM clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_PWM1SEL_HIRC         (0x3UL<<CLK_CLKSEL1_PWM1SEL_Pos)   /*!< Setting PWM1 and PWM clock source as HIRC  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV_HCLK(x)    (((x)-1) << CLK_CLKDIV_HCLKDIV_Pos)    /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV_ADC(x)     (((x)-1) << CLK_CLKDIV_ADCDIV_Pos)     /*!< CLKDIV Setting for ADC clock divider. It could be 1~4096  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LDOSEL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_LDOSEL_1_8V			(0x0ul<<CLK_DPDSTATE_LDOVOLT_Pos)	/*!< Select LDO Output 1.8 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_4V			(0x1ul<<CLK_DPDSTATE_LDOVOLT_Pos)	/*!< Select LDO Output 2.4 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_5V			(0x2ul<<CLK_DPDSTATE_LDOVOLT_Pos)  	/*!< Select LDO Output 2.5 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_7V			(0x3ul<<CLK_DPDSTATE_LDOVOLT_Pos)	/*!< Select LDO Output 2.7 Voltage  \hideinitializer */
#define CLK_LDOSEL_3_0V			(0x4ul<<CLK_DPDSTATE_LDOVOLT_Pos)	/*!< Select LDO Output 3.0 Voltage  \hideinitializer */
#define CLK_LDOSEL_3_3V			(0x5ul<<CLK_DPDSTATE_LDOVOLT_Pos)  	/*!< Select LDO Output 3.3 Voltage  \hideinitializer */
#define CLK_LDOSEL_1_5V			(0x6ul<<CLK_DPDSTATE_LDOVOLT_Pos)	/*!< Select LDO Output 1.5 Voltage  \hideinitializer */
#define CLK_LDOSEL_1_7V			(0x7ul<<CLK_DPDSTATE_LDOVOLT_Pos)	/*!< Select LDO Output 1.7 Voltage  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_AHPBCLK(x)                  ((x >>31) & 0x1)    /*!< Calculate AHBCLK/APBCLK offset on MODULE index  \hideinitializer */
#define MODULE_CLKSEL(x)                   ((x >>30) & 0x1)    /*!< Calculate CLKSEL0~1 register offset by MODULE index  \hideinitializer */
#define MODULE_CLKSEL_Msk(x)               ((x >>27) & 0x7)    /*!< Calculate mask bits of CLKSEL0~1 by MODULE index  \hideinitializer */
#define MODULE_CLKSEL_Pos(x)               ((x >>22) & 0x1f)   /*!< Calculate mask bits offset of CLKSEL0~1 by MODULE index  \hideinitializer */
#define MODULE_CLKDIV(x)                   ((x >>22) & 0x0)    /*!< Calculate APBCLK CLKDIV by MODULE index  \hideinitializer */
#define MODULE_CLKDIV_Msk(x)               ((x >>10) & 0xfff)  /*!< Calculate mask bits of CLKDIV by MODULE index  \hideinitializer */
#define MODULE_CLKDIV_Pos(x)               ((x >>5 ) & 0x1f)   /*!< Calculate ask bits offset of CLKDIV by MODULE index  \hideinitializer */
#define MODULE_IP_EN_Pos(x)                ((x >>0 ) & 0x1f)   /*!< Calculate APBCLK enabled offset by MODULE index  \hideinitializer */
#define MODULE_NoMsk                       0x0                 /*!< Not mask by MODULE index  \hideinitializer */

/*--------------------------------------------------------------------------------------------------------------------------------------*/
/*   AHBCLK/APBCLK(1) | CLKSEL(1) | CLKSEL_Msk(3) |    CLKSEL_Pos(5)    | CLKDIV(0) | CLKDIV_Msk(12) |     CLKDIV_Pos(5)  |  IP_EN_Pos(5)*/
/*--------------------------------------------------------------------------------------------------------------------------------------*/
#define PDMA_MODULE      ((0UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_PDMACKEN_Pos)  /*!< PDMA Module  \hideinitializer */ 
#define SPIM_MODULE      ((0UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_SPIMCKEN_Pos)  /*!< SPIM Module  \hideinitializer */ 
#define ISP_MODULE       ((0UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_ISPCKEN_Pos)   /*!< ISP Module  \hideinitializer */
#define WDT_MODULE       ((1UL<<31)|(1<<30)|(           3<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_WDTCKEN_Pos)   /*!< Watchdog Timer Module  \hideinitializer */
#define RTC_MODULE       ((1UL<<31)|(1<<30)|(           1<<27)|(24<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_RTCCKEN_Pos)    /*!< RTC Module  \hideinitializer */
#define TMR0_MODULE      ((1UL<<31)|(1<<30)|(           7<<27)|( 8<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_TMR0CKEN_Pos)   /*!< Timer0 Module  \hideinitializer */
#define TMR1_MODULE      ((1UL<<31)|(1<<30)|(           7<<27)|(12<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_TMR1CKEN_Pos)   /*!< Timer1 Module  \hideinitializer */
#define TMR2_MODULE      ((1UL<<31)|(1<<30)|(           7<<27)|(16<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_TMR2CKEN_Pos)   /*!< Timer2 Module  \hideinitializer */
#define TMRF_MODULE      ((1UL<<31)|(1<<30)|(           7<<27)|(20<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_TMRFCKEN_Pos)   /*!< TimerF Module  \hideinitializer */
#define SPI0_MODULE      ((1UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_SPI0CKEN_Pos)   /*!< SPI0 Module  \hideinitializer */
#define PWM0_MODULE      ((1UL<<31)|(1<<30)|(           3<<27)|(28<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_PWM0CKEN_Pos)   /*!< PWM0 Module  \hideinitializer */
#define PWM1_MODULE      ((1UL<<31)|(1<<30)|(           3<<27)|(30<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_PWM1CKEN_Pos)   /*!< PWM1 Module  \hideinitializer */
#define ADC_MODULE       ((1UL<<31)|(1<<30)|(           3<<27)|( 2<<22)|(0<<22)|(       0x07F<<10)|(16<<5)|CLK_APBCLK_ADCCKEN_Pos)    /*!< ADC Module  \hideinitializer */
#define DPWM_MODULE      ((1UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_DPWMCKEN_Pos)   /*!< DPWM Module  \hideinitializer */
#define APU_MODULE       ((1UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_DACCKEN_Pos)    /*!< APU Module  \hideinitializer */
#define UART0_MODULE     ((0UL<<31)|(0<<30)|(MODULE_NoMsk<<27)|( 0<<22)|(0<<22)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_UART0CKEN_Pos)  /*!< UART0 Module  \hideinitializer */


/*@}*/ /* end of group ISD9000_CLK_EXPORTED_CONSTANTS */


/** @addtogroup ISD9000_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief     Get power down state.
  * @param[in] clk Base address of CLK module.
  * @param[in] u8Flag Power state flag.
  *            - \ref CLK_PWRCTL_PD_WAIT_CPU_Msk
  *            - \ref CLK_PWRCTL_PWR_DOWN_Msk
  *            - \ref CLK_PWRCTL_WINT_STS_Msk
  *            - \ref CLK_PWRCTL_WINT_EN_Msk
  * @return    power down flag
  */
#define CLK_GET_POWERDOWNFLAG(clk, u8Flag)		(clk->PWRCTL&u8Flag)

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

void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel);
void CLK_StandbyPowerDown(void);
void CLK_DeepSleep(void);
void CLK_Sleep(void);
//void CLK_EnablePowerDownMode(void);
//void CLK_DisablePowerDownMode(void);
//void CLK_ClearWakeUpStatusFlag(void);
//void CLK_EnableWakeUpInterrupt(void);
//void CLK_DisableWakeUpInterrupt(void);
//void CLK_EnableWakeUpDelay(void);
//void CLK_DisableWakeUpDelay(void);

void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
uint32_t CLK_GetHCLKFreq(void);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_EnableLDO( uint32_t u32LDOSel );
void CLK_DisableLDO(void);
void CLK_EnableX32Filter(void);
void CLK_DisableX32Filter(void);

/*@}*/ /* end of group ISD9000_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_CLK_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
