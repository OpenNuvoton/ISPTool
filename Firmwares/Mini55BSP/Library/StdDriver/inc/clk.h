/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/07/03 11:23a $
 * @brief    MINI55 series CLK driver header file
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


/** @addtogroup MINI55_Device_Driver MINI55 Device Driver
  @{
*/

/** @addtogroup MINI55_CLK_Driver CLK Driver
  @{
*/



/** @addtogroup MINI55_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCTL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRCTL_XTL12M              0x01UL /*!< Setting External Crystal Oscillator as 12MHz         */
#define CLK_PWRCTL_HXT                 0x01UL /*!< Setting External Crystal Oscillator as 12MHz         */
#define CLK_PWRCTL_XTL32K              0x02UL /*!< Setting External Crystal Oscillator as 32KHz         */
#define CLK_PWRCTL_LXT                 0x02UL /*!< Setting External Crystal Oscillator as 32KHz         */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_XTAL          0x00UL /*!< Setting clock source as external XTAL */
#define CLK_CLKSEL0_HCLKSEL_LIRC          0x03UL /*!< Setting clock source as internal 10KHz RC clock */
#define CLK_CLKSEL0_HCLKSEL_HIRC          0x07UL /*!< Setting clock source as internal RC clock */
#define CLK_CLKSEL0_STCLKSEL_XTAL         0x00UL /*!< Setting clock source as HXT or LXT */
#define CLK_CLKSEL0_STCLKSEL_XTAL_DIV2    0x10UL /*!< Setting clock source as external XTAL/2 */
#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV2    0x18UL /*!< Setting clock source as HCLK/2 */
#define CLK_CLKSEL0_STCLKSEL_HIRC_DIV2    0x38UL /*!< Setting clock source as internal RC clock/2 */
#define CLK_CLKSEL0_STCLKSEL_HCLK         (0x01UL<<SysTick_CTRL_CLKSOURCE_Pos) /*!< Setting clock source as HCLK */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_XTAL          0x00000000UL /*!< Setting WDT clock source as external XTAL */
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  0x00000002UL /*!< Setting WDT clock source as HCLK/2048 */
#define CLK_CLKSEL1_WDTSEL_IRC10K        0x00000003UL /*!< Setting WDT clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_WDTSEL_LIRC          0x00000003UL /*!< Setting WDT clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_ADCSEL_XTAL          0x00000000UL /*!< Setting ADC clock source as external XTAL */
#define CLK_CLKSEL1_ADCSEL_HCLK          0x00000008UL /*!< Setting ADC clock source as HCLK */
#define CLK_CLKSEL1_ADCSEL_HIRC          0x0000000CUL /*!< Setting ADC clock source as internal RC clock */
#define CLK_CLKSEL1_SPISEL_HXTorLXT      0x00000000UL /*!< Setting SPI clock source as HXT or LXT */
#define CLK_CLKSEL1_SPISEL_HCLK          0x00000010UL /*!< Setting SPI clock source as HCLK */
#define CLK_CLKSEL1_TMR0SEL_XTAL         0x00000000UL /*!< Setting Timer 0 clock source as external XTAL */
#define CLK_CLKSEL1_TMR0SEL_LIRC         0x00000100UL /*!< Setting Timer 0 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR0SEL_HCLK         0x00000200UL /*!< Setting Timer 0 clock source as HCLK */
#define CLK_CLKSEL1_TMR0SEL_HIRC         0x00000700UL /*!< Setting Timer 0 clock source as internal RC clock */
#define CLK_CLKSEL1_TMR1SEL_XTAL         0x00000000UL /*!< Setting Timer 1 clock source as external XTAL */
#define CLK_CLKSEL1_TMR1SEL_LIRC         0x00001000UL /*!< Setting Timer 1 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR1SEL_HCLK         0x00002000UL /*!< Setting Timer 1 clock source as HCLK */
#define CLK_CLKSEL1_TMR1SEL_HIRC         0x00007000UL /*!< Setting Timer 1 clock source as internal RC clock */
#define CLK_CLKSEL1_UART0SEL_XTAL        0x00000000UL /*!< Setting UART0 clock source as external XTAL */
#define CLK_CLKSEL1_UART0SEL_HIRC        0x02000000UL /*!< Setting UART0 clock source as external internal RC clock */
#define CLK_CLKSEL1_UART1SEL_XTAL        0x00000000UL /*!< Setting UART1 clock source as external XTAL */
#define CLK_CLKSEL1_UART1SEL_HIRC        0x08000000UL /*!< Setting UART1 clock source as external internal RC clock */
#define CLK_CLKSEL1_PWMCH01SEL_HCLK      0x20000000UL /*!< Setting PWM01 clock source as external HCLK */
#define CLK_CLKSEL1_PWMCH23SEL_HCLK      0x80000000UL /*!< Setting PWM23 clock source as external HCLK */


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_FDIVSEL_XTAL        0x00000000UL /*!< Setting FRQDIV clock source as external XTAL */
#define CLK_CLKSEL2_FDIVSEL_HXT         0x00000000UL /*!< Setting FRQDIV clock source as external XTAL */
#define CLK_CLKSEL2_FDIVSEL_LXT         0x00000000UL /*!< Setting FRQDIV clock source as external XTAL */
#define CLK_CLKSEL2_FDIVSEL_HCLK        0x00000008UL /*!< Setting FRQDIV clock source as HCLK */
#define CLK_CLKSEL2_FDIVSEL_HIRC        0x0000000CUL /*!< Setting FRQDIV clock source as internal RC clock */
#define CLK_CLKSEL2_PWMCH45SEL_HCLK     0x00000020UL /*!< Setting PWM45 clock source as HCLK */


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV_ADC(x)   (((x)-1) << 16)  /*!< CLKDIV Setting for ADC clock divider. It could be 1~256 */
#define CLK_CLKDIV_UART(x)  (((x)-1) <<  8)  /*!< CLKDIV Setting for UART0 clock divider. It could be 1~16 */
#define CLK_CLKDIV_UART0(x) (((x)-1) <<  8) /*!< CLKDIV Setting for UART0 clock divider. It could be 1~16 */
#define CLK_CLKDIV_UART1(x) (((x)-1) << 12) /*!< CLKDIV Setting for UART1 clock divider. It could be 1~16 */
#define CLK_CLKDIV_HCLK(x)  ((x)-1)         /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16 */

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
/*   APBCLK(1) | CLKSEL(2) | CLKSEL_Msk(4) |    CLKSEL_Pos(5)    | CLKDIV(2) | CLKDIV_Msk(8) |     CLKDIV_Pos(5)  |  IP_EN_Pos(5)*/
/*-------------------------------------------------------------------------------------------------------------------------------*/
#define WDT_MODULE        ((0x0<<31)|(0x1<<29)|(0x3<<25)|( 0<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_WDTCKEN_Pos )  /*!< Watchdog Timer Module */
#define TMR0_MODULE       ((0x0<<31)|(0x1<<29)|(0x7<<25)|( 8<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_TMR0CKEN_Pos)  /*!< Timer0 Module */
#define TMR1_MODULE       ((0x0<<31)|(0x1<<29)|(0x7<<25)|(12<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_TMR1CKEN_Pos)  /*!< Timer1 Module */
#define FDIV_MODULE       ((0x0<<31)|(0x3<<29)|(0x3<<25)|( 2<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_CLKOCKEN_Pos)  /*!< Frequency Divider Output Module */
#define I2C_MODULE        ((0x0<<31)|(0x3<<29)|(MODULE_NoMsk<<25)|(31<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_I2CCKEN_Pos)   /*!< I2C Module */
#define SPI_MODULE        ((0x0<<31)|(0x1<<29)|(0x1<<25)|( 4<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_SPICKEN_Pos)   /*!< SPI Module */
#define UART0_MODULE      ((0x0<<31)|(0x1<<29)|(0x3<<25)|(24<<20)|(0x0<<18)|(0x0F<<10)|( 8<<5)|CLK_APBCLK_UART0CKEN_Pos)          /*!< UART0 Module */
#define UART1_MODULE      ((0x0<<31)|(0x1<<29)|(0x3<<25)|(26<<20)|(0x0<<18)|(0x0F<<10)|(12<<5)|CLK_APBCLK_UART1CKEN_Pos)          /*!< UART1 Module */
#define PWM01_MODULE      ((0x0<<31)|(0x1<<29)|(0x3<<25)|(28<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_PWMCH01CKEN_Pos) /*!< PWM Channel0 and Channel1 Module */
#define PWM23_MODULE      ((0x0<<31)|(0x1<<29)|(0x3<<25)|(30<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_PWMCH23CKEN_Pos) /*!< PWM Channel2 and Channel3 Module */
#define PWM45_MODULE      ((0x0<<31)|(0x3<<29)|(0x3<<25)|( 4<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_PWMCH45CKEN_Pos) /*!< PWM Channel4 and Channel5 Module */
#define ADC_MODULE        ((0x0<<31)|(0x1<<29)|(0x3<<25)|( 2<<20)|(0x0<<18)|(0xFF<<10)|(16<<5)|CLK_APBCLK_ADCCKEN_Pos)           /*!< ADC Module */
#define ACMP_MODULE       ((0x0<<31)|(0x3<<29)|(MODULE_NoMsk<<25)|(31<<20)|(0x3<<18)|(MODULE_NoMsk<<10)|(31<<5)|CLK_APBCLK_ACMPCKEN_Pos)  /*!< ACMP Module */

/*@}*/ /* end of group MINI55_CLK_EXPORTED_CONSTANTS */


/** @addtogroup MINI55_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/
/**
  * @brief      This function execute delay function.
  * @param[in]  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             72MHz => 233016us, 50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
__STATIC_INLINE void CLK_SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    /* Disable SysTick counter */
    SysTick->CTRL = 0;
}

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);



/*@}*/ /* end of group MINI55_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI55_CLK_Driver */

/*@}*/ /* end of group MINI55_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
