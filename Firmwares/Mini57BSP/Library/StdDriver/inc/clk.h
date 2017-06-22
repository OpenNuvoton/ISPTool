/**************************************************************************//**
 * @file     clk.h
 * @version  V1.0
 * $Revision  1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series CLK driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup Mini57_CLK_EXPORTED_CONSTANTS CLK Exported Constants
@{
*/
#define FREQ_48MHZ              48000000    /*!< The frequency for 48MHz     */
#define FREQ_24MHZ              24000000    /*!< The frequency for 24MHz     */
#define FREQ_16MHZ              16000000    /*!< The frequency for 16MHz     */
#define FREQ_12MHZ              12000000    /*!< The frequency for 12MHz     */
#define FREQ_8MHZ                8000000    /*!< The frequency for 8MHz      */
#define FREQ_6MHZ                6000000    /*!< The frequency for 6MHz      */
#define FREQ_1MHZ                1000000    /*!< The frequency for 1MHz      */

/********************* Bit definition of PWRCTL register **********************/
#define CLK_PWRCTL_HXT_EN           (0x1UL<<CLK_PWRCTL_XTLEN_Pos)       /*!< Enable external high speed crystal HXT       */
#define CLK_PWRCTL_LXT_EN           (0x2UL<<CLK_PWRCTL_XTLEN_Pos)       /*!< Enable external low speed crystal HXT        */
#define CLK_PWRCTL_HIRC_EN          (0x1UL<<CLK_PWRCTL_HIRCEN_Pos)      /*!< Enable internal high speed oscillator HIRC  */
#define CLK_PWRCTL_LIRC_EN          (0x1UL<<CLK_PWRCTL_LIRCEN_Pos)      /*!< Enable internal low speed oscillator LIRC   */
#define CLK_PWRCTL_DELY_EN          (0x1UL<<CLK_PWRCTL_PDWKDLY_Pos)     /*!< Enable the wake-up delay counter            */
#define CLK_PWRCTL_WAKEINT_EN       (0x1UL<<CLK_PWRCTL_PDWKIEN_Pos)     /*!< Enable the wake-up interrupt                */
#define CLK_PWRCTL_PWRDOWN_EN       (0x1UL<<CLK_PWRCTL_PDEN_Pos)        /*!< Enable the power down feature               */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLK_CLKSEL0 constant definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_HCLK_SRC_EXT            (0x0UL<<CLK_CLKSEL0_HCLKSEL_Pos)    /*!< Select HCLK clock source from external crystal                      */
#define CLK_HCLK_SRC_LIRC           (0x1UL<<CLK_CLKSEL0_HCLKSEL_Pos)    /*!< Select HCLK clock source from internal low speed oscillator LIRC    */
#define CLK_HCLK_SRC_HIRC           (0x3UL<<CLK_CLKSEL0_HCLKSEL_Pos)    /*!< Select HCLK clock source from internal high speed oscillator HIRC   */

#define CLK_SYSTICK_SRC_EXT         (0x00UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Select SysTick clock source from external crystal       */
#define CLK_SYSTICK_SRC_EXT_HALF    (0x01UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Select SysTick clock source from external crystal / 2   */
#define CLK_SYSTICK_SRC_HCLK_HALF   (0x02UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Select SysTick clock source from HCLK / 2               */
#define CLK_SYSTICK_SRC_HIRC_HALF   (0x03UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Select SysTick clock source from HIRC / 2               */
#define CLK_SYSTICK_SRC_HCLK        (0x04UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Select SysTick clock source from HCLK. This is NOT valid mask for CLK_CLKSEL0[STCLKSEL]. This is just a index to make driver set 1 to SYST_CTL[CLKSRC] to use HCLK (core clock) as SysTick clock source.    */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLK_CLKSEL1 constant definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKO_SRC_EXT            (0x00UL<<CLK_CLKSEL1_CLKOSEL_Pos)   /*!< Select CLKO clock source from external crystal  */
#define CLK_CLKO_SRC_HCLK           (0x02UL<<CLK_CLKSEL1_CLKOSEL_Pos)   /*!< Select CLKO clock source from HCLK              */
#define CLK_CLKO_SRC_HIRC           (0x03UL<<CLK_CLKSEL1_CLKOSEL_Pos)   /*!< Select CLKO clock source from HIRC              */

#define CLK_TMR1_SRC_EXT            (0x00UL<<CLK_CLKSEL1_TMR1SEL_Pos)   /*!< Select Timer1 clock source from external crystal    */
#define CLK_TMR1_SRC_LIRC           (0x01UL<<CLK_CLKSEL1_TMR1SEL_Pos)   /*!< Select Timer1 clock source from LIRC                */
#define CLK_TMR1_SRC_HCLK           (0x02UL<<CLK_CLKSEL1_TMR1SEL_Pos)   /*!< Select Timer1 clock source from HCLK                */
#define CLK_TMR1_SRC_T1             (0x03UL<<CLK_CLKSEL1_TMR1SEL_Pos)   /*!< Select Timer1 clock source from T1                  */
#define CLK_TMR1_SRC_HIRC           (0x07UL<<CLK_CLKSEL1_TMR1SEL_Pos)   /*!< Select Timer1 clock source from HIRC                */

#define CLK_TMR0_SRC_EXT            (0x00UL<<CLK_CLKSEL1_TMR0SEL_Pos)   /*!< Select Timer0 clock source from external crystal    */
#define CLK_TMR0_SRC_LIRC           (0x01UL<<CLK_CLKSEL1_TMR0SEL_Pos)   /*!< Select Timer0 clock source from LIRC                */
#define CLK_TMR0_SRC_HCLK           (0x02UL<<CLK_CLKSEL1_TMR0SEL_Pos)   /*!< Select Timer0 clock source from HCLK                */
#define CLK_TMR0_SRC_T0             (0x03UL<<CLK_CLKSEL1_TMR0SEL_Pos)   /*!< Select Timer0 clock source from T0                  */
#define CLK_TMR0_SRC_HIRC           (0x07UL<<CLK_CLKSEL1_TMR0SEL_Pos)   /*!< Select Timer0 clock source from HIRC                */

#define CLK_EADC_SRC_EXT             (0x00UL<<CLK_CLKSEL1_ADCSEL_Pos)    /*!< Select EADC clock source from external crystal   */
#define CLK_EADC_SRC_HCLK            (0x02UL<<CLK_CLKSEL1_ADCSEL_Pos)    /*!< Select EADC clock source from HCLK               */
#define CLK_EADC_SRC_HIRC            (0x03UL<<CLK_CLKSEL1_ADCSEL_Pos)    /*!< Select EADC clock source from HIRC               */

#define CLK_WDT_SRC_EXT             (0x00UL<<CLK_CLKSEL1_WDTSEL_Pos)    /*!< Select WDT clock source from external crystal   */
#define CLK_WDT_SRC_HCLK_2048       (0x02UL<<CLK_CLKSEL1_WDTSEL_Pos)    /*!< Select WDT clock source from HCLK / 2048       */
#define CLK_WDT_SRC_LIRC            (0x03UL<<CLK_CLKSEL1_WDTSEL_Pos)    /*!< Select WDT clock source from LIRC               */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLK_CLKDIV constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV_HCLK(x)          (((x)-1) << CLK_CLKDIV_HCLKDIV_Pos) /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16  */
#define CLK_CLKDIV_EADC(x)          (((x)-1) << CLK_CLKDIV_ADCDIV_Pos)  /*!< CLKDIV Setting for EADC clock divider. It could be 1~256 */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_APBCLK(x)        ((x >>30) & 0x3)             /*!< Calculate AHBCLK/APBCLK offset address on MODULE index     */
#define MODULE_CLKSEL(x)        ((x >>27) & 0x7)             /*!< Calculate CLKSEL offset address on MODULE index            */
#define MODULE_CLKSEL_Msk(x)    ((1<<((x >>22) & 0x1F))-1)   /*!< Calculate CLKSEL mask offset on MODULE index               */
#define MODULE_CLKSEL_Pos(x)    ((x >>17) & 0x1f)            /*!< Calculate CLKSEL position offset on MODULE index           */
#define MODULE_CLKDIV(x)        ((x >>14) & 0x7)             /*!< Calculate CLKDIV position offset on MODULE index           */
#define MODULE_CLKDIV_Msk(x)    ((1<<((x >>10) & 0xF))-1)    /*!< Calculate CLKDIV mask offset on MODULE index               */
#define MODULE_CLKDIV_Pos(x)    ((x >>5 ) & 0x1f)            /*!< Calculate CLKDIV mask bit position offset on MODULE index  */
#define MODULE_IP_EN_Pos(x)     ((x >>0 ) & 0x1f)            /*!< Calculate clock enable bit position on MODULE index        */
#define MODULE_NoMsk            0x0                          /*!< Not mask on MODULE index                                   */

/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*  AHBCLK/APBCLK(2)           | CLKSEL_OFFSET(3) | CLKSEL_Msk bit no(5) | CLKSEL Bit Pos(5)           | CLKDIV_OFFSET(3) | CLKDIV_Msk bit no(4) | CLKDIV Bit Pos(5)        | IP_EN_Pos(5) */
/*  [31:30]                    | [29:27]            [26:22]                [21:17]                     | [16:14]            [13:10]                [9:5]                    | [4:0]        */
/*  CKEN OFFSET                | MODULE_NoMsk: Meaning no clock select.                                | MODULE_NoMsk: Meaning no clock divider                             |              */
/*  0: AHBCLK                  | 0: CLKSEL0         n: field bit number                                | 0: CLKDIV          n: field bit number                             |              */
/*  1: APBCLK                  | 1: CLKSEL1                                                            |                                                                    |              */
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#define HDIV_MODULE  ((0UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_AHBCLK_HDIVCKEN_Pos)   /*!< HDIV Module     */
#define ISP_MODULE   ((0UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_AHBCLK_ISPCKEN_Pos)    /*!< ISP Module      */

#define ACMP_MODULE  ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_ACMPCKEN_Pos)   /*!< ACMP Module     */
#define EADC_MODULE  ((1UL<<30)|(           1<<27)|(           2<<22)    |( CLK_CLKSEL1_ADCSEL_Pos<<17)|(           0<<14)|(           8<<10)    |(CLK_CLKDIV_ADCDIV_Pos<<5)|CLK_APBCLK_ADCCKEN_Pos)    /*!< EADC Module     */
#define USCI1_MODULE ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_USCI1CKEN_Pos)  /*!< USCI1 Module    */
#define USCI0_MODULE ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_USCI0CKEN_Pos)  /*!< USCI0 Module    */
#define BPWM_MODULE  ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_BPWMCKEN_Pos)   /*!< BPWM Module     */
#define EPWM_MODULE  ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_EPWMCKEN_Pos)   /*!< EPWM Module     */
#define PGA_MODULE   ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_PGACKEN_Pos)    /*!< PGA Module      */
#define ECAP_MODULE  ((1UL<<30)|(MODULE_NoMsk<<27)|(MODULE_NoMsk<<22)    |(           MODULE_NoMsk<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_ECAPCKEN_Pos)   /*!< ECAP Module     */
#define CLKO_MODULE  ((1UL<<30)|(           1<<27)|(           2<<22)    |(CLK_CLKSEL1_CLKOSEL_Pos<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_CLKOCKEN_Pos)   /*!< CLKO Module     */
#define TMR1_MODULE  ((1UL<<30)|(           1<<27)|(           3<<22)    |(CLK_CLKSEL1_TMR1SEL_Pos<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_TMR1CKEN_Pos)   /*!< TMR1 Module     */
#define TMR0_MODULE  ((1UL<<30)|(           1<<27)|(           3<<22)    |(CLK_CLKSEL1_TMR0SEL_Pos<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_TMR0CKEN_Pos)   /*!< TMR0 Module     */
#define WDT_MODULE   ((1UL<<30)|(           1<<27)|(           2<<22)    |( CLK_CLKSEL1_WDTSEL_Pos<<17)|(MODULE_NoMsk<<14)|(MODULE_NoMsk<<10)    |(         MODULE_NoMsk<<5)|CLK_APBCLK_WDTCKEN_Pos)    /*!< WDT Module      */

/*@}*/ /* end of group Mini57_CLK_EXPORTED_CONSTANTS */

/** @addtogroup Mini57_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetPCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SysTickDelay(uint32_t us);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);

/*@}*/ /* end of group Mini57_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_CLK_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __CLK_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
