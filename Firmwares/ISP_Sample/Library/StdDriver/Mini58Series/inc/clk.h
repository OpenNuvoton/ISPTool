/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 16 $
 * $Date: 15/06/05 9:38a $ 
 * @brief    Mini58 series CLK driver header file
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


/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_CLK_Driver CLK Driver
  @{
*/



/** @addtogroup Mini58_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

#define FREQ_25MHZ         25000000
#define FREQ_50MHZ         50000000
#define FREQ_72MHZ         72000000
#define FREQ_100MHZ        100000000
#define FREQ_200MHZ        200000000
#define FREQ_250MHZ        250000000
#define FREQ_500MHZ        500000000


/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCTL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRCTL_XTL12M              0x01UL /*!< Setting External Crystal Oscillator as 12MHz         */
#define CLK_PWRCTL_XTLEN_HXT           0x01UL /*!< Setting External Crystal Oscillator as 12MHz         */
#define CLK_PWRCTL_XTL32K              0x02UL /*!< Setting External Crystal Oscillator as 32KHz         */
#define CLK_PWRCTL_XTLEN_LXT           0x02UL /*!< Setting External Crystal Oscillator as 32KHz         */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_XTAL          0x00UL /*!< Setting clock source as external XTAL */
#define CLK_CLKSEL0_HCLKSEL_HXT           0x00UL /*!< Setting clock source as external HXT */
#define CLK_CLKSEL0_HCLKSEL_LXT           0x00UL /*!< Setting clock source as external LXT */
#define CLK_CLKSEL0_HCLKSEL_PLL           0x02UL /*!< Setting clock source as PLL */
#define CLK_CLKSEL0_HCLKSEL_LIRC          0x03UL /*!< Setting clock source as internal 10KHz RC clock */
#define CLK_CLKSEL0_HCLKSEL_HIRC          0x07UL /*!< Setting clock source as internal RC clock */
#define CLK_CLKSEL0_STCLKSEL_XTAL         0x00UL /*!< Setting clock source as external XTAL */ 
#define CLK_CLKSEL0_STCLKSEL_XTAL_DIV2    0x10UL /*!< Setting clock source as external XTAL/2 */
#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV2    0x18UL /*!< Setting clock source as HCLK/2 */
#define CLK_CLKSEL0_STCLKSEL_HIRC_DIV2    0x38UL /*!< Setting clock source as internal RC clock/2 */
#define CLK_CLKSEL0_STCLKSEL_HCLK         0x08UL /*!< Setting clock source as HCLK */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_XTAL          0x00000000UL /*!< Setting WDT clock source as external XTAL */ 
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  0x00000002UL /*!< Setting WDT clock source as HCLK/2048 */
#define CLK_CLKSEL1_WDTSEL_IRC10K        0x00000003UL /*!< Setting WDT clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_WDTSEL_LIRC          0x00000003UL /*!< Setting WDT clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_ADCSEL_XTAL          0x00000000UL /*!< Setting ADC clock source as external XTAL */
#define CLK_CLKSEL1_ADCSEL_PLL           0x00000004UL /*!< Setting ADC clock source as PLL */
#define CLK_CLKSEL1_ADCSEL_HCLK          0x00000008UL /*!< Setting ADC clock source as HCLK */
#define CLK_CLKSEL1_ADCSEL_HIRC          0x0000000CUL /*!< Setting ADC clock source as internal RC clock */
#define CLK_CLKSEL1_SPISEL_XTAL          0x00000000UL /*!< Setting SPI clock source as HXT or LXT */
#define CLK_CLKSEL1_SPISEL_HCLK          0x00000010UL /*!< Setting SPI clock source as HCLK */
#define CLK_CLKSEL1_SPISEL_PLL           0x00000020UL /*!< Setting SPI clock source as PLL */
#define CLK_CLKSEL1_TMR0SEL_XTAL         0x00000000UL /*!< Setting Timer 0 clock source as external XTAL */
#define CLK_CLKSEL1_TMR0SEL_LIRC         0x00000100UL /*!< Setting Timer 0 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR0SEL_HCLK         0x00000200UL /*!< Setting Timer 0 clock source as HCLK */
#define CLK_CLKSEL1_TMR0SEL_TM0          0x00000300UL /*!< Setting Timer 0 clock source as external trigger */
#define CLK_CLKSEL1_TMR0SEL_HIRC         0x00000700UL /*!< Setting Timer 0 clock source as internal RC clock */
#define CLK_CLKSEL1_TMR1SEL_XTAL         0x00000000UL /*!< Setting Timer 1 clock source as external XTAL */
#define CLK_CLKSEL1_TMR1SEL_LIRC         0x00001000UL /*!< Setting Timer 1 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR1SEL_HCLK         0x00002000UL /*!< Setting Timer 1 clock source as HCLK */
#define CLK_CLKSEL1_TMR1SEL_TM1          0x00003000UL /*!< Setting Timer 1 clock source as external trigger */
#define CLK_CLKSEL1_TMR1SEL_HIRC         0x00007000UL /*!< Setting Timer 1 clock source as internal RC clock */
#define CLK_CLKSEL1_UARTSEL_XTAL         0x00000000UL /*!< Setting UART clock source as external XTAL */
#define CLK_CLKSEL1_UARTSEL_PLL          0x01000000UL /*!< Setting UART clock source as external PLL */
#define CLK_CLKSEL1_UARTSEL_HIRC         0x02000000UL /*!< Setting UART clock source as external internal RC clock */
#define CLK_CLKSEL1_PWMCH01SEL_HCLK      0x20000000UL /*!< Setting PWM01 clock source as external HCLK */
#define CLK_CLKSEL1_PWMCH23SEL_HCLK      0x80000000UL /*!< Setting PWM23 clock source as external HCLK */


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_CLKOSEL_XTAL        0x00000000UL /*!< Setting CLKODIV clock source as external XTAL */ 
#define CLK_CLKSEL2_CLKOSEL_HXT         0x00000000UL /*!< Setting CLKODIV clock source as external XTAL */ 
#define CLK_CLKSEL2_CLKOSEL_LXT         0x00000000UL /*!< Setting CLKODIV clock source as external XTAL */ 
#define CLK_CLKSEL2_CLKOSEL_LIRC        0x00000004UL /*!< Setting CLKODIV clock source as LIRC */ 
#define CLK_CLKSEL2_CLKOSEL_HCLK        0x00000008UL /*!< Setting CLKODIV clock source as HCLK */
#define CLK_CLKSEL2_CLKOSEL_HIRC        0x0000000CUL /*!< Setting CLKODIV clock source as internal RC clock */
#define CLK_CLKSEL2_PWMCH45SEL_HCLK     0x00000020UL /*!< Setting PWMCH45 clock source as HCLK */
#define CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048   0x00020000UL /*!< Setting WWDT clock source as HCLK/2048 */ 
#define CLK_CLKSEL2_WWDTSEL_LIRC           0x00030000UL /*!< Setting WWDT clock source as internal RC clock */ 


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV_ADC(x)  (((x)-1) << 16) /*!< CLKDIV Setting for ADC clock divider. It could be 1~256 */ 
#define CLK_CLKDIV_UART(x) (((x)-1) <<  8) /*!< CLKDIV Setting for UART clock divider. It could be 1~16 */ 
#define CLK_CLKDIV_HCLK(x)  ((x)-1)        /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16 */ 

/*---------------------------------------------------------------------------------------------------------*/
/*  PLLCTL constant definitions. PLL = FIN * NF / NR / NO                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PLLCTL_PLLSRC_HXT   0x00000000UL    /*!< For PLL clock source is HXT.  3.2MHz < FIN < 150MHz */
#define CLK_PLLCTL_PLLSRC_HIRC  0x00080000UL    /*!< For PLL clock source is HIRC. 3.2MHz < FIN < 150MHz */

#define CLK_PLLCTL_NF(x)        ((x)-2)         /*!< x must be constant and 2 <= x <= 513. 200MHz < FIN*NF/NR < 500MHz. (FIN*NF/NR > 250MHz is preferred.) */
#define CLK_PLLCTL_NR(x)        (((x)-2)<<9)    /*!< x must be constant and 2 <= x <= 33.  1.6MHz < FIN/NR < 16MHz */

#define CLK_PLLCTL_NO_1         0x0000UL        /*!< For output divider is 1 */
#define CLK_PLLCTL_NO_2         0x4000UL        /*!< For output divider is 2 */
#define CLK_PLLCTL_NO_4         0xC000UL        /*!< For output divider is 4 */

#define CLK_PLLCTL_72MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 24) | CLK_PLLCTL_NO_2) /*!< Predefined PLLCTL setting for  72MHz PLL output with HXT(12MHz X'tal) */
#define CLK_PLLCTL_96MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 32) | CLK_PLLCTL_NO_2) /*!< Predefined PLLCTL setting for  96MHz PLL output with HXT(12MHz X'tal) */
#define CLK_PLLCTL_100MHz_HXT  (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF( 50) | CLK_PLLCTL_NO_2) /*!< Predefined PLLCTL setting for 100MHz PLL output with HXT(12MHz X'tal) */	

#define CLK_PLLCTL_72MHz_HIRC  (CLK_PLLCTL_PLLSRC_HIRC | CLK_PLLCTL_NR(4) | CLK_PLLCTL_NF( 26) | CLK_PLLCTL_NO_2) /*!< Predefined PLLCTL setting for 71.884800MHz PLL output with HIRC(22.1184MHz X'tal) */
#define CLK_PLLCTL_96MHz_HIRC  (CLK_PLLCTL_PLLSRC_HIRC | CLK_PLLCTL_NR(13)| CLK_PLLCTL_NF(113) | CLK_PLLCTL_NO_2) /*!< Predefined PLLCTL setting for 96.129968MHz PLL output with HIRC(22.1184MHz X'tal) */
#define CLK_PLLCTL_100MHz_HIRC (CLK_PLLCTL_PLLSRC_HIRC | CLK_PLLCTL_NR(4) | CLK_PLLCTL_NF( 36) | CLK_PLLCTL_NO_2) /*!< Predefined PLLCTL setting for 99.532800MHz PLL output with HIRC(22.1184MHz X'tal) */

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
#define ISP_MODULE       (( 0UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 2<<0)) /*!< ISP Module  \hideinitializer */
#define WDT_MODULE       (( 1UL<<31)|( 1<<29)|(            3<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< WDT Module  \hideinitializer */
#define TMR0_MODULE      (( 1UL<<31)|( 1<<29)|(            7<<25)|( 8<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 2<<0)) /*!< TMR0 Module  \hideinitializer */
#define TMR1_MODULE      (( 1UL<<31)|( 1<<29)|(            7<<25)|(12<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 3<<0)) /*!< TMR1 Module  \hideinitializer */
#define CLKO_MODULE      (( 1UL<<31)|( 3<<29)|(            3<<25)|( 2<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 6<<0)) /*!< CLKO Module  \hideinitializer */
#define I2C0_MODULE      (( 1UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 8<<0)) /*!< I2C0 Module  \hideinitializer */
#define I2C1_MODULE      (( 1UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 9<<0)) /*!< I2C1 Module  \hideinitializer */
#define SPI0_MODULE      (( 1UL<<31)|( 1<<29)|(            3<<25)|( 4<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(12<<0)) /*!< SPI Module  \hideinitializer */
#define UART0_MODULE     (( 1UL<<31)|( 1<<29)|(            3<<25)|(24<<20)|( 0<<18)|(          0xF<<10)|( 8<<5)|(16<<0)) /*!< UART0 Module  \hideinitializer */
#define UART1_MODULE     (( 1UL<<31)|( 1<<29)|(            3<<25)|(24<<20)|( 0<<18)|(          0xF<<10)|( 8<<5)|(17<<0)) /*!< UART1 Module  \hideinitializer */
#define PWMCH01_MODULE   (( 1UL<<31)|( 1<<29)|(            3<<25)|(28<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(20<<0)) /*!< PWMCH01 Module  \hideinitializer */
#define PWMCH23_MODULE   (( 1UL<<31)|( 1<<29)|(            3<<25)|(30<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(21<<0)) /*!< PWMCH23 Module  \hideinitializer */
#define PWMCH45_MODULE   (( 1UL<<31)|( 3<<29)|(            3<<25)|( 4<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(22<<0)) /*!< PWMCH45 Module  \hideinitializer */
#define ADC_MODULE       (( 1UL<<31)|( 1<<29)|(            3<<25)|( 2<<20)|( 0<<18)|(         0xFF<<10)|(16<<5)|(28<<0)) /*!< ADC Module  \hideinitializer */
#define ACMP_MODULE      (( 1UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(30<<0)) /*!< ACMP Module  \hideinitializer */
#define WWDT_MODULE      (( 1UL<<31)|( 3<<29)|(            3<<25)|(16<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< WWDT Module  \hideinitializer */



/*@}*/ /* end of group Mini58_CLK_EXPORTED_CONSTANTS */


/** @addtogroup Mini58_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Get PLL clock frequency
  * @param      None
  * @return     PLL frequency
  * @details    This function get PLL frequency. The frequency unit is Hz.
  */
__STATIC_INLINE uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq = 0, u32PllReg;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};

    u32PllReg = CLK->PLLCTL;

    if(u32PllReg & (CLK_PLLCTL_PD_Msk | CLK_PLLCTL_OE_Msk))
        return 0;           /* PLL is in power down mode or fix low */

    if(u32PllReg & CLK_PLLCTL_PLLSRC_HIRC)
        u32FIN = __HIRC;    /* PLL source clock from HIRC */
    else
        u32FIN = __XTAL;     /* PLL source clock from HXT */

    if(u32PllReg & CLK_PLLCTL_BP_Msk)
        return u32FIN;      /* PLL is in bypass mode */

    /* PLL is output enabled in normal work mode */
    u32NO = au8NoTbl[((u32PllReg & CLK_PLLCTL_OUTDIV_Msk) >> CLK_PLLCTL_OUTDIV_Pos)];
    u32NF = ((u32PllReg & CLK_PLLCTL_FBDIV_Msk) >> CLK_PLLCTL_FBDIV_Pos) + 2;
    u32NR = ((u32PllReg & CLK_PLLCTL_INDIV_Msk) >> CLK_PLLCTL_INDIV_Pos) + 2;

    /* u32FIN is shifted 2 bits to avoid overflow */
    u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);

    return u32PllFreq;
}

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SysTickDelay(uint32_t us);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);



/*@}*/ /* end of group Mini58_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_CLK_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
