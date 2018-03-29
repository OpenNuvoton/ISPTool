/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 14 $
 * $Date: 16/03/17 1:27p $
 * @brief    Nano 103 CLK driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Nano103.h"
/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup NANO103_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief  This function disable frequency output function.
  * @param  None
  * @return None
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK->APBCLK &= (~CLK_APBCLK_CLKOCKEN_Msk);
}

/**
  * @brief  This function enable frequency divider module clock,
  *         enable frequency divider clock function and configure frequency divider.
  * @param  u32ClkSrc is frequency divider function clock source
  *         - \ref CLK_CLKSEL2_CLKOSEL_HXT
  *         - \ref CLK_CLKSEL2_CLKOSEL_LXT
  *         - \ref CLK_CLKSEL2_CLKOSEL_HCLK
  *         - \ref CLK_CLKSEL2_CLKOSEL_HIRC
  *         - \ref CLK_CLKSEL2_CLKOSEL_MIRC
  * @param  u32ClkDiv is divider output frequency selection.
  * @param  u32ClkDivBy1En is frequency divided by one enable.
  * @return None
  *
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv.
  *             The formula is:
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1)
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves.
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* Select CKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | u32ClkSrc;

    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv | (u32ClkDivBy1En<<CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
    CLK->APBCLK |= CLK_APBCLK_CLKOCKEN_Msk;
}


/**
  * @brief  This function let system enter to fractal fx-2-down mode.
  * @param  None
  * @return None
  */
void CLK_PowerDown(void)
{

    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKDLY_Msk);
    __WFI();

}

/**
  * @brief  This function let system enter to Idle mode
  * @param  None
  * @return None
  */
void CLK_Idle(void)
{
    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk);
    __WFI();
}

/**
  * @brief  This function get external high frequency crystal frequency. The frequency unit is Hz.
  * @param  None
  * @return None
  */
uint32_t CLK_GetHXTFreq(void)
{
    if(CLK->PWRCTL & CLK_PWRCTL_HXT_EN )
        return __HXT;
    else
        return 0;
}

/**
  * @brief  This function get external low frequency crystal frequency. The frequency unit is Hz.
  * @param  None
  * @return LXT frequency
  */
uint32_t CLK_GetLXTFreq(void)
{
    if(CLK->PWRCTL & CLK_PWRCTL_LXT_EN )
        return __LXT;
    else
        return 0;
}

/**
  * @brief  This function get HCLK frequency. The frequency unit is Hz.
  * @param  None
  * @return HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief  This function get PCLK0 frequency. The frequency unit is Hz.
  * @param  None
  * @return PCLK0 frequency
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    uint32_t Div[]= {1,2,4,8,16,1,1,1};
    uint32_t PCLK_Div;
    PCLK_Div = CLK->APBDIV & CLK_APBDIV_APB0DIV_Msk;
    SystemCoreClockUpdate();
    return SystemCoreClock/Div[PCLK_Div];
}

/**
  * @brief  This function get PCLK1 frequency. The frequency unit is Hz.
  * @param  None
  * @return PCLK1 frequency
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t Div[]= {1,2,4,8,16,1,1,1};
    uint32_t PCLK_Div;
    PCLK_Div = CLK->APBDIV & CLK_APBDIV_APB1DIV_Msk;
    SystemCoreClockUpdate();
    return SystemCoreClock/Div[PCLK_Div];
}

/**
  * @brief  This function get CPU frequency. The frequency unit is Hz.
  * @param  None
  * @return CPU frequency
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief  This function get PLL frequency. The frequency unit is Hz.
  * @param  None
  * @return PLL frequency
  */
uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32Freq =0, u32PLLSrc;
    uint32_t u32SRC_N,u32PLL_M,u32PllReg;

    u32PllReg = CLK->PLLCTL;

    if (u32PllReg & CLK_PLLCTL_PD)
        return 0;    /* PLL is in power down mode */

    if((u32PllReg & CLK_PLLCTL_PLLSRC_Msk) == CLK_PLLCTL_PLL_SRC_HXT)
    {
        /* PLL source clock from HXT */
        u32PLLSrc = __HXT;
    }
    else if((u32PllReg & CLK_PLLCTL_PLLSRC_Msk) == CLK_PLLCTL_PLL_SRC_HIRC)
    {
        /* HIRC Source Selection */
        if(CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)
        {
            /* Clock source from HIRC1 (36MHz) */
            u32PLLSrc =__HIRC36M;
        }
        else
        {
            /* Clock source from HIRC0 (12MHz) */
            if(CLK->PWRCTL & CLK_PWRCTL_HIRC0FSEL_Msk)
                u32PLLSrc =__HIRC16M;
            else
                u32PLLSrc =__HIRC12M;
        }
    }
    else
    {
        /* PLL source clock from MIRC (4MHz) */
        u32PLLSrc =__MIRC;
    }

    u32SRC_N = (u32PllReg & CLK_PLLCTL_INDIV_Msk) >> CLK_PLLCTL_INDIV_Pos;
    u32PLL_M = (u32PllReg & CLK_PLLCTL_PLLMLP_Msk) >> CLK_PLLCTL_PLLMLP_Pos;

    u32Freq = u32PLLSrc * u32PLL_M / (u32SRC_N+1);

    return u32Freq;
}

/**
  * @brief  This function set HCLK frequency. The frequency unit is Hz. The range of u32Hclk is 16 ~ 48 MHz
  * @param[in]  u32Hclk is HCLK frequency
  * @return None
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    if(CLK->PWRCTL & CLK_PWRCTL_HXT_EN)
        CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HXT, u32Hclk);
    else if(CLK->PWRCTL & (CLK_PWRCTL_HIRC0_EN | CLK_PWRCTL_HIRC1_EN))
        CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HIRC, u32Hclk);
    else
        CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_MIRC, u32Hclk);

    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_HCLK_CLK_DIVIDER(1));
    return SystemCoreClock;
}

/**
  * @brief  This function set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *         - \ref CLK_CLKSEL0_HCLKSEL_HXT
  *         - \ref CLK_CLKSEL0_HCLKSEL_LXT
  *         - \ref CLK_CLKSEL0_HCLKSEL_PLL
  *         - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *         - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  *         - \ref CLK_CLKSEL0_HCLKSEL_HIRC0
  *         - \ref CLK_CLKSEL0_HCLKSEL_HIRC1
  *         - \ref CLK_CLKSEL0_HCLKSEL_MIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *         - \ref CLK_HCLK_CLK_DIVIDER(x)
  * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | u32ClkDiv;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~(CLK_CLKSEL0_HIRCSEL_Msk | CLK_CLKSEL0_HCLKSEL_Msk)) | u32ClkSrc;
    SystemCoreClockUpdate();
}

/**
  * @brief  This function set APB PCLK0 clock divider
  * @param[in]  u32ClkDiv is PCLK0 clock divider. Including :
  *         - \ref CLK_APB0DIV_HCLK
  *         - \ref CLK_APB0DIV_1_2HCLK
  *         - \ref CLK_APB0DIV_1_4HCLK
  *         - \ref CLK_APB0DIV_1_8HCLK
  *         - \ref CLK_APB0DIV_1_16HCLK
  * @return None
  */
void CLK_SetPCLK0(uint32_t u32ClkDiv)
{
    CLK->APBDIV = (CLK->APBDIV & ~CLK_APBDIV_APB0DIV_Msk) | u32ClkDiv;
}

/**
  * @brief  This function set APB PCLK1 clock divider
  * @param[in]  u32ClkDiv is PCLK1 clock divider. Including :
  *         - \ref CLK_APB1DIV_HCLK
  *         - \ref CLK_APB1DIV_1_2HCLK
  *         - \ref CLK_APB1DIV_1_4HCLK
  *         - \ref CLK_APB1DIV_1_8HCLK
  *         - \ref CLK_APB1DIV_1_16HCLK
  * @return None
  */
void CLK_SetPCLK1(uint32_t u32ClkDiv)
{
    CLK->APBDIV = (CLK->APBDIV & ~CLK_APBDIV_APB1DIV_Msk) | u32ClkDiv;
}

/**
  * @brief  This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return None
  * @details Valid parameter combinations listed in following table:
  *-
  * |Module index         |Clock source                            |Divider                                 |
  * | :------------------ | :------------------------------------  | :------------------------------------  |
  * |\ref ISP_MODULE      |\ref CLK_CLKSEL0_ISPSEL_HIRC            | x                                      |
  * |\ref ISP_MODULE      |\ref CLK_CLKSEL0_ISPSEL_MIRC            | x                                      |
  * |\ref UART0_MODULE    |\ref CLK_CLKSEL1_UART0SEL_HXT           |\ref CLK_UART0_CLK_DIVIDER(x)           |
  * |\ref UART0_MODULE    |\ref CLK_CLKSEL1_UART0SEL_LXT           |\ref CLK_UART0_CLK_DIVIDER(x)           |
  * |\ref UART0_MODULE    |\ref CLK_CLKSEL1_UART0SEL_PLL           |\ref CLK_UART0_CLK_DIVIDER(x)           |
  * |\ref UART0_MODULE    |\ref CLK_CLKSEL1_UART0SEL_HIRC          |\ref CLK_UART0_CLK_DIVIDER(x)           |
  * |\ref UART0_MODULE    |\ref CLK_CLKSEL1_UART0SEL_MIRC          |\ref CLK_UART0_CLK_DIVIDER(x)           |
  * |\ref PWM0_MODULE     |\ref CLK_CLKSEL1_PWM0SEL_PLL            | x                                      |
  * |\ref PWM0_MODULE     |\ref CLK_CLKSEL1_PWM0SEL_PCLK0          | x                                      |
  * |\ref TMR0_MODULE     |\ref CLK_CLKSEL1_TMR0SEL_HXT            |\ref CLK_TMR0_CLK_DIVIDER(x)            |
  * |\ref TMR0_MODULE     |\ref CLK_CLKSEL1_TMR0SEL_LXT            |\ref CLK_TMR0_CLK_DIVIDER(x)            |
  * |\ref TMR0_MODULE     |\ref CLK_CLKSEL1_TMR0SEL_LIRC           |\ref CLK_TMR0_CLK_DIVIDER(x)            |
  * |\ref TMR0_MODULE     |\ref CLK_CLKSEL1_TMR0SEL_HIRC           |\ref CLK_TMR0_CLK_DIVIDER(x)            |
  * |\ref TMR0_MODULE     |\ref CLK_CLKSEL1_TMR0SEL_MIRC           |\ref CLK_TMR0_CLK_DIVIDER(x)            |
  * |\ref TMR0_MODULE     |\ref CLK_CLKSEL1_TMR0SEL_EXT            |\ref CLK_TMR0_CLK_DIVIDER(x)            |
  * |\ref TMR1_MODULE     |\ref CLK_CLKSEL1_TMR1SEL_HXT            |\ref CLK_TMR1_CLK_DIVIDER(x)            |
  * |\ref TMR1_MODULE     |\ref CLK_CLKSEL1_TMR1SEL_LXT            |\ref CLK_TMR1_CLK_DIVIDER(x)            |
  * |\ref TMR1_MODULE     |\ref CLK_CLKSEL1_TMR1SEL_LIRC           |\ref CLK_TMR1_CLK_DIVIDER(x)            |
  * |\ref TMR1_MODULE     |\ref CLK_CLKSEL1_TMR1SEL_HIRC           |\ref CLK_TMR1_CLK_DIVIDER(x)            |
  * |\ref TMR1_MODULE     |\ref CLK_CLKSEL1_TMR1SEL_MIRC           |\ref CLK_TMR1_CLK_DIVIDER(x)            |
  * |\ref TMR1_MODULE     |\ref CLK_CLKSEL1_TMR1SEL_EXT            |\ref CLK_TMR1_CLK_DIVIDER(x)            |
  * |\ref ADC_MODULE      |\ref CLK_CLKSEL1_ADCSEL_HXT             |\ref CLK_ADC_CLK_DIVIDER(x)             |
  * |\ref ADC_MODULE      |\ref CLK_CLKSEL1_ADCSEL_LXT             |\ref CLK_ADC_CLK_DIVIDER(x)             |
  * |\ref ADC_MODULE      |\ref CLK_CLKSEL1_ADCSEL_PLL             |\ref CLK_ADC_CLK_DIVIDER(x)             |
  * |\ref ADC_MODULE      |\ref CLK_CLKSEL1_ADCSEL_HIRC            |\ref CLK_ADC_CLK_DIVIDER(x)             |
  * |\ref ADC_MODULE      |\ref CLK_CLKSEL1_ADCSEL_MIRC            |\ref CLK_ADC_CLK_DIVIDER(x)             |
  * |\ref ADC_MODULE      |\ref CLK_CLKSEL1_ADCSEL_HCLK            |\ref CLK_ADC_CLK_DIVIDER(x)             |
  * |\ref SPI0_MODULE     |\ref CLK_CLKSEL1_SPI0SEL_HXT            | x                                      |
  * |\ref SPI0_MODULE     |\ref CLK_CLKSEL1_SPI0SEL_PLL            | x                                      |
  * |\ref SPI0_MODULE     |\ref CLK_CLKSEL1_SPI0SEL_HIRC           | x                                      |
  * |\ref SPI0_MODULE     |\ref CLK_CLKSEL1_SPI0SEL_HCLK           | x                                      |
  * |\ref SPI2_MODULE     |\ref CLK_CLKSEL1_SPI2SEL_HXT            | x                                      |
  * |\ref SPI2_MODULE     |\ref CLK_CLKSEL1_SPI2SEL_PLL            | x                                      |
  * |\ref SPI2_MODULE     |\ref CLK_CLKSEL1_SPI2SEL_HIRC           | x                                      |
  * |\ref SPI2_MODULE     |\ref CLK_CLKSEL1_SPI2SEL_HCLK           | x                                      |
  * |\ref WDT_MODULE      |\ref CLK_CLKSEL1_WDTSEL_LXT             | x                                      |
  * |\ref WDT_MODULE      |\ref CLK_CLKSEL1_WDTSEL_LIRC            | x                                      |
  * |\ref WDT_MODULE      |\ref CLK_CLKSEL1_WDTSEL_HCLKDIV2048     | x                                      |
  * |\ref WWDT_MODULE     |\ref CLK_CLKSEL1_WWDTSEL_LIRC           | x                                      |
  * |\ref WWDT_MODULE     |\ref CLK_CLKSEL1_WWDTSEL_HCLKDIV2048    | x                                      |
  * |\ref UART1_MODULE    |\ref CLK_CLKSEL2_UART1SEL_HXT           |\ref CLK_UART1_CLK_DIVIDER(x)           |
  * |\ref UART1_MODULE    |\ref CLK_CLKSEL2_UART1SEL_LXT           |\ref CLK_UART1_CLK_DIVIDER(x)           |
  * |\ref UART1_MODULE    |\ref CLK_CLKSEL2_UART1SEL_PLL           |\ref CLK_UART1_CLK_DIVIDER(x)           |
  * |\ref UART1_MODULE    |\ref CLK_CLKSEL2_UART1SEL_HIRC          |\ref CLK_UART1_CLK_DIVIDER(x)           |
  * |\ref UART1_MODULE    |\ref CLK_CLKSEL2_UART1SEL_MIRC          |\ref CLK_UART1_CLK_DIVIDER(x)           |
  * |\ref CLKO_MODULE     |\ref CLK_CLKSEL2_CLKOSEL_HXT            | x                                      |
  * |\ref CLKO_MODULE     |\ref CLK_CLKSEL2_CLKOSEL_LXT            | x                                      |
  * |\ref CLKO_MODULE     |\ref CLK_CLKSEL2_CLKOSEL_HIRC           | x                                      |
  * |\ref CLKO_MODULE     |\ref CLK_CLKSEL2_CLKOSEL_HIRC           | x                                      |
  * |\ref CLKO_MODULE     |\ref CLK_CLKSEL2_CLKOSEL_HCLK           | x                                      |
  * |\ref TMR2_MODULE     |\ref CLK_CLKSEL2_TMR2SEL_HXT            |\ref CLK_TMR2_CLK_DIVIDER(x)            |
  * |\ref TMR2_MODULE     |\ref CLK_CLKSEL2_TMR2SEL_LXT            |\ref CLK_TMR2_CLK_DIVIDER(x)            |
  * |\ref TMR2_MODULE     |\ref CLK_CLKSEL2_TMR2SEL_LIRC           |\ref CLK_TMR2_CLK_DIVIDER(x)            |
  * |\ref TMR2_MODULE     |\ref CLK_CLKSEL2_TMR2SEL_HIRC           |\ref CLK_TMR2_CLK_DIVIDER(x)            |
  * |\ref TMR2_MODULE     |\ref CLK_CLKSEL2_TMR2SEL_MIRC           |\ref CLK_TMR2_CLK_DIVIDER(x)            |
  * |\ref TMR2_MODULE     |\ref CLK_CLKSEL2_TMR2SEL_EXT            |\ref CLK_TMR2_CLK_DIVIDER(x)            |
  * |\ref TMR3_MODULE     |\ref CLK_CLKSEL2_TMR3SEL_HXT            |\ref CLK_TMR3_CLK_DIVIDER(x)            |
  * |\ref TMR3_MODULE     |\ref CLK_CLKSEL2_TMR3SEL_LXT            |\ref CLK_TMR3_CLK_DIVIDER(x)            |
  * |\ref TMR3_MODULE     |\ref CLK_CLKSEL2_TMR3SEL_LIRC           |\ref CLK_TMR3_CLK_DIVIDER(x)            |
  * |\ref TMR3_MODULE     |\ref CLK_CLKSEL2_TMR3SEL_HIRC           |\ref CLK_TMR3_CLK_DIVIDER(x)            |
  * |\ref TMR3_MODULE     |\ref CLK_CLKSEL2_TMR3SEL_MIRC           |\ref CLK_TMR3_CLK_DIVIDER(x)            |
  * |\ref TMR3_MODULE     |\ref CLK_CLKSEL2_TMR3SEL_EXT            |\ref CLK_TMR3_CLK_DIVIDER(x)            |
  * |\ref SC0_MODULE      |\ref CLK_CLKSEL2_SC0SEL_HXT             |\ref CLK_SC0_CLK_DIVIDER(x)             |
  * |\ref SC0_MODULE      |\ref CLK_CLKSEL2_SC0SEL_PLL             |\ref CLK_SC0_CLK_DIVIDER(x)             |
  * |\ref SC0_MODULE      |\ref CLK_CLKSEL2_SC0SEL_HIRC            |\ref CLK_SC0_CLK_DIVIDER(x)             |
  * |\ref SC0_MODULE      |\ref CLK_CLKSEL2_SC0SEL_MIRC            |\ref CLK_SC0_CLK_DIVIDER(x)             |
  * |\ref SC0_MODULE      |\ref CLK_CLKSEL2_SC0SEL_HCLK            |\ref CLK_SC0_CLK_DIVIDER(x)             |
  * |\ref SPI1_MODULE     |\ref CLK_CLKSEL2_SPI1SEL_HXT            | x                                      |
  * |\ref SPI1_MODULE     |\ref CLK_CLKSEL2_SPI1SEL_PLL            | x                                      |
  * |\ref SPI1_MODULE     |\ref CLK_CLKSEL2_SPI1SEL_HIRC           | x                                      |
  * |\ref SPI1_MODULE     |\ref CLK_CLKSEL2_SPI1SEL_HCLK           | x                                      |
  * |\ref SPI3_MODULE     |\ref CLK_CLKSEL2_SPI3SEL_HXT            | x                                      |
  * |\ref SPI3_MODULE     |\ref CLK_CLKSEL2_SPI3SEL_PLL            | x                                      |
  * |\ref SPI3_MODULE     |\ref CLK_CLKSEL2_SPI3SEL_HIRC           | x                                      |
  * |\ref SPI3_MODULE     |\ref CLK_CLKSEL2_SPI3SEL_HCLK           | x                                      |
  */

void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32tmp=0,u32sel=0,u32div=0;

    if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk)
    {
        u32div =(uint32_t)&CLK->CLKDIV0+((MODULE_CLKDIV(u32ModuleIdx))*4);
        u32tmp = *(volatile uint32_t *)(u32div);
        u32tmp = ( u32tmp & ~(MODULE_CLKDIV_Msk(u32ModuleIdx)<<MODULE_CLKDIV_Pos(u32ModuleIdx)) ) | u32ClkDiv;
        *(volatile uint32_t *)(u32div) = u32tmp;
    }

    if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk)
    {
        u32sel = (uint32_t)&CLK->CLKSEL0+((MODULE_CLKSEL(u32ModuleIdx))*4);
        u32tmp = *(volatile uint32_t *)(u32sel);
        u32tmp = ( u32tmp & ~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)) ) | u32ClkSrc;
        *(volatile uint32_t *)(u32sel) = u32tmp;
    }
}

/**
  * @brief  This function enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including:
  *         - \ref CLK_PWRCTL_HXTEN_Msk
  *         - \ref CLK_PWRCTL_LXTEN_Msk
  *         - \ref CLK_PWRCTL_HIRC0EN_Msk
  *         - \ref CLK_PWRCTL_LIRCEN_Msk
  *         - \ref CLK_PWRCTL_HIRC1EN_Msk
  *         - \ref CLK_PWRCTL_MIRCEN_Msk
  * @return None
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief  This function disable clock source
  * @param  u32ClkMask is clock source mask. Including:
  *         - \ref CLK_PWRCTL_HXTEN_Msk
  *         - \ref CLK_PWRCTL_LXTEN_Msk
  *         - \ref CLK_PWRCTL_HIRC0EN_Msk
  *         - \ref CLK_PWRCTL_LIRCEN_Msk
  *         - \ref CLK_PWRCTL_HIRC1EN_Msk
  *         - \ref CLK_PWRCTL_MIRCEN_Msk
  * @return None
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~u32ClkMask;
}

/**
  * @brief  This function enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *         - \ref GPIO_MODULE
  *         - \ref PDMA_MODULE
  *         - \ref ISP_MODULE
  *         - \ref SRAM_MODULE
  *         - \ref STC_MODULE
  *         - \ref WDT_MODULE
  *         - \ref WWDT_MODULE
  *         - \ref RTC_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref TMR2_MODULE
  *         - \ref TMR3_MODULE
  *         - \ref CLKO_MODULE
  *         - \ref I2C0_MODULE
  *         - \ref I2C1_MODULE
  *         - \ref ACMP0_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref SPI1_MODULE
  *         - \ref SPI2_MODULE
  *         - \ref SPI3_MODULE
  *         - \ref UART0_MODULE
  *         - \ref UART1_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref ADC_MODULE
  *         - \ref SC0_MODULE
  *         - \ref SC1_MODULE
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief  This function disable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *         - \ref GPIO_MODULE
  *         - \ref PDMA_MODULE
  *         - \ref ISP_MODULE
  *         - \ref SRAM_MODULE
  *         - \ref STC_MODULE
  *         - \ref WDT_MODULE
  *         - \ref WWDT_MODULE
  *         - \ref RTC_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref TMR2_MODULE
  *         - \ref TMR3_MODULE
  *         - \ref CLKO_MODULE
  *         - \ref I2C0_MODULE
  *         - \ref I2C1_MODULE
  *         - \ref ACMP0_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref SPI1_MODULE
  *         - \ref SPI2_MODULE
  *         - \ref SPI3_MODULE
  *         - \ref UART0_MODULE
  *         - \ref UART1_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref ADC_MODULE
  *         - \ref SC0_MODULE
  *         - \ref SC1_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief  This function set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *         - \ref CLK_PLLCTL_PLL_SRC_MIRC
  *         - \ref CLK_PLLCTL_PLL_SRC_HIRC
  *         - \ref CLK_PLLCTL_PLL_SRC_HXT
  * @param[in]  u32PllFreq is PLL frequency
  * @return None
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    uint32_t u32PllCr,u32PLL_N,u32PLL_M,u32PLLReg;
    if ( u32PllFreq < FREQ_16MHZ)
        u32PllFreq=FREQ_16MHZ;
    else if(u32PllFreq > FREQ_36MHZ)
        u32PllFreq=FREQ_36MHZ;

    if(u32PllClkSrc == CLK_PLLCTL_PLL_SRC_HXT)
    {
        /* PLL source clock from HXT */
        CLK->PLLCTL = (CLK->PLLCTL & ~CLK_PLLCTL_PLL_SRC_HIRC);
        u32PllCr = __HXT;
    }
    else if(u32PllClkSrc == CLK_PLLCTL_PLL_SRC_HIRC)
    {
        /* PLL source clock from HIRC */
        CLK->PLLCTL = (CLK->PLLCTL & ~CLK_PLLCTL_PLL_SRC_HIRC) | (CLK_PLLCTL_PLL_SRC_HIRC);

        /* HIRC Source Selection */
        if(CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)
        {
            /* Clock source from HIRC1 (36MHz) */
            u32PllCr =__HIRC36M;
        }
        else
        {
            /* Clock source from HIRC0 (12MHz) */
            if(CLK->PWRCTL & CLK_PWRCTL_HIRC0FSEL_Msk)
                u32PllCr =__HIRC16M;
            else
                u32PllCr =__HIRC12M;
        }
    }
    else
    {
        /* PLL source clock from MIRC (4MHz) */
        CLK->PLLCTL = (CLK->PLLCTL & ~CLK_PLLCTL_PLL_SRC_MIRC) | (CLK_PLLCTL_PLL_SRC_MIRC);
        u32PllCr =__MIRC;
    }

    u32PLL_N=u32PllCr/1000000;
    u32PLL_M=u32PllFreq/1000000;
    while(1)
    {
        if(u32PLL_M<=48 && u32PLL_N<=36 ) break;
        u32PLL_M >>=1;
        u32PLL_N >>=1;
    }
    u32PLLReg = (u32PLL_M<<CLK_PLLCTL_PLLMLP_Pos) | ((u32PLL_N-1)<<CLK_PLLCTL_INDIV_Pos);
    CLK->PLLCTL = ( CLK->PLLCTL & ~(CLK_PLLCTL_PLLMLP_Msk | CLK_PLLCTL_INDIV_Msk ) )| u32PLLReg;

    if(u32PllClkSrc==CLK_PLLCTL_PLL_SRC_HIRC)
        CLK->PLLCTL = (CLK->PLLCTL & ~CLK_PLLCTL_PLLSRC_Msk) | (CLK_PLLCTL_PLL_SRC_HIRC);
    else if(u32PllClkSrc==CLK_PLLCTL_PLL_SRC_HXT)
        CLK->PLLCTL = (CLK->PLLCTL & ~CLK_PLLCTL_PLLSRC_Msk) | (CLK_PLLCTL_PLL_SRC_HXT);
    else
        CLK->PLLCTL = (CLK->PLLCTL & ~CLK_PLLCTL_PLLSRC_Msk) | (CLK_PLLCTL_PLL_SRC_MIRC);

    CLK->PLLCTL &= ~CLK_PLLCTL_PD_Msk;
    return CLK_GetPLLClockFreq();
}

/**
  * @brief  This function disable PLL
  * @param  None
  * @return None
  */
void CLK_DisablePLL(void)
{
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
}

/**
  * @brief  This function execute delay function.
  * @param[in]  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
void CLK_SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  =  (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV8
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK
  * @param[in]  u32Count is System Tick reload value. It should be 0x1~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt.
  *                    The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    SysTick->CTRL=0;
    if( u32ClkSrc== CLK_CLKSEL0_STCLKSEL_HCLK )    /* Set System Tick clock source */
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    else
    {
        SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
    }
    SysTick->LOAD  = u32Count;                /* Set System Tick reload value */
    SysTick->VAL = 0;                         /* Clear System Tick current value and counter flag  */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; /* Set System Tick counter enabled */
}

/**
  * @brief      Disable System Tick counter
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    SysTick->CTRL = 0;    /* Set System Tick counter disabled */
}

/**
  * @brief  This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including
  *           - \ref CLK_STATUS_CLKSFAIL_Msk
  *           - \ref CLK_STATUS_MIRCSTB_Msk
  *           - \ref CLK_STATUS_HIRC1STB_Msk
  *           - \ref CLK_STATUS_HIRC0STB_Msk
  *           - \ref CLK_STATUS_LIRCSTB_Msk
  *           - \ref CLK_STATUS_PLLSTB_Msk
  *           - \ref CLK_STATUS_LXTSTB_Msk
  *           - \ref CLK_STATUS_HXTSTB_Msk
  * @return   0  clock is not stable
  *           1  clock is stable
  *
  * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt=2160000;

    while((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if(i32TimeOutCnt-- <= 0)
            return 0;
    }
    return 1;
}


/*@}*/ /* end of group NANO103_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_CLK_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/

