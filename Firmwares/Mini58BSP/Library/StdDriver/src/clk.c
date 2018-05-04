/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 15 $
 * $Date: 15/06/05 9:39a $
 * @brief    Mini58 series CLK driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "Mini58Series.h"
/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup Mini58_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief  This function disable frequency output function.
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
  * @param[in]  u32ClkSrc is frequency divider function clock source
  *                  - \ref CLK_CLKSEL2_CLKOSEL_XTAL
  *                  - \ref CLK_CLKSEL2_CLKOSEL_LIRC
  *                  - \ref CLK_CLKSEL2_CLKOSEL_HCLK
  *                  - \ref CLK_CLKSEL2_CLKOSEL_HIRC
  * @param[in]  u32ClkDiv Set the clock divider to CKO. 0 <= u32ClkDiv <= 15
  * @param[in]  u32ClkDivBy1En is frequency divided by one enable.
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
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv | u32ClkDivBy1En<<CLK_CLKOCTL_DIV1EN_Pos;

    /* Enable CKO clock source */
    CLK->APBCLK |= CLK_APBCLK_CLKOCKEN_Msk;

    /* Select CKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | u32ClkSrc;
}

/**
  * @brief  This function let system enter to Power-down mode.
  * @return None
  */
void CLK_PowerDown(void)
{
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIF_Msk);
    __WFI();
}

/**
  * @brief  This function let system enter to Idle mode
  * @return None
  */
void CLK_Idle(void)
{
    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk);
    __WFI();
}

/**
  * @brief  This function get external high frequency crystal frequency. The frequency unit is Hz.
  * @return None
  */
uint32_t CLK_GetHXTFreq(void)
{
    if((CLK->PWRCTL & CLK_PWRCTL_XTLEN_Msk)==CLK_PWRCTL_XTLEN_HXT )
        return __HXT;
    else
        return 0;
}

/**
  * @brief  This function get external low frequency crystal frequency. The frequency unit is Hz.
  * @return LXT frequency
  */
uint32_t CLK_GetLXTFreq(void)
{
    if((CLK->PWRCTL & CLK_PWRCTL_XTLEN_Msk )==CLK_PWRCTL_XTLEN_LXT )
        return __XTAL;
    else
        return 0;
}

/**
  * @brief  This function get HCLK frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief  This function get CPU frequency. The frequency unit is Hz.
  * @return CPU frequency
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency. The range of u32Hclk is 26 MHz ~ 50 MHz.
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency. The frequency unit is Hz.
  *             It would configure PLL frequency to 100MHz ~ 200MHz,
  *             set HCLK clock divider as 2 and switch HCLK clock source to PLL.
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* The range of u32Hclk is 25 MHz ~ 50 MHz */
    if(u32Hclk > FREQ_50MHZ)
        u32Hclk = FREQ_50MHZ;
    if(u32Hclk < FREQ_25MHZ)
        u32Hclk = FREQ_25MHZ;

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKDIV &= (~CLK_CLKDIV_HCLKDIV_Msk);

    /* Configure PLL setting if HXT clock is enabled */
    if( (CLK->PWRCTL & CLK_PWRCTL_XTLEN_Msk)==CLK_PWRCTL_XTLEN_HXT )
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, (u32Hclk << 1));

    /* Configure PLL setting if HXT clock is not enabled */
    else
    {
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC, (u32Hclk << 1));

        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;
    }

    /* Select HCLK clock source to PLL,
       Select HCLK clock source divider as 2
       and update system core clock
    */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV_HCLK(2));

    /* Disable HIRC if HIRC is disabled before setting core clock */
    if(u32HIRCSTB == 0)
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;

    /* Return actually HCLK frequency is PLL frequency divide 2 */
    return u32Hclk >> 1;
}

/**
  * @brief  This function set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *                  - \ref CLK_CLKSEL0_HCLKSEL_XTAL
  *                  - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *                  - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *                  - \ref CLK_CLKDIV_HCLK(x)
  * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    /* Apply new Divider */
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

/**
  * @brief  This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return None
  * @details Valid parameter combinations listed in following table:
  *
  * |Module index        |Clock source                          |Divider                 |
  * | :----------------  | :----------------------------------- | :--------------------- |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_XTAL          | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LIRC          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_XTAL         | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LIRC         | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HCLK         | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_TM0          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HIRC         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_XTAL         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LIRC         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HCLK         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_TM1          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HIRC         | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_XTAL         | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HCLK         | x                      |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HIRC         | x                      |
  * |\ref I2C0_MODULE    | x                                    | x                      |
  * |\ref I2C1_MODULE    | x                                    | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL1_SPISEL_XTAL          | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL1_SPISEL_HCLK          | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL1_SPISEL_PLL           | x                      |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_XTAL         |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_PLL          |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UARTSEL_HIRC         |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UARTSEL_XTAL         |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UARTSEL_PLL          |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UARTSEL_HIRC         |\ref CLK_CLKDIV_UART(x) |
  * |\ref PWMCH01_MODULE |\ref CLK_CLKSEL1_PWMCH01SEL_HCLK      | x                      |
  * |\ref PWMCH23_MODULE |\ref CLK_CLKSEL1_PWMCH23SEL_HCLK      | x                      |
  * |\ref PWMCH45_MODULE |\ref CLK_CLKSEL2_PWMCH45SEL_HCLK      | x                      |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_XTAL          |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_PLL           |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_HCLK          |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_HIRC          |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ACMP_MODULE    | x                                    | x                      |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048 | x                      |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL2_WWDTSEL_LIRC         | x                      |
  */

void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32tmp=0,u32sel=0,u32div=0;

    if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk)
    {
        u32sel = (uint32_t)&CLK->CLKSEL0+((MODULE_CLKSEL(u32ModuleIdx))*4);
        u32tmp = *(volatile uint32_t *)(u32sel);
        u32tmp = ( u32tmp & ~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)) ) | u32ClkSrc;
        *(volatile uint32_t *)(u32sel) = u32tmp;
    }

    if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk)
    {
        u32div =(uint32_t)&CLK->CLKDIV+((MODULE_CLKDIV(u32ModuleIdx))*4);
        u32tmp = *(volatile uint32_t *)(u32div);
        u32tmp = ( u32tmp & ~(MODULE_CLKDIV_Msk(u32ModuleIdx)<<MODULE_CLKDIV_Pos(u32ModuleIdx)) ) | u32ClkDiv;
        *(volatile uint32_t *)(u32div) = u32tmp;
    }
}

/**
  * @brief  This function set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including
  *                  - \ref CLK_CLKSEL0_STCLKSEL_XTAL
  *                  - \ref CLK_CLKSEL0_STCLKSEL_XTAL_DIV2
  *                  - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *                  - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  * @return None
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_XTAL
  *             - \ref CLK_CLKSEL0_STCLKSEL_XTAL_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
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
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;
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
  * @brief  This function enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *                  - \ref CLK_PWRCTL_XTLEN_HXT or \ref CLK_PWRCTL_XTLEN_LXT,
  *                  - \ref CLK_PWRCTL_LIRCEN_Msk
  *                  - \ref CLK_PWRCTL_HIRCEN_Msk
  * @return None
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    if(u32ClkMask & CLK_PWRCTL_XTLEN_Msk)
        CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_XTLEN_Msk) | u32ClkMask;
    else
        CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief  This function disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *                  - \ref CLK_PWRCTL_XTLEN_Msk,
  *                  - \ref CLK_PWRCTL_LIRCEN_Msk,
  *                  - \ref CLK_PWRCTL_HIRCEN_Msk,
  * @return None
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &=~u32ClkMask;
}

/**
  * @brief  This function enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *                  - \ref WDT_MODULE
  *                  - \ref TMR0_MODULE
  *                  - \ref TMR1_MODULE
  *                  - \ref CLKO_MODULE
  *                  - \ref I2C0_MODULE
  *                  - \ref I2C1_MODULE
  *                  - \ref SPI0_MODULE
  *                  - \ref UART0_MODULE
  *                  - \ref UART1_MODULE
  *                  - \ref PWMCH01_MODULE
  *                  - \ref PWMCH23_MODULE
  *                  - \ref PWMCH45_MODULE
  *                  - \ref ADC_MODULE
  *                  - \ref ACMP_MODULE
  *                  - \ref WWDT_MODULE
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief  This function disable module clock
  * @param[in]  u32ModuleIdx is module index
  *                  - \ref WDT_MODULE
  *                  - \ref TMR0_MODULE
  *                  - \ref TMR1_MODULE
  *                  - \ref CLKO_MODULE
  *                  - \ref I2C0_MODULE
  *                  - \ref I2C1_MODULE
  *                  - \ref SPI0_MODULE
  *                  - \ref UART0_MODULE
  *                  - \ref UART1_MODULE
  *                  - \ref PWMCH01_MODULE
  *                  - \ref PWMCH23_MODULE
  *                  - \ref PWMCH45_MODULE
  *                  - \ref ADC_MODULE
  *                  - \ref ACMP_MODULE
  *                  - \ref WWDT_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_PLLCTL_PLLSRC_HXT
  *             - \ref CLK_PLLCTL_PLLSRC_HIRC
  * @param[in]  u32PllFreq is PLL frequency.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency.
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32CLK_SRC, u32NRT;
    uint32_t u32Tmp, u32Tmp2, u32Tmp3, u32Min, u32MinNF, u32MinNR,u32Best;

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* PLL source clock is from HXT */
    if(u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT)
    {
        /* Enable HXT clock */
        CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_XTLEN_Msk) | CLK_PWRCTL_XTLEN_HXT ;

        /* Wait for HXT clock ready */
        CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk);

        /* Select PLL source clock from HXT */
        u32CLK_SRC = CLK_PLLCTL_PLLSRC_HXT;
        u32PllSrcClk = __XTAL;

        /* u32NR start from 2 */
        u32NRT = 2;
    }

    /* PLL source clock is from HIRC */
    else
    {
        /* Enable HIRC clock */
        CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

        /* Wait for HIRC clock ready */
        CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

        /* Select PLL source clock from HIRC */
        u32CLK_SRC = CLK_PLLCTL_PLLSRC_HIRC;
        u32PllSrcClk = __HIRC;

        /* u32NR start from 4 when FIN = 22.1184MHz to avoid calculation overflow */
        u32NRT = 4;
    }

    /* Select "NO" according to request frequency */
    if((u32PllFreq <= FREQ_200MHZ) && (u32PllFreq > FREQ_100MHZ))
    {
        u32NO = 0;
        u32PllFreq = u32PllFreq;
    }
    else if((u32PllFreq <= FREQ_100MHZ) && (u32PllFreq >= FREQ_50MHZ))
    {
        u32NO = 1;
        u32PllFreq = u32PllFreq << 1;
    }
    else if((u32PllFreq < FREQ_50MHZ) && (u32PllFreq >= FREQ_25MHZ))
    {
        u32NO = 3;
        u32PllFreq = u32PllFreq << 2;
    }
    else
    {
        /* Wrong frequency request. Just return default setting. */
        goto lexit;
    }

    /* Find best solution */
    u32Min = (uint32_t) - 1;
    u32MinNR = 0;
    u32MinNF = 0;
    for(u32NR=u32NRT; u32NR <= 33; u32NR++)
    {
        u32Tmp = u32PllSrcClk / u32NR;
        if((u32Tmp > 1600000) && (u32Tmp < 15000000))
        {
            for(u32NF = 2; u32NF <= 513; u32NF++)
            {
                u32Tmp2 = u32Tmp * u32NF;
                if((u32Tmp2 >= 100000000) && (u32Tmp2 <= 200000000))
                {
                    u32Tmp3 = (u32Tmp2 > u32PllFreq) ? u32Tmp2 - u32PllFreq : u32PllFreq - u32Tmp2;
                    if(u32Tmp3 < u32Min)
                    {
                        u32Min = u32Tmp3;
                        u32MinNR = u32NR;
                        u32MinNF = u32NF;
                        /* Break when get good results */
                        if(u32Min == 0)
                        {
                            /* Enable and apply new PLL setting. */
                            CLK->PLLCTL = u32CLK_SRC | (u32NO << 14) | ((u32MinNR - 2) << 9) | (u32MinNF - 2);

                            /* Wait for PLL clock stable */
                            CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

                            /* Return actual PLL output clock frequency */
                            return u32PllSrcClk / ((u32NO + 1) * u32MinNR) * u32MinNF;
                        }
                    }
                }
            }
        }
    }


    /* Find best solution */
    u32Best = u32Min;
    u32Min = (uint32_t) - 1;
    u32MinNR = 0;
    u32MinNF = 0;
    for(u32NR=u32NRT; u32NR <= 33; u32NR++)
    {
        u32Tmp = u32PllSrcClk / u32NR;
        if((u32Tmp > 1600000) && (u32Tmp < 15000000))
        {
            for(u32NF = 2; u32NF <= 513; u32NF++)
            {
                u32Tmp2 = u32Tmp * u32NF;
                if((u32Tmp2 >= 100000000) && (u32Tmp2 <= 200000000))
                {
                    u32Tmp3 = (u32Tmp2 > u32PllFreq) ? u32Tmp2 - u32PllFreq : u32PllFreq - u32Tmp2;
                    if(u32Tmp3 < u32Min)
                    {
                        u32Min = u32Tmp3;
                        u32MinNR = u32NR;
                        u32MinNF = u32NF;

                        /* Break when get good results */
                        if(u32Min == u32Best)
                        {
                            /* Enable and apply new PLL setting. */
                            CLK->PLLCTL = u32CLK_SRC | (u32NO << 14) | ((u32MinNR - 2) << 9) | (u32MinNF - 2);

                            /* Wait for PLL clock stable */
                            CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

                            /* Return actual PLL output clock frequency */
                            return u32PllSrcClk / ((u32NO + 1) * u32MinNR) * u32MinNF;
                        }
                    }
                }
            }
        }
    }


lexit:

    /* Apply default PLL setting and return */
    if(u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT)
        CLK->PLLCTL = CLK_PLLCTL_72MHz_HXT; /* 72MHz */
    else
        CLK->PLLCTL = CLK_PLLCTL_72MHz_HIRC; /* 71.8848MHz */

    /* Wait for PLL clock stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    return CLK_GetPLLClockFreq();

}

/**
  * @brief      Disable PLL
  * @param      None
  * @return     None
  * @details    This function set PLL in Power-down mode.
  *             The register write-protection function should be disabled before using this function.
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
  * @brief  This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including
  *                  - \ref CLK_STATUS_CLKSFAIL_Msk
  *                  - \ref CLK_STATUS_HIRCSTB_Msk
  *                  - \ref CLK_STATUS_LIRCSTB_Msk
  *                  - \ref CLK_STATUS_XTLSTB_Msk
  *
  * @return   0  clock is not stable
  *           1  clock is stable
  *
  * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt = 2160000;

    while((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if(i32TimeOutCnt-- <= 0)
            return 0;
    }
    return 1;
}


/*@}*/ /* end of group Mini58_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_CLK_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
