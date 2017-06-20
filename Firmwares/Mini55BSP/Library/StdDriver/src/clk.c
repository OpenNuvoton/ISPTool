/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 15/07/09 8:41a $
 * @brief    MINI55 series CLK driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "Mini55Series.h"
/** @addtogroup MINI55_Device_Driver MINI55 Device Driver
  @{
*/

/** @addtogroup MINI55_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup MINI55_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
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
  *                  - \ref CLK_CLKSEL2_FDIVSEL_XTAL
  *                  - \ref CLK_CLKSEL2_FDIVSEL_HCLK
  *                  - \ref CLK_CLKSEL2_FDIVSEL_HIRC
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
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_FDIVSEL_Msk)) | u32ClkSrc;
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
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;
    __WFI();
}

/**
  * @brief  This function get external high frequency crystal frequency. The frequency unit is Hz.
  * @return None
  */
uint32_t CLK_GetHXTFreq(void)
{
    if(CLK->PWRCTL & CLK_PWRCTL_XTL12M )
        return __XTAL12M;
    else
        return 0;
}

/**
  * @brief  This function get external low frequency crystal frequency. The frequency unit is Hz.
  * @return LXT frequency
  */
uint32_t CLK_GetLXTFreq(void)
{
    if(CLK->PWRCTL & CLK_PWRCTL_LXT )
        return __XTAL32K;
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
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | u32ClkDiv;
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
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HIRC         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_XTAL         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LIRC         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HCLK         | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HIRC         | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FDIVSEL_XTAL         | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FDIVSEL_HCLK         | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FDIVSEL_HIRC         | x                      |
  * |\ref I2C_MODULE     | x                                    | x                      |
  * |\ref SPI_MODULE     |\ref CLK_CLKSEL1_SPISEL_HXTorLXT      | x                      |
  * |\ref SPI_MODULE     |\ref CLK_CLKSEL1_SPISEL_HCLK          | x                      |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART0SEL_XTAL        |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART0SEL_HIRC        |\ref CLK_CLKDIV_UART(x) |
    * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UART1SEL_XTAL        |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UART1SEL_HIRC        |\ref CLK_CLKDIV_UART(x) |
  * |\ref PWM01_MODULE   |\ref CLK_CLKSEL1_PWMCH01SEL_HCLK      | x                      |
  * |\ref PWM23_MODULE   |\ref CLK_CLKSEL1_PWMCH23SEL_HCLK      | x                      |
  * |\ref PWM45_MODULE   |\ref CLK_CLKSEL2_PWMCH45SEL_HCLK      | x                      |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_XTAL          |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_HCLK          |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADCSEL_HIRC          |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ACMP_MODULE    | x                                    | x                      |
  */

void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32tmp=0,u32sel=0,u32div=0;

    if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk) {
        u32sel = (uint32_t)&CLK->CLKSEL0+((MODULE_CLKSEL(u32ModuleIdx))*4);
        u32tmp = *(volatile uint32_t *)(u32sel);
        u32tmp = ( u32tmp & ~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)) ) | u32ClkSrc;
        *(volatile uint32_t *)(u32sel) = u32tmp;
    }

    if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk) {
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
  * @brief  This function enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *                  - \ref CLK_PWRCTL_HXT or \ref CLK_PWRCTL_LXT,
  *                  - \ref CLK_PWRCTL_LIRCEN_Msk
  *                  - \ref CLK_PWRCTL_HIRCEN_Msk
  * @return None
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    if(u32ClkMask & CLK_PWRCTL_XTLEN_Msk)
        CLK->PWRCTL &=~CLK_PWRCTL_XTLEN_Msk;
    CLK->PWRCTL |=u32ClkMask;
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
  *                  - \ref FDIV_MODULE
  *                  - \ref I2C_MODULE
  *                  - \ref SPI_MODULE
  *                  - \ref UART0_MODULE
  *                  - \ref UART1_MODULE
  *                  - \ref PWM01_MODULE
  *                  - \ref PWM23_MODULE
  *                  - \ref PWM45_MODULE
  *                  - \ref ADC_MODULE
  *                  - \ref ACMP_MODULE
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->APBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief  This function disable module clock
  * @param[in]  u32ModuleIdx is module index
  *                  - \ref WDT_MODULE
  *                  - \ref TMR0_MODULE
  *                  - \ref TMR1_MODULE
  *                  - \ref FDIV_MODULE
  *                  - \ref I2C_MODULE
  *                  - \ref SPI_MODULE
  *                  - \ref UART0_MODULE
  *                  - \ref UART1_MODULE
  *                  - \ref PWM01_MODULE
  *                  - \ref PWM23_MODULE
  *                  - \ref PWM45_MODULE
  *                  - \ref ADC_MODULE
  *                  - \ref ACMP_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->APBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
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
    int32_t i32TimeOutCnt;

    i32TimeOutCnt = __HSI / 200; /* About 5ms */

    while((CLK->STATUS & u32ClkMask) != u32ClkMask) {
        if(i32TimeOutCnt-- <= 0)
            return 0;
    }
    return 1;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_XTAL
  *             - \ref CLK_CLKSEL0_STCLKSEL_XTAL_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;

    /* Set System Tick clock source */
    if( u32ClkSrc == CLK_CLKSEL0_STCLKSEL_HCLK )
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    else
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief      Disable System Tick counter
  * @param      None
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;
}

/*@}*/ /* end of group MINI55_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI55_CLK_Driver */

/*@}*/ /* end of group MINI55_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
