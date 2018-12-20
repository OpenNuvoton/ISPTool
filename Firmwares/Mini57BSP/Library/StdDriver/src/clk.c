/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series CLK driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "Mini57Series.h"
/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup Mini57_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      This function disable frequency output function.
  * @param      None
  * @return     None
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK->CLKOCTL &= (~CLK_CLKOCTL_CLKOEN_Msk);
}

/**
  * @brief      This function enable frequency divider module clock,
  *             enable frequency divider clock function and configure frequency divider.
  * @param      u32ClkSrc is frequency divider function clock source
  *             - \ref CLK_CLKO_SRC_EXT
  *             - \ref CLK_CLKO_SRC_HCLK
  *             - \ref CLK_CLKO_SRC_HIRC
  * @param      u32ClkDiv is divider output frequency selection.
  * @param      u32ClkDivBy1En is frequency divided by one enable.
  * @return     None
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
    CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, 0UL);

    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk))
                   | ((u32ClkDiv) << CLK_CLKOCTL_FREQSEL_Pos)
                   | ((u32ClkDivBy1En) << CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
    CLK->CLKOCTL |= CLK_CLKOCTL_CLKOEN_Msk;
}


/**
  * @brief      This function let system enter to Power-down mode.
  * @param      None
  * @return     None
  */
void CLK_PowerDown(void)
{
    /* Enable PD.0 (nRESET pin) interrupt that trigger by falling edge to make sure
       RESET button can wake up system from power down mode. */
    PD->INTTYPE &= (~BIT0);     /* edge trigger for PD0 */
    PD->INTEN   |=   BIT0;      /* enable falling or low trigger for PD0 */

    /* enable M0 register SCR[SEVONPEND] and SCR[SLEEPDEEP] */
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk);
    /* clear interrupt status and enable wake up interrupt */
    CLK->PWRCTL |= (CLK_PWRCTL_PDWKIF_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    /* enable system power-down feature */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);
    /* execute Wait For Interrupt; Enter power-down mode since CLK_PWRCTL_PDEN_Msk is 1 */
    __WFI();
}

/**
  * @brief      This function let system enter to Idle mode
  * @param      None
  * @return     None
  */
void CLK_Idle(void)
{
    /* disable system power-down feature */
    CLK->PWRCTL &= ~(CLK_PWRCTL_PDEN_Msk);
    /* execute Wait For Interrupt; Enter idle mode since CLK_PWRCTL_PDEN_Msk is 0 */
    __WFI();
}

/**
  * @brief      This function get external high frequency crystal frequency (HXT). The frequency unit is Hz.
  * @param      None
  * @return     HXT frequency
  */
uint32_t CLK_GetHXTFreq(void)
{
    if ( ((CLK->PWRCTL & CLK_PWRCTL_XTLEN_Msk) >> CLK_PWRCTL_XTLEN_Pos) == 0x01 )
        return __HXT;
    else
        return 0;
}

/**
  * @brief      This function get external low frequency crystal frequency (LXT). The frequency unit is Hz.
  * @param      None
  * @return     LXT frequency
  */
uint32_t CLK_GetLXTFreq(void)
{
    if ( ((CLK->PWRCTL & CLK_PWRCTL_XTLEN_Msk) >> CLK_PWRCTL_XTLEN_Pos) == 0x02 )
        return __LXT;
    else
        return 0;
}

/**
  * @brief      This function get HCLK frequency. The frequency unit is Hz.
  * @param      None
  * @return     HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      This function get PCLK frequency. The frequency unit is Hz.
  * @param      None
  * @return     PCLK frequency
  * @details    This function get PCLK frequency. The frequency unit is Hz.
  *             The PCLK is always same to HCLK in Mini57.
  */
uint32_t CLK_GetPCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      This function get CPU frequency. The frequency unit is Hz.
  * @param      None
  * @return     CPU frequency
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      This function set HCLK frequency. The frequency unit is Hz.
  * @param[in]  u32Hclk is HCLK frequency
  * @return     HCLK frequency.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    uint32_t u32Div;

    if ((u32Hclk <= __HIRC) && (u32Hclk > 0))
    {
        u32Div = __HIRC / u32Hclk;
        if (__HIRC % u32Hclk != 0)
            u32Div++;
        if (u32Div > 16)
        {
            printf("ERROR: CLK_SetCoreClock(): HCLK divider (%d) cannot > 16 !!\n", u32Div);
            return SystemCoreClock; /* Don't change HCLK. Return current HCLK. */
        }
    }
    else
    {
        printf("ERROR: CLK_SetCoreClock(): Invalid HCLK frequency %d !!\n", u32Hclk);
        return SystemCoreClock; /* Don't change HCLK. Return current HCLK. */
    }
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_SetHCLK(CLK_HCLK_SRC_HIRC, CLK_CLKDIV_HCLK(u32Div));
    return SystemCoreClock;
}

/**
  * @brief      This function set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_HCLK_SRC_EXT
  *             - \ref CLK_HCLK_SRC_LIRC
  *             - \ref CLK_HCLK_SRC_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV_HCLK(x) where x=1..16
  * @return     None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    CLK->CLKDIV  = (CLK->CLKDIV  & ~CLK_CLKDIV_HCLKDIV_Msk)  | u32ClkDiv;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    SystemCoreClockUpdate();
}

/**
  * @brief      This function set PCLK clock source and PCLK clock divider
  * @param[in]  u32ClkSrc is PCLK clock source. Including :
  *             - \ref CLK_HCLK_SRC_EXT
  *             - \ref CLK_HCLK_SRC_LIRC
  *             - \ref CLK_HCLK_SRC_HIRC
  * @param[in]  u32ClkDiv is PCLK clock divider. Including :
  *             - \ref CLK_CLKDIV_HCLK(x) where x=1..16
  * @return     None
  * @details    This function set PCLK clock source and PCLK clock divider.
  *             The PCLK is always same to HCLK in Mini57.
  */
void CLK_SetPCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    CLK->CLKDIV  = (CLK->CLKDIV  & ~CLK_CLKDIV_HCLKDIV_Msk)  | u32ClkDiv;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    SystemCoreClockUpdate();
}

/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *-
  * |Module index           |Clock source                       |Divider                    |
  * | :-------------------  | :-------------------------------  | :-----------------------  |
  * |\ref HDIV_MODULE       | x                                 | x                         |
  * |\ref ISP_MODULE        | x                                 | x                         |
  * |\ref ACMP_MODULE       | x                                 | x                         |
  * |\ref EADC_MODULE       |\ref CLK_EADC_SRC_EXT              |\ref CLK_CLKDIV_EADC(x)    |
  * |\ref EADC_MODULE       |\ref CLK_EADC_SRC_HCLK             |\ref CLK_CLKDIV_EADC(x)    |
  * |\ref EADC_MODULE       |\ref CLK_EADC_SRC_HIRC             |\ref CLK_CLKDIV_EADC(x)    |
  * |\ref USCI0_MODULE      | x                                 | x                         |
  * |\ref USCI1_MODULE      | x                                 | x                         |
  * |\ref BPWM_MODULE       | x                                 | x                         |
  * |\ref EPWM_MODULE       | x                                 | x                         |
  * |\ref PGA_MODULE        | x                                 | x                         |
  * |\ref ECAP_MODULE       | x                                 | x                         |
  * |\ref CLKO_MODULE       |\ref CLK_CLKO_SRC_EXT              | x                         |
  * |\ref CLKO_MODULE       |\ref CLK_CLKO_SRC_HCLK             | x                         |
  * |\ref CLKO_MODULE       |\ref CLK_CLKO_SRC_HIRC             | x                         |
  * |\ref TMR0_MODULE       |\ref CLK_TMR0_SRC_EXT              | x                         |
  * |\ref TMR0_MODULE       |\ref CLK_TMR0_SRC_LIRC             | x                         |
  * |\ref TMR0_MODULE       |\ref CLK_TMR0_SRC_HCLK             | x                         |
  * |\ref TMR0_MODULE       |\ref CLK_TMR0_SRC_T0               | x                         |
  * |\ref TMR0_MODULE       |\ref CLK_TMR0_SRC_HIRC             | x                         |
  * |\ref TMR1_MODULE       |\ref CLK_TMR1_SRC_EXT              | x                         |
  * |\ref TMR1_MODULE       |\ref CLK_TMR1_SRC_LIRC             | x                         |
  * |\ref TMR1_MODULE       |\ref CLK_TMR1_SRC_HCLK             | x                         |
  * |\ref TMR1_MODULE       |\ref CLK_TMR1_SRC_T1               | x                         |
  * |\ref TMR1_MODULE       |\ref CLK_TMR1_SRC_HIRC             | x                         |
  * |\ref WDT_MODULE        |\ref CLK_WDT_SRC_EXT               | x                         |
  * |\ref WDT_MODULE        |\ref CLK_WDT_SRC_HCLK_2048         | x                         |
  * |\ref WDT_MODULE        |\ref CLK_WDT_SRC_LIRC              | x                         |
  */

void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32tmp=0,u32sel=0,u32div=0;

    if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk)
    {
        u32div =(uint32_t)&CLK->CLKDIV+((MODULE_CLKDIV(u32ModuleIdx))*4);   /* Get register address */
        u32tmp = *(volatile uint32_t *)(u32div);                            /* Get register content value */
        u32tmp = ( u32tmp & ~(MODULE_CLKDIV_Msk(u32ModuleIdx)<<MODULE_CLKDIV_Pos(u32ModuleIdx)) ) | u32ClkDiv;
        *(volatile uint32_t *)(u32div) = u32tmp;
    }

    if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk)
    {
        u32sel = (uint32_t)&CLK->CLKSEL0+((MODULE_CLKSEL(u32ModuleIdx))*4); /* Get register address */
        u32tmp = *(volatile uint32_t *)(u32sel);                            /* Get register content value */
        u32tmp = ( u32tmp & ~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)) ) | u32ClkSrc;
        *(volatile uint32_t *)(u32sel) = u32tmp;
    }
}

/**
  * @brief      This function enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including:
  *             - \ref CLK_PWRCTL_HXT_EN
  *             - \ref CLK_PWRCTL_LXT_EN
  *             - \ref CLK_PWRCTL_HIRC_EN
  *             - \ref CLK_PWRCTL_LIRC_EN
  * @return     None
  * @details    This function enable clock source.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    if((u32ClkMask == CLK_PWRCTL_HXT_EN) || (u32ClkMask == CLK_PWRCTL_LXT_EN))
        CLK->PWRCTL &= ~CLK_PWRCTL_XTLEN_Msk;
    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief      This function disable clock source
  * @param      u32ClkMask is clock source mask. Including:
  *             - \ref CLK_PWRCTL_HXT_EN
  *             - \ref CLK_PWRCTL_LXT_EN
  *             - \ref CLK_PWRCTL_HIRC_EN
  *             - \ref CLK_PWRCTL_LIRC_EN
  * @return None
  * @details    This function disable clock source.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~u32ClkMask;
}

/**
  * @brief      This function enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref HDIV_MODULE
  *             - \ref ISP_MODULE
  *             - \ref ACMP_MODULE
  *             - \ref EADC_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref USCI1_MODULE
  *             - \ref BPWM_MODULE
  *             - \ref EPWM_MODULE
  *             - \ref PGA_MODULE
  *             - \ref ECAP_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref WDT_MODULE
  * @return     None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief      This function disable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref HDIV_MODULE
  *             - \ref ISP_MODULE
  *             - \ref ACMP_MODULE
  *             - \ref EADC_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref USCI1_MODULE
  *             - \ref BPWM_MODULE
  *             - \ref EPWM_MODULE
  *             - \ref PGA_MODULE
  *             - \ref ECAP_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref WDT_MODULE
  * @return     None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief      This function execute delay function.
  * @param[in]  us      Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                     50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
void CLK_SysTickDelay(uint32_t us)
{
    uint32_t delay_tick;

    delay_tick = us * CyclesPerUs;
    if (delay_tick > SysTick_LOAD_RELOAD_Msk)   /* SysTick_LOAD_RELOAD_Msk is 24 bits for Mini57 */
    {
        printf("ERROR: CLK_SysTickDelay(): the delay tick (%d) cannot > %d !\n", us, SysTick_LOAD_RELOAD_Msk/CyclesPerUs);
        return;
    }
    SysTick->LOAD = delay_tick;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_SYSTICK_SRC_HCLK
  *             - \ref CLK_SYSTICK_SRC_EXT
  *             - \ref CLK_SYSTICK_SRC_EXT_HALF
  *             - \ref CLK_SYSTICK_SRC_HCLK_HALF
  *             - \ref CLK_SYSTICK_SRC_HIRC_HALF
  * @param[in]  u32Count is System Tick reload value. It should be 0x1~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    SysTick->CTRL=0;
    if( u32ClkSrc== CLK_SYSTICK_SRC_HCLK )    /* Set System Tick clock source */
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
    SysTick->CTRL = 0;      /* Set System Tick counter disabled */
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including
  *             - \ref CLK_STATUS_CLKSFAIL_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_XTLSTB_Msk
  * @return     0  clock is not stable
  *             1  clock is stable
  *
  * @details    To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
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


/*@}*/ /* end of group Mini57_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_CLK_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

