/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/11 10:26a $
 * @brief    I94100 Series CLK driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "I94100.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup I94100_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable clock divider output function
  * @param      None
  * @return     None
  * @details    This function disable clock divider output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
	CLK->APBCLK0 &= (~CLK_APBCLK0_CLKOCKEN_Msk);
}

/**
  * @brief      This function enable clock divider output module clock,
  *             enable clock divider output function and set frequency selection.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKSEL1_CLKOSEL_HXT
  *             - \ref CLK_CLKSEL1_CLKOSEL_LXT
  *             - \ref CLK_CLKSEL1_CLKOSEL_HCLK
  *             - \ref CLK_CLKSEL1_CLKOSEL_HIRC
  * @param[in]  u32ClkDiv is divider output frequency selection. It could be 0~15.
  * @param[in]  u32ClkDivBy1En is clock divided by one enabled.
  *				0 = Clock Output will output clock with source frequency divided by u32ClkDiv.
  *				1 = Clock Output will output clock with source frequency.
  * @return     None
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv. \n
  *             The formula is: \n
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1) \n
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves. \n
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
	CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CKO clock source */
	CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_CLKOSEL_Msk)) | u32ClkSrc;
}

/**
  * @brief      Enter to Power-down mode
  * @param      None
  * @return     None
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function. 
  */
void CLK_PowerDown(void)
{
    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled and Power-down entry condition */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWTCPU_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Enter to Idle mode
  * @param      None
  * @return     None
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Get external high speed crystal clock frequency
  * @param      None
  * @return     External high frequency crystal frequency
  * @details    This function get external high frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHXTFreq(void)
{
    if(CLK->PWRCTL & CLK_PWRCTL_HXTEN_Msk)
        return __HXT;
    else
        return 0;
}


/**
  * @brief      Get external low speed crystal clock frequency
  * @param      None
  * @return     External low speed crystal clock frequency
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
    if(CLK->PWRCTL & CLK_PWRCTL_LXTEN_Msk)
        return __LXT;
    else
        return 0;
}

/**
  * @brief      Get PCLK0 frequency
  * @param      None
  * @return     PCLK0 frequency
  * @details    This function get PCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
	uint32_t u32Freq;
    SystemCoreClockUpdate();
	
	if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_PCLK0DIV1) {
        u32Freq = SystemCoreClock;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_PCLK0DIV2) {
        u32Freq = SystemCoreClock / 2UL;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_PCLK0DIV4) {
        u32Freq = SystemCoreClock / 4UL;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_PCLK0DIV8) {
        u32Freq = SystemCoreClock / 8UL;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_PCLK0DIV16) {
        u32Freq = SystemCoreClock / 16UL;
    } else {
        u32Freq = SystemCoreClock;
    }
	return u32Freq;
}


/**
  * @brief      Get PCLK1 frequency
  * @param      None
  * @return     PCLK1 frequency
  * @details    This function get PCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t u32Freq;
	SystemCoreClockUpdate();
	
    if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_PCLK1DIV1) {
        u32Freq = SystemCoreClock;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_PCLK1DIV2) {
        u32Freq = SystemCoreClock / 2UL;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_PCLK1DIV4) {
        u32Freq = SystemCoreClock / 4UL;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_PCLK1DIV8) {
        u32Freq = SystemCoreClock / 8UL;
    } else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_PCLK1DIV16) {
        u32Freq = SystemCoreClock / 16UL;
    } else {
        u32Freq = SystemCoreClock;
    }
	return u32Freq;
}


/**
  * @brief      Get HCLK frequency
  * @param      None
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief      Get CPU frequency
  * @param      None
  * @return     CPU frequency
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency. 
  *             The normal mode range of u32Hclk is running up to 160MHz.
  *             The turbo mode range of u32Hclk is running up to 200MHz(160~200MHz).
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency. The frequency unit is Hz. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
	uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKDIV0 &= (~CLK_CLKDIV0_HCLKDIV_Msk);

    /* Configure PLL setting if HXT clock is enabled */
    if((CLK->PWRCTL & CLK_PWRCTL_HXTEN_Msk) == CLK_PWRCTL_HXTEN_Msk) {
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, u32Hclk);
    }
    /* Configure PLL setting if HXT clock is not enabled */
    else {
        u32Hclk = CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC, u32Hclk);

        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;
    }

    /* Select HCLK clock source to PLL,
       and update system core clock
    */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1UL));

    /* Disable HIRC if HIRC is disabled before setting core clock */
    if(u32HIRCSTB == 0UL) {
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;
    }

    /* Return actually HCLK frequency is PLL frequency divide 1 */
    return u32Hclk;
}

/**
  * @brief      This function set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_CLKSEL0_HCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_HCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_HCLKSEL_PLL
  *             - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV0_HCLK(x)
  * @return     None
  * @details    This function set HCLK clock source and HCLK clock divider. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch to HIRC for Safe. Avoid HCLK too high when applying new divider. */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Apply new Divider */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if(u32HIRCSTB == 0)
        CLK->PWRCTL &= ~CLK_PWRCTL_HIRCEN_Msk;
}

/**
  * @brief      This function set PCLK divider.
  * @param[in]  u32PCLKDiv is PCLK divider. Including :
  *             - \ref CLK_PCLKDIV_PCLK0DIV1
  *             - \ref CLK_PCLKDIV_PCLK0DIV2
  *             - \ref CLK_PCLKDIV_PCLK0DIV4
  *             - \ref CLK_PCLKDIV_PCLK0DIV8
  *             - \ref CLK_PCLKDIV_PCLK0DIV16
  *             - \ref CLK_PCLKDIV_PCLK1DIV1
  *             - \ref CLK_PCLKDIV_PCLK1DIV2
  *             - \ref CLK_PCLKDIV_PCLK1DIV4
  *             - \ref CLK_PCLKDIV_PCLK1DIV8
  *             - \ref CLK_PCLKDIV_PCLK1DIV16
  * @return       0: Set divider failed; 1: Set divider succeed
  * @details    When the clock rate of HCLK greater than 100 MHz, \n 
  * 			the value of APB1DIV (CLK_PCLKDIV[6:4]) and APB0DIV(CLK_PCLKDIV[2:0]) must be greater than 0.. \n
  */
uint32_t CLK_SetPCLKDivider(uint32_t u32PCLKDiv)
{
	uint32_t u32HCLKFreq;
	
	if(u32PCLKDiv == CLK_PCLKDIV_PCLK0DIV1 || u32PCLKDiv == CLK_PCLKDIV_PCLK1DIV1)
	{
		u32HCLKFreq = CLK_GetHCLKFreq();
		if(u32HCLKFreq >= FREQ_100MHZ)
			return 0;
	}
	else
	{
		if(u32PCLKDiv & CLK_PCLKDIV_APB0DIV_Msk)
		{
			CLK->PCLKDIV = (CLK->PCLKDIV & ~CLK_PCLKDIV_APB0DIV_Msk) | u32PCLKDiv;
			return 1;
		}
		else if(u32PCLKDiv & CLK_PCLKDIV_APB1DIV_Msk)
		{
			CLK->PCLKDIV = (CLK->PCLKDIV & ~CLK_PCLKDIV_APB1DIV_Msk) | u32PCLKDiv;
			return 1;
		}
		else
			return 0;
	}
	return 0;
}

/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index        |Clock source                           |Divider                  |
  * | :----------------  | :-----------------------------------  | :---------------------- |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LXT            | x                       |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_PCLK0_DIV2048  | x                       |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LIRC           | x                       |
  * |\ref RTC_MODULE     |\ref CLK_CLKSEL3_RTCSEL_LXT            | x                       |
  * |\ref RTC_MODULE     |\ref CLK_CLKSEL3_RTCSEL_LIRC           | x                       |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HXT           | x                       |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LXT           | x                       |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_PCLK0         | x                       |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_EXT_TRG       | x                       |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LIRC          | x                       |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HIRC          | x                       |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HXT           | x                       |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LXT           | x                       |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_PCLK0         | x                       |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_EXT_TRG       | x                       |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LIRC          | x                       |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HIRC          | x                       |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_HXT           | x                       |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_LXT           | x                       |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_PCLK1         | x                       |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_EXT_TRG       | x                       |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_LIRC          | x                       |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_HIRC          | x                       |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_HXT           | x                       |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_LXT           | x                       |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_PCLK1         | x                       |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_EXT_TRG       | x                       |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_LIRC          | x                       |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3SEL_HIRC          | x                       |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL1_CLKOSEL_HXT           | x                       |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL1_CLKOSEL_LXT           | x                       |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL1_CLKOSEL_HCLK          | x                       |
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL1_CLKOSEL_HIRC          | x                       |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_HXT           | x                       |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_PLL           | x                       |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_PCLK0         | x                       |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL2_SPI0SEL_HIRC          | x                       |
  * |\ref SPI1_MODULE    |\ref CLK_CLKSEL2_SPI1SEL_HXT           | x                       |
  * |\ref SPI1_MODULE    |\ref CLK_CLKSEL2_SPI1SEL_PLL           | x                       |
  * |\ref SPI1_MODULE    |\ref CLK_CLKSEL2_SPI1SEL_PCLK1         | x                       |
  * |\ref SPI1_MODULE    |\ref CLK_CLKSEL2_SPI1SEL_HIRC          | x                       |
  * |\ref SPI2_MODULE    |\ref CLK_CLKSEL2_SPI2SEL_HXT           | x                       |
  * |\ref SPI2_MODULE    |\ref CLK_CLKSEL2_SPI2SEL_PLL           | x                       |
  * |\ref SPI2_MODULE    |\ref CLK_CLKSEL2_SPI2SEL_PCLK0         | x                       |
  * |\ref SPI2_MODULE    |\ref CLK_CLKSEL2_SPI2SEL_HIRC          | x                       |
  * |\ref DMIC_MODULE    |\ref CLK_CLKSEL2_DMICSEL_HXT           | x                       |
  * |\ref DMIC_MODULE    |\ref CLK_CLKSEL2_DMICSEL_PLL           | x                       |
  * |\ref DMIC_MODULE    |\ref CLK_CLKSEL2_DMICSEL_PCLK0         | x                       |
  * |\ref DMIC_MODULE    |\ref CLK_CLKSEL2_DMICSEL_HIRC          | x                       |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART0SEL_HXT          |\ref CLK_CLKDIV0_UART0(x)|
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART0SEL_PLL          |\ref CLK_CLKDIV0_UART0(x)|
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART0SEL_LXT          |\ref CLK_CLKDIV0_UART0(x)|
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART0SEL_HIRC         |\ref CLK_CLKDIV0_UART0(x)|
  * |\ref USBD_MODULE  	 | x          							 |\ref CLK_CLKDIV0_USBD(x) |
  * |\ref EADC_MODULE    | x                                     |\ref CLK_CLKDIV0_EADC(x) |
  * |\ref I2S0_MODULE	 |\ref CLK_CLKSEL3_I2S0SEL_HXT			 | x					   |
  * |\ref I2S0_MODULE	 |\ref CLK_CLKSEL3_I2S0SEL_PLL			 | x					   |
  * |\ref I2S0_MODULE	 |\ref CLK_CLKSEL3_I2S0SEL_PCLK0		 | x					   |
  * |\ref I2S0_MODULE	 |\ref CLK_CLKSEL3_I2S0SEL_HIRC			 | x					   |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL2_PWM0SEL_PLL           | x                       |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL2_PWM0SEL_PCLK0         | x                       |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048  | x                       |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL1_WWDTSEL_LIRC          | x                       |
  * |\ref DPWM_MODULE    |\ref CLK_CLKSEL2_DPWMSEL_HXT           | x                       |
  * |\ref DPWM_MODULE    |\ref CLK_CLKSEL2_DPWMSEL_PCLK0         | x                       |
  * |\ref DPWM_MODULE    |\ref CLK_CLKSEL2_DPWMSEL_HIRC          | x                       |
  * |\ref DPWM_MODULE    |\ref CLK_CLKSEL2_DPWMSEL_PLL           | x                       |
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32sel = 0, u32div = 0;

    if(MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32div = (uint32_t)&CLK->CLKDIV0 + ((MODULE_CLKDIV(u32ModuleIdx)) * 4);
        /* Apply new divider */
        M32(u32div) = (M32(u32div) & (~(MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;
    }

    if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32sel = (uint32_t)&CLK->CLKSEL0 + ((MODULE_CLKSEL(u32ModuleIdx)) * 4);
        /* Set new clock selection setting */
        M32(u32sel) = (M32(u32sel) & (~(MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;
    }
}


/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  * @return     None
  * @details    This function set SysTick clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc;

}

/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HXTEN_Msk
  *             - \ref CLK_PWRCTL_LXTEN_Msk
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return     None
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_HXTEN_Msk
  *             - \ref CLK_PWRCTL_LXTEN_Msk
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return     None
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~u32ClkMask;
}

/**
  * @brief      Enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref PDMA_MODULE
  *             - \ref ISP_MODULE
  *             - \ref CRC_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref RTC_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref DMIC_MODULE
  *             - \ref UART0_MODULE
  *             - \ref USBD_MODULE
  *             - \ref EADC_MODULE
  *				- \ref I2S0_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref DPWM_MODULE
  * @return     None
  * @details    This function is used to enable module clock.
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK + (MODULE_APBCLK(u32ModuleIdx) * 4))  |= 1 << MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief      Disable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref PDMA_MODULE
  *             - \ref ISP_MODULE
  *             - \ref CRC_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref RTC_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref DMIC_MODULE
  *             - \ref UART0_MODULE
  *             - \ref USBD_MODULE
  *             - \ref EADC_MODULE
  *				- \ref I2S0_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref DPWM_MODULE
  * @return     None
  * @details    This function is used to disable module clock.
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK + (MODULE_APBCLK(u32ModuleIdx) * 4))  &= ~(1 << MODULE_IP_EN_Pos(u32ModuleIdx));
}


/**
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_PLLCTL_PLLSRC_HXT
  *             - \ref CLK_PLLCTL_PLLSRC_HIRC
  * @param[in]  u32PllFreq is PLL frequency.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32CLK_SRC, u32PllClk;
    uint32_t u32Tmp, u32Tmp2, u32Tmp3, u32Min, u32MinNF, u32MinNR, u32MinNO, u32basFreq;

	// Set LDO voltage for turbo mode.
    if(u32PllFreq > FREQ_160MHZ ) 
	{
		if( u32PllFreq > FREQ_200MHZ )
		{
			u32PllFreq = FREQ_200MHZ;
		}
        CLK->LDOCTL |= CLK_LDOCTL_OVEN_Msk;
    }
	else
	{
		CLK->LDOCTL &= ~CLK_LDOCTL_OVEN_Msk;
	}
	
    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* PLL source clock is from HXT */
    if(u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT) {
        /* Enable HXT clock */
        CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

        /* Wait for HXT clock ready */
        CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

        /* Select PLL source clock from HXT */
        u32CLK_SRC = CLK_PLLCTL_PLLSRC_HXT;
        u32PllSrcClk = __HXT;

        /* u32NR start from 2 */
        u32NR = 2UL;
    }

    /* PLL source clock is from HIRC */
    else {
        /* Enable HIRC clock */
        CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

        /* Wait for HIRC clock ready */
        CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

        /* Select PLL source clock from HIRC */
        u32CLK_SRC = CLK_PLLCTL_PLLSRC_HIRC;
        u32PllSrcClk = __HIRC;

        /* u32NR start from 4 when FIN = 22.1184MHz to avoid calculation overflow */
        u32NR = 4UL;
    }

    if((u32PllFreq <= FREQ_200MHZ) && (u32PllFreq >= FREQ_50MHZ)) {

        /* Find best solution */
        u32Min = (uint32_t) - 1;
        u32MinNR = 0UL;
        u32MinNF = 0UL;
        u32MinNO = 0UL;
        u32basFreq = u32PllFreq;

        for(u32NO = 1UL; u32NO <= 4UL; u32NO++) {
            /* Break when get good results */
            if (u32Min == 0UL) {
                break;
            }

            if (u32NO != 3UL) {

                if(u32NO == 4UL) {
                    u32PllFreq = u32basFreq << 2;
                } else if(u32NO == 2UL) {
                    u32PllFreq = u32basFreq << 1;
                } else {
                }

                for(u32NR = 2UL; u32NR <= 32UL; u32NR++) {
                    /* Break when get good results */
                    if (u32Min == 0UL) {
                        break;
                    }

                    u32Tmp = u32PllSrcClk / u32NR;
                    if((u32Tmp >= 4000000UL) && (u32Tmp <= 8000000UL)) {
                        for(u32NF = 2UL; u32NF <= 513UL; u32NF++) {
                            /* u32Tmp2 is shifted 2 bits to avoid overflow */
                            u32Tmp2 = (((u32Tmp * 2UL) >> 2) * u32NF);

                            if((u32Tmp2 >= FREQ_50MHZ) && (u32Tmp2 <= FREQ_125MHZ)) {
                                u32Tmp3 = (u32Tmp2 > (u32PllFreq>>2)) ? u32Tmp2 - (u32PllFreq>>2) : (u32PllFreq>>2) - u32Tmp2;
                                if(u32Tmp3 < u32Min) {
                                    u32Min = u32Tmp3;
                                    u32MinNR = u32NR;
                                    u32MinNF = u32NF;
                                    u32MinNO = u32NO;

                                    /* Break when get good results */
                                    if(u32Min == 0UL) {
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        /* Enable and apply new PLL setting. */
        CLK->PLLCTL = u32CLK_SRC | ((u32MinNO - 1UL) << 14) | ((u32MinNR - 1UL) << 9) | (u32MinNF - 2UL);

        /* Wait for PLL clock stable */
        CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

        /* Actual PLL output clock frequency */
        u32PllClk = u32PllSrcClk / (u32MinNO * (u32MinNR)) * (u32MinNF) * 2UL;
    } else {
        /* Wrong frequency request. Just return default setting. */
        /* Apply default PLL setting and return */
        if(u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT) {
            CLK->PLLCTL = CLK_PLLCTL_72MHz_HXT;
        } else {
            CLK->PLLCTL = CLK_PLLCTL_72MHz_HIRC;
        }

        /* Wait for PLL clock stable */
        CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

        /* Actual PLL output clock frequency */
        u32PllClk = CLK_GetPLLClockFreq();
    }

    return u32PllClk;

}

/**
  * @brief      Disable PLL
  * @param      None
  * @return     None
  * @details    This function set PLL in Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisablePLL(void)
{
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
}


/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_PLLSTB_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (~300ms)
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

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_LXT
  *             - \ref CLK_CLKSEL0_STCLKSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  *             - \ref CLK_CLKSEL0_STCLKSEL_HCLK
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
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


/**
  * @brief      Power-down mode selected
  * @param[in]  u32PDMode is power down mode index. Including :
  *             - \ref CLK_PMUCTL_PDMSEL_PD
  *             - \ref CLK_PMUCTL_PDMSEL_LLPD
  *             - \ref CLK_PMUCTL_PDMSEL_SPD0
  *             - \ref CLK_PMUCTL_PDMSEL_SPD1
  *             - \ref CLK_PMUCTL_PDMSEL_DPD
  * @return     None
  * @details    This function is used to set power-down mode.
  */

void CLK_SetPowerDownMode(uint32_t u32PDMode)
{
    CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_PDMSEL_Msk)) | u32PDMode;
}



/**
 * @brief       Get power manager wake up source
 * @param[in]   None
 * @return      None
 * @details     This function get power manager wake up source.
 */

uint32_t CLK_GetPMUWKSrc(void)
{
    return (CLK->PMUSTS);
}


/**
 * @brief       Set specified GPIO as wake up source at Stand-by Power down mode
 * @param[in]   u32Port GPIO port. It could be 0:PA; 1:PB; 2:PC; 3:PD.
 * @param[in]   u32Pin  The pin of specified GPIO port. It could be 0 ~ 15.
 * @param[in]   u32TriggerType
 *              - \ref CLK_SPDWKPIN_RISING
 *              - \ref CLK_SPDWKPIN_FALLING
 * @param[in]   u32DebounceEn
 *              - \ref CLK_SPDWKPIN_DEBOUNCEEN
 *              - \ref CLK_SPDWKPIN_DEBOUNCEDIS
 * @return      None
 * @details     This function is used to set specified GPIO as wake up source
 *              at Stand-by Power down mode.
 */
void CLK_EnableSPDWKPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn)
{
    uint32_t u32tmpAddr = 0UL;
    uint32_t u32tmpVal = 0UL;

    // GPx Stand-by Power-down Wake-up Pin Select
    u32tmpAddr = (uint32_t)&CLK->PASWKCTL;
    u32tmpAddr += (0x4UL * u32Port);

    u32tmpVal = inpw((uint32_t *)u32tmpAddr);
    u32tmpVal = (u32tmpVal & ~(CLK_PASWKCTL_WKPSEL_Msk | CLK_PASWKCTL_PRWKEN_Msk | CLK_PASWKCTL_PFWKEN_Msk | CLK_PASWKCTL_DBEN_Msk | CLK_PASWKCTL_WKEN_Msk)) |
                (u32Pin << CLK_PASWKCTL_WKPSEL_Pos) | u32TriggerType | u32DebounceEn | CLK_SPDWKPIN_ENABLE;
	
	*(uint32_t *)u32tmpAddr = u32tmpVal;
}

/**
 * @brief       Disable SPD wakeup pin
 * @param[in]   u32Port GPIO port. It could be 0:PA; 1:PB; 2:PC; 3:PD.
 * @return      None
 * @details     This function is used to disable specified GPIO pin for wakeup 
 *              at Stand-by Power down mode.
 */
void CLK_DisableSPDWKPin(uint32_t u32Port)
{
	uint32_t u32tmpAddr = 0UL;

    // GPx Stand-by Power-down Wake-up Pin Select
    u32tmpAddr = (uint32_t)&CLK->PASWKCTL;
    u32tmpAddr += (0x4UL * u32Port);
	
	*(uint32_t *)u32tmpAddr = 0;
}

/*@}*/ /* end of group I94100_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_CLK_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
