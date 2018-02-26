/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/10/22 2:35p $
 * @brief    ISD9000 CLK driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup ISD9000_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief  Enters device into Deep Power Down mode and selects the acceptable causes for wakeup.
  * @param  u32DPDWakeupMode  PDP wake up mode.
  *         - \ref  CLK_DPDWAKEUP_PINOSC10K
  *         - \ref  CLK_DPDWAKEUP_PIN
  *         - \ref  CLK_DPDWAKEUP_OSC10K
  *         - \ref  CLK_DPDWAKEUP_POR
  * @param  u32TimerSel  Wakeup time selection from OSC 10k.
  *         - \ref  CLK_DPDWAKETIME_12ms
  *         - \ref  CLK_DPDWAKETIME_25ms
  *         - \ref  CLK_DPDWAKETIME_50ms
  *         - \ref  CLK_DPDWAKETIME_100ms
  * @return None
  */
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel)
{
	uint8_t u8Lock = SYS_Unlock();
		
	//SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;		
	CLK->PWRCTL &= 0xFF0000FF;		
	CLK->PWRCTL |= CLK_PWRCTL_DPDEN_Msk; //Go into Deep Power Down upon WFI/WFE command	
	CLK->PWRCTL |= u32TimerSel << CLK_PWRCTL_SELWKTMR_Pos;	 //Sets wakeup timer time
		
	switch(u32DPDWakeupMode)
	{
		case CLK_DPDWAKEUP_PINOSC10K:	    //Wakeup by Pin or OSC10k count 
			break;                         //Pin and OSC10K are enabled above
		    
		case CLK_DPDWAKEUP_PIN:		                              //Wakeup by Pin 
			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_LIRCDPDEN_Pos;   //Disable OSC16K Wakeup
			break;

		case CLK_DPDWAKEUP_OSC10K:	                            //Wakeup by OSC10k count 
 			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_WKPINEN_Pos;    //Disable PIN Wakeup
			break;
			
		case CLK_DPDWAKEUP_POR:          	//Wakeup by Power On Wakeup only  
		    CLK->PWRCTL |= (CLK_PWRCTL_LIRCDPDEN_Msk|CLK_PWRCTL_WKPINEN_Msk);    //Disable PIN Wakeup and OSC10K Wakeup
		    break;
		
	}
		
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Standby Power Down mode.
  * @return None
  */
void CLK_StandbyPowerDown(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = (CLK->PWRCTL&(~CLK_PWRCTL_FLASHPWR_Msk))|(0x02<<CLK_PWRCTL_FLASHPWR_Pos);		
	CLK->PWRCTL |= CLK_PWRCTL_STOPEN_Msk; 
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Deep sleep mode .
  * @return None
  */
void CLK_DeepSleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;	
	CLK->PWRCTL &= 0xFF0000FF;		
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Sleep mode.
  * @return None
  */
void CLK_Sleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
		
	//SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;		
	CLK->PWRCTL &= 0xFF0000FF;	
		
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function enable power down mode
  */
/*void CLK_EnablePowerDownMode(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = (CLK->PWRCTL & (~CLK_PWRCTL_PD_WAIT_CPU_Msk)) | CLK_PWRCTL_PD_WAIT_CPU_Msk;
	CLK->PWRCTL = (CLK->PWRCTL & (~CLK_PWRCTL_PWR_DOWN_Msk)) | CLK_PWRCTL_PWR_DOWN_Msk;
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function disable power down mode
  */
/*void CLK_DisablePowerDownMode(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = CLK->PWRCTL & (~CLK_PWRCTL_PD_WAIT_CPU_Msk);
	CLK->PWRCTL = CLK->PWRCTL & (~CLK_PWRCTL_PWR_DOWN_Msk);
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function clear wake up status flag
  */
/*void CLK_ClearWakeUpStatusFlag(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = (CLK->PWRCTL & (~CLK_PWRCTL_WINT_STS_Msk)) | CLK_PWRCTL_WINT_STS_Msk;
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function enable wake up interrupt
  */
/*void CLK_EnableWakeUpInterrupt(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = (CLK->PWRCTL & (~CLK_PWRCTL_WINT_EN_Msk)) | CLK_PWRCTL_WINT_EN_Msk;
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function disable wake up interrupt
  */
/*void CLK_DisableWakeUpInterrupt(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = CLK->PWRCTL & (~CLK_PWRCTL_WINT_EN_Msk);
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function enable wake up delay
  */
/*void CLK_EnableWakeUpDelay(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = (CLK->PWRCTL & (~CLK_PWRCTL_WU_DLY_Msk)) | CLK_PWRCTL_WU_DLY_Msk;
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function disable wake up delay
  */
/*void CLK_DisableWakeUpDelay(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL = CLK->PWRCTL & (~CLK_PWRCTL_WU_DLY_Msk);
	SYS_Lock(u8Lock);
}*/

/**
  * @brief  This function enable clock source
  * @param  u32ClkMask is clock source mask:
  *         - \ref CLK_PWRCTL_LXTEN_Msk
  *         - \ref CLK_PWRCTL_HIRCEN_Msk
  *         - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return None
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL |= u32ClkMask;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function disable clock source
  * @param  u32ClkMask is clock source mask:
  *         - \ref CLK_PWRCTL_LXTEN_Msk
  *         - \ref CLK_PWRCTL_HIRCEN_Msk
  *         - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return None
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL &= ~u32ClkMask;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function set HCLK clock source and HCLK clock divider
  * @param  u32ClkSrc is HCLK clock source :
  *         - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *         - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  *         - \ref CLK_CLKSEL0_HCLKSEL_LXT
  * @param  u32ClkDiv is HCLK clock divider:
  *         - \ref CLK_CLKDIV_HCLK(x)
  * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;
	CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | u32ClkDiv;
	SYS_Lock(u8Lock);
	SystemCoreClockUpdate();
}

/**
  * @brief      Get HCLK frequency  
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
	SystemCoreClockUpdate();
	return SystemCoreClock;
}

/**
  * @brief  This function enable module clock
  * @param  u32ModuleIdx is module index :
  *         - \ref PDMA_MODULE
  *         - \ref SPIM_MODULE
  *         - \ref ISP_MODULE
  *         - \ref APU_MODULE
  *         - \ref WDT_MODULE
  *         - \ref RTC_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref TMR2_MODULE
  *         - \ref TMRF_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref PWM1_MODULE
  *         - \ref ADC_MODULE
  *         - \ref DPWM_MODULE
  *         - \ref UART0_MODULE
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
	uint8_t u8Lock = SYS_Unlock();
	*(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_AHPBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function disable module clock
  * @param  u32ModuleIdx is module index :
  *         - \ref PDMA_MODULE
  *         - \ref SPIM_MODULE
  *         - \ref ISP_MODULE
  *         - \ref APU_MODULE
  *         - \ref WDT_MODULE
  *         - \ref RTC_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref TMR2_MODULE
  *         - \ref TMRF_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref PWM1_MODULE
  *         - \ref ADC_MODULE
  *         - \ref DPWM_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
	uint8_t u8Lock = SYS_Unlock();
	*(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_AHPBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function set selected module clock source and module clock divider
  * @param  u32ModuleIdx is module index.
  * @param  u32ClkSrc is module clock source.
  * @param  u32ClkDiv is module clock divider.
  * @return None
  * @details Valid parameter combinations listed in following table:
  *
  * |Module index          |Clock source                                 |Divider                       |
  * | :------------------- | :------------------------------------------ | :-------------------------   |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048         | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_LXT                  | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_LIRC                 | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_HIRC                 | x                            |
  * |\ref ADC_MODULE       |\ref CLK_CLKSEL1_ADCSEL_HCLK1                |\ref CLK_CLKDIV_ADC(x)        |
  * |\ref ADC_MODULE       |\ref CLK_CLKSEL1_ADCSEL_HCLK2                |\ref CLK_CLKDIV_ADC(x)        |
  * |\ref ADC_MODULE       |\ref CLK_CLKSEL1_ADCSEL_HIRC                 |\ref CLK_CLKDIV_ADC(x)        |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HCLK                | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_LXT                 | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_EXT                 | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_LIRC                | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HIRC                | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HCLK                | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LXT                 | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LIRC                | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_EXT                 | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LIRC                | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_HCLK                | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_LXT                 | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_LIRC                | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_HIRC                | x                            |
  * |\ref TMRF_MODULE      |\ref CLK_CLKSEL1_TMRFSEL_LXT_32K_DIV32       | x                            |
  * |\ref TMRF_MODULE      |\ref CLK_CLKSEL1_TMRFSEL_LXT_32K_DIV128      | x                            |
  * |\ref TMRF_MODULE      |\ref CLK_CLKSEL1_TMRFSEL_LIRC_DIV32          | x                            |
  * |\ref TMRF_MODULE      |\ref CLK_CLKSEL1_TMRFSEL_LIRC_DIV128         | x                            |
  * |\ref TMRF_MODULE      |\ref CLK_CLKSEL1_TMRFSEL_HIRC_DIV32768       | x                            |
  * |\ref TMRF_MODULE      |\ref CLK_CLKSEL1_TMRFSEL_HIRC_DIV32768_DIV4  | x                            |
  * |\ref RTC_MODULE       |\ref CLK_CLKSEL1_RTCSEL_LIRC                 | x                            |
  * |\ref RTC_MODULE       |\ref CLK_CLKSEL1_RTCSEL_LXT                  | x                            | 
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_HCLK                | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_LXT                 | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_LIRC                | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_HIRC                | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_HCLK                | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_LXT                 | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_LIRC                | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_HIRC                | x                            |
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
	uint32_t u32tmp=0,u32sel=0,u32div=0;
  
	uint8_t u8Lock = SYS_Unlock();
	
	if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk)
	{
		u32div =(uint32_t)&CLK->CLKDIV+((MODULE_CLKDIV(u32ModuleIdx))*4);
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
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function enable LDO output
  * @param  u32LDOSel LDO select voltage.
  *         - \ref  CLK_LDOSEL_1_8V
  *         - \ref  CLK_LDOSEL_2_4V
  *         - \ref  CLK_LDOSEL_2_5V
  *         - \ref  CLK_LDOSEL_2_7V
  *         - \ref  CLK_LDOSEL_3_0V
  *         - \ref  CLK_LDOSEL_3_3V
  *         - \ref  CLK_LDOSEL_1_5V
  *         - \ref  CLK_LDOSEL_1_7V
  * @return None
  */
void CLK_EnableLDO( uint32_t u32LDOSel )
{
	CLK->DPDSTATE = (CLK->DPDSTATE&(~(CLK_DPDSTATE_LDOVOLT_Msk|CLK_DPDSTATE_LDOPD_Msk)))|u32LDOSel;
}

/**
  * @brief  This function disable LDO output
  */
void CLK_DisableLDO(void)
{
	CLK->DPDSTATE |= CLK_DPDSTATE_LDOPD_Msk;
}

void CLK_EnableX32Filter(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL |= CLK_PWRCTL_XTL32K_FILTER_Msk;
	SYS_Lock(u8Lock);
}

void CLK_DisableX32Filter(void)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL &= ~CLK_PWRCTL_XTL32K_FILTER_Msk;
	SYS_Lock(u8Lock);	
}


/*@}*/ /* end of group ISD9000_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_CLK_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
