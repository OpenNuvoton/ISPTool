/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/07/17 2:35p $
 * @brief    ISD9100 CLK driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include  "ISD9100.h"

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup ISD9100_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Get external low speed crystal clock frequency
  * @return     External low speed crystal clock frequency
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
	if(CLK->PWRCTL & CLK_PWRCTL_LXTEN_Msk )
		return __LXT;
	else
		return 0;
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
  * @brief      Get HIRC frequency  
  * @return     HIRC frequency
  * @details    This function get HIRC frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHIRCFreq(void)
{
	return gau32HiRCSrcTbl[(CLK->CLKSEL0 & CLK_CLKSEL0_HIRCFSEL_Msk)>>CLK_CLKSEL0_HIRCFSEL_Pos];
}

/**
  * @brief  This function disable module clock
  * @param  u32ModuleIdx is module index. Including :
  *         - \ref PDMA_MODULE
  *         - \ref ISP_MODULE
  *         - \ref WDT_MODULE
  *         - \ref RTC_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref I2C0_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref DPWM_MODULE
  *         - \ref UART_MODULE
  *         - \ref BFAL_MODULE
  *         - \ref CRC_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref ACMP_MODULE
  *         - \ref SBRAM_MODULE    
  *         - \ref ADC_MODULE
  *         - \ref I2S0_MODULE
  *         - \ref ANA_MODULE 
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
	*(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_AHPBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief  This function disable clock source
  * @param  u32ClkMask is clock source mask. Including:
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
  * @brief  This function enable module clock
  * @param  u32ModuleIdx is module index. Including :
  *         - \ref PDMA_MODULE
  *         - \ref ISP_MODULE
  *         - \ref WDT_MODULE
  *         - \ref RTC_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref I2C0_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref DPWM_MODULE
  *         - \ref UART_MODULE
  *         - \ref BFAL_MODULE
  *         - \ref CRC_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref ACMP_MODULE
  *         - \ref SBRAM_MODULE     
  *         - \ref ADC_MODULE
  *         - \ref I2S0_MODULE
  *         - \ref ANA_MODULE 
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
	*(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_AHPBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief  This function enable clock source
  * @param  u32ClkMask is clock source mask. Including:
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
  * @brief  This function set selected module clock source and module clock divider
  * @param  u32ModuleIdx is module index.
  * @param  u32ClkSrc is module clock source.
  * @param  u32ClkDiv is module clock divider.
  * @return None
  * @details Valid parameter combinations listed in following table:
  *
  * |Module index          |Clock source                          |Divider                       |
  * | :------------------- | :-------------------------------     | :-------------------------   |
  * |\ref PDMA_MODULE      | x                                    | x                            |
  * |\ref ISP_MODULE       | x                                    | x                            |
  * |                      |                                      |                              |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_HIRC          | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_LXT           | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_LIRC          | x                            |
  * |\ref RTC_MODULE       | x                                    | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_LIRC         | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_LXT          | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HCLK         | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_EXT          | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HIRC         | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LIRC         | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LXT          | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HCLK         | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_EXT          | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HIRC         | x                            | 
  * |\ref I2C0_MODULE      | x                                    | x                            |
  * |\ref SPI0_MODULE      | x                                    | x                            |
  * |\ref DPWM_MODULE      |\ref CLK_CLKSEL1_DPWMSEL_HIRC         | x                            |  
  * |\ref DPWM_MODULE      |\ref CLK_CLKSEL1_DPWMSEL_HIRC2X       | x                            |  
  * |\ref UART_MODULE      | x                                    |\ref CLK_CLKDIV0_UART(x)      |
  * |\ref BFAL_MODULE      | x                                    | x                            |    
  * |\ref RTC_MODULE       | x                                    | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0CH01SEL_LIRC     | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0CH01SEL_LXT      | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0CH01SEL_HCLK     | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0CH01SEL_HIRC     | x                            |
  * |\ref ACMP_MODULE      | x                                    | x                            |
  * |\ref SBRAM_MODULE     | x                                    | x                            |
  * |\ref ADC_MODULE       | x                                    |\ref CLK_CLKDIV0_ADC(x)       |
  * |\ref I2S0_MODULE      | x                                    | x                            |
  * |\ref ANA_MODULE       | x                                    | x                            |
  *
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
  * @brief  This function set HCLK clock source and HCLK clock divider
  * @param  u32ClkSrc is HCLK clock source. Including :
  *         - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  *         - \ref CLK_CLKSEL0_HCLKSEL_LXT
  *         - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  * @param  u32HIRCType type for HIRC (OSC48M) clock source
  *         - \ref CLK_CLKSEL0_HIRCFSEL_48M (default)
  *         - \ref CLK_CLKSEL0_HIRCFSEL_32M
  * @param  u32ClkDiv is HCLK clock divider. Including :
  *         - \ref CLK_CLKDIV0_HCLK(x)
   * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32HIRCType, uint32_t u32ClkDiv)
{
	uint8_t u8Lock = SYS_Unlock();
	
	CLK->CLKSEL0 = (CLK->CLKSEL0 & ~(CLK_CLKSEL0_HCLKSEL_Msk|CLK_CLKSEL0_HIRCFSEL_Msk)) | u32ClkSrc | u32HIRCType;
	CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;
	CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | u32ClkDiv;
	
	SYS_Lock(u8Lock);
	
	SystemCoreClockUpdate();
}

/**
  * @brief  This function set SysTick clock source
  * @param  u32ClkSrc is SysTick clock source. Including :
  *         - \ref CLK_CLKSEL0_STCLKSEL_LIRC
  *         - \ref CLK_CLKSEL0_STCLKSEL_LXT
  *         - \ref CLK_CLKSEL0_STCLKSEL_LIRC_DIV2
  *         - \ref CLK_CLKSEL0_STCLKSEL_HCLK_DIV2
  *         - \ref CLK_CLKSEL0_STCLKSEL_HIRC_DIV2
  * @return None
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{                         
	uint8_t u8Lock = SYS_Unlock();
	CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | u32ClkSrc ;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function execute delay function.
  * @param  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                            48MHz => 349525us
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
}

/**
  * @brief  Enters device into Deep Power Down mode and selects the acceptable causes for wakeup.
  * @param  u32DPDWakeupMode  PDP wake up mode.
  *         - \ref  CLK_DPDWAKEUP_PINOSC16K
  *         - \ref  CLK_DPDWAKEUP_PIN
  *         - \ref  CLK_DPDWAKEUP_OSC16K
  *         - \ref  CLK_DPDWAKEUP_POR
  * @param  u32TimerSel  Wakeup time selection from OSC 16k.
  *         - \ref  CLK_DPDWAKETIME_12ms
  *         - \ref  CLK_DPDWAKETIME_25ms
  *         - \ref  CLK_DPDWAKETIME_50ms
  *         - \ref  CLK_DPDWAKETIME_100ms
  * @return None
  */
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel)
{
	uint8_t u8Lock = SYS_Unlock();
		
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;		
	CLK->PWRCTL &= 0xFF0000FF;		
	CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_DPDEN_Pos; //Go into Deep Power Down upon WFI/WFE command	
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOP_Pos; //Don't go into stop mode upon WFI/WFE command	
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; //Don't go into Standby Power Down upon WFI/WFE command	
	//Power On Wakeup wakeup cannot be disabled
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_WKPINEN_Pos;	 //Wakeup Pin (Pin 1): 0 Enabled 1 Disabled
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_LIRCDPDEN_Pos;  //16k Oscillator: 0 Enabled 1 Disabled (No timed wakeup possible)	
	CLK->PWRCTL |= u32TimerSel << CLK_PWRCTL_SELWKTMR_Pos;	 //Sets wakeup timer time
		
	switch(u32DPDWakeupMode)
	{
		case CLK_DPDWAKEUP_PINOSC16K:	    //Wakeup by Pin or OSC10k count 
			break;                         //Pin and OSC10K are enabled above
		    
		case CLK_DPDWAKEUP_PIN:		                              //Wakeup by Pin 
			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_LIRCDPDEN_Pos;   //Disable OSC16K Wakeup
			break;

		case CLK_DPDWAKEUP_OSC16K:	                            //Wakeup by OSC10k count 
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
		
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;	
	CLK->PWRCTL &= 0xFF0000FF;		
	CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_SPDEN_Pos; //Go into Standby Power Down	upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOP_Pos;	//Don't go into Stop mode upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos;	// Don't go into Deep Power Down upon WFI/WFE command
			
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
	CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_STOP_Pos;  //Go into Stop mode upon upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; //Don't go into Standby Power Down upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos;	 // Don't go into Deep Power Down upon WFI/WFE command
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Sleep mode.
  * @return None
  */
void CLK_Sleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
		
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;		
	CLK->PWRCTL &= 0xFF0000FF;	
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOP_Pos;  //Don't go into Stop mode upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; //Don't go into Standby Power Down upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos; // Don't go into Deep Power Down upon WFI/WFE command
		
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function enable LDO output
  * @param  u32LDOSel LDO select voltage.
  *         - \ref  CLK_LDOSEL_3_0V
  *         - \ref  CLK_LDOSEL_1_8V
  *         - \ref  CLK_LDOSEL_2_4V
  *         - \ref  CLK_LDOSEL_3_3V
  * @return None
  */
void CLK_EnableLDO( uint32_t u32LDOSel )
{
	uint8_t u8Lock = SYS_Unlock();
	
	CLK->APBCLK0 |= CLK_APBCLK0_ANACKEN_Msk;
	ANA->LDOPD &= ~ANA_LDOPD_PD_Msk;
	ANA->LDOSEL = (ANA->LDOSEL&~ANA_LDOSEL_LDOSEL_Msk)|u32LDOSel; 
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function disable LDO output
  */
void CLK_DisableLDO(void)
{
	uint8_t u8Lock = SYS_Unlock();
	
	ANA->LDOPD |= ANA_LDOPD_PD_Msk;
	
	SYS_Lock(u8Lock);
}

/*@}*/ /* end of group ISD9100_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_CLK_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
