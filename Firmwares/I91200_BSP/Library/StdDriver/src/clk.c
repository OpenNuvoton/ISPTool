/**************************************************************************//**
 * @file     clk.c
 * @version  V1.01
 * $Revision: 2 $
 * $Date: 16/12/5 2:35p $
 * @brief    I91200 CLK driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/ 

/** @addtogroup I91200_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup I91200_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

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
	return (CLK->PWRCTL&CLK_PWRCTL_HIRCEN_Msk)?((((CLK->CLKSEL0&CLK_CLKSEL0_HIRCFSEL_Msk)>>CLK_CLKSEL0_HIRCFSEL_Pos)==1)?__HIRC_32M:__HIRC_48M):0;
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
  *         - \ref UART0_MODULE
  *         - \ref UART1_MODULE
  *         - \ref SARADC_MODULE
  *         - \ref BFAL_MODULE
  *         - \ref PWM0CH01_MODULE
  *         - \ref PWM0CH23_MODULE
  *         - \ref SDADC_MODULE
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
  *         - \ref CLK_PWRCTL_LXT_EN
  *         - \ref CLK_PWRCTL_HIRC_EN
  *         - \ref CLK_PWRCTL_LIRC_EN
  *         - \ref CLK_PWRCTL_HXT_EN
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
  *         - \ref UART0_MODULE
  *         - \ref UART1_MODULE
  *         - \ref SARADC_MODULE
  *         - \ref BFAL_MODULE
  *         - \ref PWM0CH01_MODULE
  *         - \ref PWM0CH23_MODULE
  *         - \ref SDADC_MODULE
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
  *         - \ref CLK_PWRCTL_LXT_EN
  *         - \ref CLK_PWRCTL_HIRC_EN
  *         - \ref CLK_PWRCTL_LIRC_EN
  *         - \ref CLK_PWRCTL_HXT_EN
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
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HXT          | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LIRC         | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LXT          | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HCLK         | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_EXT          | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HXT          | x                            | 
  * |\ref I2C0_MODULE      | x                                    | x                            |
  * |\ref SPI0_MODULE      | x                                    | x                            |
  * |\ref SPI1_MODULE      | x                                    | x                            |
  * |\ref DPWM_MODULE      |\ref CLK_CLKSEL1_DPWMSEL_HXT          |\ref CLK_CLKDIV0_DPWM(x)      |  
  * |\ref DPWM_MODULE      |\ref CLK_CLKSEL1_DPWMSEL_HCLK         |\ref CLK_CLKDIV0_DPWM(x)      |  
  * |\ref UART0_MODULE     | x                                    |\ref CLK_CLKDIV0_UART0(x)     |
  * |\ref UART1_MODULE     | x                                    |\ref CLK_CLKDIV0_UART1(x)     |
  * |\ref BFAL_MODULE      | x                                    |\ref CLK_CLKDIV0_BIQ(x)       |    
  * |\ref RTC_MODULE       | x                                    | x                            |
  * |\ref PWM0CH01_MODULE  |\ref CLK_CLKSEL1_PWM0CH01SEL_LIRC     | x                            |
  * |\ref PWM0CH01_MODULE  |\ref CLK_CLKSEL1_PWM0CH01SEL_LXT      | x                            |
  * |\ref PWM0CH01_MODULE  |\ref CLK_CLKSEL1_PWM0CH01SEL_HCLK     | x                            |
  * |\ref PWM0CH01_MODULE  |\ref CLK_CLKSEL1_PWM0CH01SEL_HIRC     | x                            |
  * |\ref PWM0CH23_MODULE  |\ref CLK_CLKSEL1_PWM0CH23SEL_LIRC     | x                            |
  * |\ref PWM0CH23_MODULE  |\ref CLK_CLKSEL1_PWM0CH23SEL_LXT      | x                            |
  * |\ref PWM0CH23_MODULE  |\ref CLK_CLKSEL1_PWM0CH23SEL_HCLK     | x                            |
  * |\ref PWM0CH23_MODULE  |\ref CLK_CLKSEL1_PWM0CH23SEL_HIRC     | x                            |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_HCLK       |\ref CLK_CLKDIV0_SARADC(x)    |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_LIRC       |\ref CLK_CLKDIV0_SARADC(x)    |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_HIRC       |\ref CLK_CLKDIV0_SARADC(x)    |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_LXT        |\ref CLK_CLKDIV0_SARADC(x)    |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL1_SDADCSEL_HCLK        |\ref CLK_CLKDIV0_SDADC(x)     |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL1_SDADCSEL_HXT         |\ref CLK_CLKDIV0_SDADC(x)     |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL2_I2S0SEL_LIRC         | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL2_I2S0SEL_LXT          | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL2_I2S0SEL_HCLK         | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL2_I2S0SEL_HIRC         | x                            |
  * |\ref ANA_MODULE       | x                                    | x                            |
  *
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
	uint32_t u32tmp=0,u32sel=0,u32div=0;
    
	if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=0)
	{
		u32div =(uint32_t)&CLK->CLKDIV0+((MODULE_CLKDIV(u32ModuleIdx))*4);
		u32tmp = *(volatile uint32_t *)(u32div);
		u32tmp = ( u32tmp & ~(MODULE_CLKDIV_Msk(u32ModuleIdx)<<MODULE_CLKDIV_Pos(u32ModuleIdx)) ) | u32ClkDiv;
		*(volatile uint32_t *)(u32div) = u32tmp;
    }
		
	if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=0)
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
  *         - \ref CLK_CLKSEL0_HCLKSEL_HXT
  * @param  u32HIRCType type for HIRC clock source
  *         - \ref CLK_CLKSEL0_HIRCFSEL_48M (default)
  *         - \ref CLK_CLKSEL0_HIRCFSEL_32M
  * @param  u32ClkDiv is HCLK clock divider. Including :
  *         - \ref CLK_CLKDIV0_HCLK(x)
   * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32HIRCType, uint32_t u32ClkDiv)
{
	uint8_t u8Lock = SYS_Unlock();	
	CLK->CLKSEL0 = (CLK->CLKSEL0&~(CLK_CLKSEL0_HCLKSEL_Msk|CLK_CLKSEL0_HIRCFSEL_Msk))|(u32ClkSrc|u32HIRCType);
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
	CLK->CLKSEL0 = (CLK->CLKSEL0&(~CLK_CLKSEL0_STCLKSEL_Msk))|u32ClkSrc ;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function execute delay function.
  * @param  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                            48MHz => 349525us
  * @return None
  * @details    Use the SYSTICK to generate the delay time and the UNIT is in us.
  *             The SYSTICK clock source is from HCLK, i.e the same as system core clock.
  */
void CLK_SysTickDelay(uint32_t us)
{
	SYSTICK->RVR = us * CyclesPerUs;
	SYSTICK->CVR = (0x00);
	SYSTICK->CSR = SYSTICK_CSR_CLKSRC_Msk | SYSTICK_CSR_ENABLE_Msk;
    /* Waiting for down-count to zero */
	while((SYSTICK->CSR & SYSTICK_CSR_COUNTFLAG_Msk) == 0);
}

/**
  * @brief  Enters device into Deep Power Down mode and selects the acceptable causes for wakeup.
  * @param  u32DPDWakeupMode  PDP wake up mode.
  *         - \ref  CLK_DPDWAKEUP_PINOSC10K
  *         - \ref  CLK_DPDWAKEUP_PIN
  *         - \ref  CLK_DPDWAKEUP_OSC10K
  *         - \ref  CLK_DPDWAKEUP_POR
  * @param  u32TimerSel  Wake up time. WAKEUP after 64* (u32TimerSel+1) OSC10k clocks (6.4 * (u32TimerSel+1) ms).
  * @return None
  */
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel)
{
	uint8_t u8Lock = SYS_Unlock();
		
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	CLK->PWRCTL &= 0xFF0000FF;		
	CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_DPDEN_Pos; 	// Go into Deep Power Down upon WFI/WFE command	
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOPEN_Pos; 	// Don't go into stop mode upon WFI/WFE command	
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; 	// Don't go into Standby Power Down upon WFI/WFE command	
													// Power On Wakeup cannot be disabled
	
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_WKPINEN_Pos;				// Wakeup Pin (Pin 1): 0 Enabled 1 Disabled
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_LIRCDPDEN_Pos;			// 10K Oscillator: 0 Enabled 1 Disabled (No timed wakeup possible)
	CLK->WAKE10K |= u32TimerSel << CLK_WAKE10K_SELWKTMR_Pos;	// Sets wakeup timer time. WAKEUP after 64* (u32TimerSel+1) OSC10k clocks (6.4 * (u32TimerSel+1) ms).
	CLK->WAKE10K |= CLK_WAKE10K_WAKE10KEN_Msk;					// Enable wakeup timer time
		
	switch(u32DPDWakeupMode)
	{
		//Wakeup by Pin or OSC10k count
		case CLK_DPDWAKEUP_PINOSC10K:
			//Pin and OSC10K are already enabled above
		break;
		
		//Wakeup by Pin 
		case CLK_DPDWAKEUP_PIN:
			//Disable OSC16K Wakeup
			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_LIRCDPDEN_Pos;   
		break;

		//Wakeup by OSC10k count
		case CLK_DPDWAKEUP_OSC10K:
			//Disable PIN Wakeup
 			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_WKPINEN_Pos;    	
		break;

		//Wakeup by Power On Wakeup only
		case CLK_DPDWAKEUP_POR:
			//Disable PIN Wakeup and OSC10K Wakeup
		    CLK->PWRCTL |= (CLK_PWRCTL_LIRCDPDEN_Msk|CLK_PWRCTL_WKPINEN_Msk);    
		break;
	}
		
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Standby Power Down mode.
  * @return None
  */
void CLK_StandbyPowerDown(uint8_t u8IOHold)
{
	uint8_t u8Lock = SYS_Unlock();
		
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	CLK->PWRCTL &= 0xFF0000FF;		
	CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_SPDEN_Pos;  	// Go into Standby Power Down	upon WFI/WFE command
	//CLK->DBGPD &= ~CLK_DBGPD_DISPDREQ_Msk;
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOPEN_Pos;	// Don't go into Stop mode upon WFI/WFE command
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos;	// Don't go into Deep Power Down upon WFI/WFE command
	
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_WKPINEN_Pos;				// Wakeup Pin (Pin 1): 0 Enabled 1 Disabled
	//CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_LIRCDPDEN_Pos;			// 10K Oscillator: 0 Enabled 1 Disabled (No timed wakeup possible)
	/*CLK->WAKE10K |= 500 << CLK_WAKE10K_SELWKTMR_Pos;	// Sets wakeup timer time. WAKEUP after 64* (u32TimerSel+1) OSC10k clocks (6.4 * (u32TimerSel+1) ms).
	CLK->WAKE10K |= CLK_WAKE10K_WAKE10KEN_Msk;					// Enable wakeup timer time
	*/
	/*if(u8IOHold == 1)
		CLK->PWRCTL |= CLK_PWRCTL_HOLDIO_Msk;
	else
		CLK->PWRCTL &= CLK_PWRCTL_HOLDIO_Msk;*/
		
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Stop mode.
  * @param	u8FlashOnOff Flash will turn off/on when entering stop mode.
  * @return None
  */
void CLK_Stop(uint8_t u8FlashOnOff)
{
	uint8_t u8Lock = SYS_Unlock();
		
	//SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;	
	CLK->PWRCTL &= 0xFF0000FF;
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; 	// Don't into Standby Power Down	upon WFI/WFE command
	CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_STOPEN_Pos;	// Go go into Stop mode upon WFI/WFE command
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos;	// Don't go into Deep Power Down upon WFI/WFE command
	
	if(u8FlashOnOff == 1)
		CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_FLASHEN_Msk) | 0x2ul << CLK_PWRCTL_FLASHEN_Pos;
	else
		CLK->PWRCTL &= ~CLK_PWRCTL_FLASHEN_Msk;
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Deep sleep mode .
  * @return None
  */
void CLK_DeepSleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
	
	SYSINFO->SCR |= SYSINFO_SCR_SLPDEEP_Msk;
	
	CLK->PWRCTL &= 0xFF0000FF;		
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOPEN_Pos;  //Go into Stop mode upon upon WFI/WFE command
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; 	//Don't go into Standby Power Down upon WFI/WFE command
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos;	// Don't go into Deep Power Down upon WFI/WFE command
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Sleep mode.
  * @return None
  */
void CLK_Sleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
		
	SYSINFO->SCR &= ~SYSINFO_SCR_SLPDEEP_Msk;
	
	CLK->PWRCTL &= 0xFF0000FF;	
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_STOPEN_Pos;  //Don't go into Stop mode upon WFI/WFE command
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_SPDEN_Pos; //Don't go into Standby Power Down upon WFI/WFE command
	CLK->PWRCTL |= 0x0ul << CLK_PWRCTL_DPDEN_Pos; // Don't go into Deep Power Down upon WFI/WFE command
		
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

/*@}*/ /* end of group I91200_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_CLK_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
