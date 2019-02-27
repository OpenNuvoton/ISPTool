/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//***********************************************************************************************************

#include "ML51.h"


 /**
 * @brief This function config PWM clock base
 * @param[in] u8PWMCLKSource PWM0 clock source from HIRC or TIMER1.
                   - \ref  PWM_FSYS
									 - \ref  PWM_TIMER1
 * @param[in] u8PWM0CLKDIV the divider value of PWM clock.  - \ref (1\2\4\8\16\32\64\128) 
 * @return none
 * @note        
 * @example PWM0_ClockSource(PWM0_HIRC,128);
  */
void PWM0_ClockSource(unsigned char u8PWMCLKSource, unsigned char u8PWM0CLKDIV)
{
		switch (u8PWMCLKSource)
		{
				case PWM_FSYS:	clr_CKCON_PWMCKS; break;
				case PWM_TIMER1: set_CKCON_PWMCKS; break;
		}
		SFRS = 0x01;
		switch (u8PWM0CLKDIV)
		{
				case 1:	  PWM0CON1&=0xF8; break;
				case 2:	  PWM0CON1&=0xF8;PWM0CON1|=0x02; break;
				case 4:	  PWM0CON1&=0xF8;PWM0CON1|=0x02; break;
				case 8:	  PWM0CON1&=0xF8;PWM0CON1|=0x03; break;
				case 16:	PWM0CON1&=0xF8;PWM0CON1|=0x04; break;
				case 32:	PWM0CON1&=0xF8;PWM0CON1|=0x05; break;
				case 64:	PWM0CON1&=0xF8;PWM0CON1|=0x06; break;
				case 128:	PWM0CON1&=0xF8;PWM0CON1|=0x07; break;
		}
}

 /**
 * @brief This function config PWM generator 
 * @param[in] u8ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u8OPMode PWM run mode select from Independent, Complementary or Synchronized mode.
 * @param[in] u8PwmType select Edge-Aligned Type or Center-Aligned Type
 * @param[in] u32PWM0Frequency Target generator frequency, note the actually PWM period is 16bit value. from 0x0000 ~ 0xFFFF
 * @param[in] u16DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return none
 * @note  none       
 * @example PWM0_ConfigOutputChannel(0,Independent,EdgeAligned,0x6FF,10);
  */
void PWM0_ConfigOutputChannel(unsigned char u8PWM0ChannelNum,
															unsigned char u8PWM0OPMode,
															unsigned char u8PWM0PwmType,
                              unsigned long u32PWM0Frequency,
                              unsigned int u16PWM0DutyCycle)
{
	SFRS = 0x01;
	switch (u8PWM0OPMode)
	{
			case Independent:		PWM0CON1&=0x3F;break;
			case Complementary:	PWM0CON1&=0x3F;PWM0CON1|=0x40;break;
			case Synchronous:	  PWM0CON1&=0x3F;PWM0CON1|=0x80;break;
	}
	switch (u8PWM0PwmType)
	{
			case EdgeAligned:		PWM0CON1&=0xEF;break;
			case CenterAligned:	PWM0CON1|=0x10;break;
	}
	switch (u8PWM0ChannelNum)
	{
		  case 0:	PWM0C1H=(u32PWM0Frequency*u16PWM0DutyCycle/100)>>8;PWM0C0L=(u32PWM0Frequency*u16PWM0DutyCycle/100);break;
		  case 1:	PWM0C1H=(u32PWM0Frequency*u16PWM0DutyCycle/100)>>8;PWM0C1L=(u32PWM0Frequency*u16PWM0DutyCycle/100);break;
		  case 2:	PWM0C2H=(u32PWM0Frequency*u16PWM0DutyCycle/100)>>8;PWM0C2L=(u32PWM0Frequency*u16PWM0DutyCycle/100);break;
		  case 3:	PWM0C3H=(u32PWM0Frequency*u16PWM0DutyCycle/100)>>8;PWM0C3L=(u32PWM0Frequency*u16PWM0DutyCycle/100);break;
		  case 4:	PWM0C4H=(u32PWM0Frequency*u16PWM0DutyCycle/100)>>8;PWM0C4L=(u32PWM0Frequency*u16PWM0DutyCycle/100);break;
		  case 5:	PWM0C5H=(u32PWM0Frequency*u16PWM0DutyCycle/100)>>8;PWM0C5L=(u32PWM0Frequency*u16PWM0DutyCycle/100);break;
	}
	PWM0PH = u32PWM0Frequency>>8;
	PWM0PL = u32PWM0Frequency;
}

 /**
 * @brief This function config PWM Complementary pair inset dead zone time 
 * @param[in] u8PWM0Pair PWM0 channel pair need insert pair define. (PWM0_CH01 / PWM0_CH23 / PWM0_CH45 / PWM0_ALL) 
 * @param[in] u8PWM0DZValue the insert value.  PWM dead-time = (u8PWM0DZValue+1)/Fsys
 * @return none
 * @note        
 * @example PWM0_DeadZoneEnable(PWM0_CH01,0x55);
  */
void PWM0_DeadZoneEnable(unsigned char u8PWM0Pair, unsigned int u16PWM0DZValue)
{
		SFRS = 0x01;
		BIT_TMP=EA;EA=0;
		ACC=u16PWM0DZValue&0x0100>>4;
		TA=0xAA;TA=0x55;PWM0DTEN|=ACC;
		switch (u8PWM0Pair)
		{
				case PWM0_CH01:	TA=0xAA;TA=0x55;PWM0DTEN|=0x01; break;
				case PWM0_CH23: TA=0xAA;TA=0x55;PWM0DTEN|=0x02; break;
				case PWM0_CH45: TA=0xAA;TA=0x55;PWM0DTEN|=0x04; break;
				case PWM0_ALL:  TA=0xAA;TA=0x55;PWM0DTEN|=0x07; break;
		}
		ACC=u16PWM0DZValue;
		TA=0xAA;TA=0x55;
		PWM0DTCNT=ACC;
		EA=BIT_TMP;
}

/**
* @brief This function disable all PWM Complementary pair inset dead zone time 
* @param[in] none
* @return none
* @note        
* @example PWM0_DeadZone_ALL_Disable();
*/
void PWM0_DeadZone_ALL_Disable(void)
{
		SFRS = 0x01;
		TA=0xAA;TA=0x55;
		PWM0DTEN=0x00;
		EA=BIT_TMP;
}
		
/**
* @brief This function action all PWM run
* @param[in] none
* @return none
* @note        
* @example PWM0_RUN();
*/
void PWM0_RUN(void)
{
		set_PWM0CON0_PWMRUN;
}

/**
* @brief This function action all PWM stop run
* @param[in] none
* @return none
* @note        
* @example PWM0_RUN();
*/
void PWM0_STOP(void)
{
	clr_PWM0CON0_PWMRUN;
}

#if 0  		//reserved PWM1 function.
 /**
 * @brief This function config PWM1 generator 
 * @param[in] u8PWM1ChannelNum PWM1 channel number. Valid values are between 0~5
 * @param[in] u8PWM1OPMode PWM1 run mode select from Independent, Complementary or Synchronized mode.
 * @param[in] u8PWM1PwmType select Edge-Aligned Type or Center-Aligned Type
 * @param[in] u32PWM1Frequency Target generator frequency, note the actually PWM period is 16bit value. from 0x0000 ~ 0xFFFF
 * @param[in] u16PWM1DutyCycle Target generator duty cycle percentage of PWM1. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return none
 * @note        
 * @example PWM1_ConfigOutputChannel(0,Synchronized,EdgeAligned,0x6FF,10);
  */
void PWM1_ConfigOutputChannel(unsigned char u8PWM1ChannelNum,
															unsigned char u8PWM1OPMode,
															unsigned char u8PWM1PwmType,
                              unsigned long u32PWM1Frequency,
                              unsigned int u16PWM1DutyCycle)
{
		SFRS = 0x02;
		switch (u8PWM1OPMode)
		{
				case Independent:		PWM1CON1&=0x3F;break;
				case Complementary:	PWM1CON1&=0x3F;PWM1CON1|=0x40;break;
				case Synchronized:	PWM1CON1&=0x3F;PWM1CON1|=0x80;break;
		}
		switch (u8PWM1PwmType)
		{
				case EdgeAligned:		PWM1CON1&=0xEF;break;
				case CenterAligned:	PWM1CON1|=0x10;break;
		}
		switch (u8PWM1ChannelNum)
		{
				case 0:	PWM1C0H=(u32PWM1Frequency*u16PWM1DutyCycle)/100)>>8;PWM1C0L=(u32PWM1Frequency*u16PWM1DutyCycle)/100);break;
				case 1:	PWM1C1H=(u32PWM1Frequency*u16PWM1DutyCycle)/100)>>8;PWM1C1L=(u32PWM1Frequency*u16PWM1DutyCycle)/100);break;
				case 2:	PWM1C2H=(u32PWM1Frequency*u16PWM1DutyCycle)/100)>>8;PWM1C2L=(u32PWM1Frequency*u16PWM1DutyCycle)/100);break;
				case 3: PWM1C3H=(u32PWM1Frequency*u16PWM1DutyCycle)/100)>>8;PWM1C3L=(u32PWM1Frequency*u16PWM1DutyCycle)/100);break;
				case 4:	PWM1C4H=(u32PWM1Frequency*u16PWM1DutyCycle)/100)>>8;PWM1C4L=(u32PWM1Frequency*u16PWM1DutyCycle)/100);break;
				case 5:	PWM1C5H=(u32PWM1Frequency*u16PWM1DutyCycle)/100)>>8;PWM1C5L=(u32PWM1Frequency*u16PWM1DutyCycle)/100);break;
		}
		PWM1PH = u32PWM1Frequency>>8;
		PWM1PL = u32PWM1Frequency;
}


 /**
 * @brief This function config PWM Complementary pair inset dead zone time 
 * @param[in] u8PWM1Pair PWM0 channel pair need insert pair define. (PWM1_CH01 / PWM1_CH23 / PWM1_CH45 / PWM1_ALL) 
 * @param[in] u8PWM1DZValue the insert value.  PWM dead-time = (u8PWM0DZValue+1)/Fsys
 * @return none
 * @note        
 * @example PWM0_DeadZoneEnable(PWM0_CH01,0x55);
  */
void PWM1_DeadZoneEnable(unsigned char u8PWM1Pair, unsigned char u8PWM1DZValue)
{
		SFRS = 0x02;
		switch (u8PWM1Pair)
		{
				case PWM1_CH01:	SFRS=0x02;PWM1DTEN|=0x01; break;
				case PWM1_CH23: SFRS=0x02;PWM1DTEN|=0x02; break;
				case PWM1_CH45: SFRS=0x02;PWM1DTEN|=0x04; break;
				case PWM1_ALL: PWM1DTEN|=0x07; break;
		}
		PWM1DTCNT=u8PWM1DZValue;
}
		
#endif 