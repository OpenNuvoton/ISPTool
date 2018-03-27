/**************************************************************************//**
 * @file     pwm.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 2:52p $
 * @brief    ISD9000 series PWM driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_PWM_Driver PWM Driver
  @{
*/


/** @addtogroup ISD9000_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief Configure PWM capture and get the nearest unit time.
 * @param[in] pwm The base address of PWM module
 * @param[in] u32UnitTimeNsec The unit time of counter
 * @param[in] u32CaptureEdge The condition to latch the counter. This parameter is not used
 * @return The nearest unit time in nano second.
 * @details This function is used to configure PWM capture and get the nearest unit time.
 */
uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  //uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge)
{
    uint32_t u32Src = 0;
    uint32_t u32PWMClockSrc = 0;
    uint32_t u32NearestUnitTimeNsec = 0;
    uint8_t  u8Divider = 1;
    /* this table is mapping divider value to register configuration */
    uint32_t u32PWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
    uint16_t u16Prescale = 2;
    uint16_t u16CNR = 0xFFFF;

    if(pwm == PWM0)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_PWM0SEL_Msk) >> CLK_CLKSEL1_PWM0SEL_Pos;
    if(pwm == PWM1)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_PWM1SEL_Msk) >> CLK_CLKSEL1_PWM1SEL_Pos;

    switch (u32Src)
    {
				case 0:
						u32PWMClockSrc = CLK_GetHCLKFreq();
						break;
				case 1:
						u32PWMClockSrc = __LXT;
						break;
				case 2:
						u32PWMClockSrc = __LIRC;
						break;
				case 3:
						u32PWMClockSrc = __HIRC;
						break;
    }

    u32PWMClockSrc /= 1000;
    for(; u16Prescale <= 0x100; u16Prescale++)
    {
        u32NearestUnitTimeNsec = (1000000 * u16Prescale * u8Divider) / u32PWMClockSrc;
        if(u32NearestUnitTimeNsec < u32UnitTimeNsec)
        {
            if((u16Prescale == 0x100) && (u8Divider == 16))  //limit to the maximum unit time(nano second)
                break;
            if(u16Prescale == 0x100)
            {
                u16Prescale = 2;
                u8Divider <<= 1; // clk divider could only be 1, 2, 4, 8, 16
                continue;
            }
            if(!((1000000  * ((u16Prescale * u8Divider) + 1)) > (u32NearestUnitTimeNsec * u32PWMClockSrc)))
                break;
            continue;
        }
        break;
    }

    // Store return value here 'cos we're gonna change u8Divider & u16Prescale & u16CNR to the real value to fill into register
    u16Prescale -= 1;

    // convert to real register value
    u8Divider = u32PWMDividerToRegTbl[u8Divider];

    // every two channels share a prescaler
    (pwm)->CLKPSC = ((pwm)->CLKPSC & (~PWM_CLKPSC_CLKPSC_Msk) ) | u16Prescale ;
    (pwm)->CLKDIV = ((pwm)->CLKDIV & ~(PWM_CLKDIV_CLKDIV_Msk) ) | u8Divider ;
    
    (pwm)->CTL |= PWM_CTL_CNTMODE_Msk ;
    (pwm)->PERIOD = u16CNR;

    return (u32NearestUnitTimeNsec);
}

/**
 * @brief Configure PWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] pwm: The base address of PWM module
 * @param[in] u32Frequency: Target generator frequency
 * @param[in] u32DutyCycle0: Target generator duty cycle percentage of channel 0. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @param[in] u32DutyCycle1: Target generator duty cycle percentage of channel 1. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @param[in] u32DutyCycle2: Target generator duty cycle percentage of channel 2. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @param[in] u32DutyCycle3: Target generator duty cycle percentage of channel 3. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @details This function is used to configure PWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @note Since every two channels, (0 & 1), (2 & 3), shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 //uint32_t u32ChannelNum,
                                 uint32_t u32Frequency,
                                 uint32_t u32DutyCycle0, uint32_t u32DutyCycle1, uint32_t u32DutyCycle2, uint32_t u32DutyCycle3)
{
    uint32_t u32Src = 0;
    uint32_t u32PWMClockSrc = 0;
    uint32_t i;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    /* this table is mapping divider value to register configuration */
    uint32_t u32PWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
    uint16_t u16CNR = 0xFFFF;

    if(pwm == PWM0)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_PWM0SEL_Msk) >> CLK_CLKSEL1_PWM0SEL_Pos;
    if(pwm == PWM1)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_PWM1SEL_Msk) >> CLK_CLKSEL1_PWM1SEL_Pos;

    switch (u32Src)
    {
				case 0:
						u32PWMClockSrc = CLK_GetHCLKFreq();
						break;
				case 1:
						u32PWMClockSrc = __LXT;
						break;
				case 2:
						u32PWMClockSrc = __LIRC;
						break;
				case 3:
						u32PWMClockSrc = __HIRC;
						break;
    }

    for(; u8Divider < 17; u8Divider <<= 1)    // clk divider could only be 1, 2, 4, 8, 16
    {
        i = (u32PWMClockSrc / u32Frequency) / u8Divider;
        // If target value is larger than CNR * prescale, need to use a larger divider
        if(i > (0x10000 * 0x100))
            continue;

        // CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF
        u8Prescale = (i + 0xFFFF) / 0x10000;

        // u8Prescale must at least be 2, otherwise the output stop
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000)
        {
            if(i == 1)
                u16CNR = 1;     // Too fast, and PWM cannot generate expected frequency...
            else
                u16CNR = i;
            break;
        }
    }
    // Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register
    i = u32PWMClockSrc / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR -= 1;
    // convert to real register value
    u8Divider = u32PWMDividerToRegTbl[u8Divider];

    // every two channels share a prescaler
    (pwm)->CLKPSC = ((pwm)->CLKPSC & (~PWM_CLKPSC_CLKPSC_Msk) ) | u8Prescale ;
    (pwm)->CLKDIV = ((pwm)->CLKDIV & (~PWM_CLKDIV_CLKDIV_Msk) ) | u8Divider ;

    (pwm)->CTL |= PWM_CTL_CNTMODE_Msk ;
    (pwm)->PERIOD = u16CNR;
		
		(pwm)->CMPDAT0 = u32DutyCycle0 * (u16CNR + 1) / 100 - 1;
		(pwm)->CMPDAT1 = u32DutyCycle1 * (u16CNR + 1) / 100 - 1;
		(pwm)->CMPDAT2 = u32DutyCycle2 * (u16CNR + 1) / 100 - 1;
		(pwm)->CMPDAT3 = u32DutyCycle3 * (u16CNR + 1) / 100 - 1;

    return(i);
}

/**
 * @brief Set PWM period cycle
 * @param[in] pwm: The base address of PWM module
 * @param[in] u16CNR: PWM period cycle
 * @return None
 * @details This function is used to set PWM period cycle
 */
void PWM_SetCNR(PWM_T *pwm,uint16_t u16CNR)
{
		if(pwm == PWM0 || pwm == PWM1)
				(pwm)->PERIOD = u16CNR;
}	

/**
 * @brief Set PWM duty cycle
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~3
 * @param[in] u16CMR: PWM duty cycle
 * @return None
 * @details This function is used to set PWM duty cycle
 */
void PWM_SetCMR(PWM_T *pwm,uint32_t u32ChannelNum,uint16_t u16CMR)
{
		if(pwm == PWM0 || pwm == PWM1)
		{
				switch (u32ChannelNum)
				{
						case 0:
								(pwm)->CMPDAT0 = u16CMR;
								break;
						case 1:
								(pwm)->CMPDAT1 = u16CMR;
								break;
						case 2:
								(pwm)->CMPDAT2 = u16CMR;
								break;
						case 3:
								(pwm)->CMPDAT3 = u16CMR;
								break;
				}
		}
}	

/**
 * @brief Start PWM module
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to start PWM module.
 */
void PWM_Start(PWM_T *pwm)
{
    (pwm)->CTL |= PWM_CTL_CNTEN_Msk;
}

/**
 * @brief Stop PWM module
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to stop PWM module.
 */
void PWM_Stop(PWM_T *pwm)
{
    (pwm)->PERIOD = 0;
		(pwm)->CTL &= (~PWM_CTL_CNTEN_Msk);
}

/**
 * @brief Enable capture of selected channel(s)
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to enable capture of selected channel(s).
 */
void PWM_EnableCapture(PWM_T *pwm)
{
  (pwm)->CAPCTL |= PWM_CAPCTL_CAPEN_Msk;

	(pwm)->PCEN |= PWM_PCEN_CAPINEN_Msk;
}

/**
 * @brief Disable capture of selected channel(s)
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to disable capture of selected channel(s).
 */
void PWM_DisableCapture(PWM_T *pwm)
{
  (pwm)->CAPCTL &= ~(PWM_CAPCTL_CAPEN_Msk) ;

	(pwm)->PCEN &= (~PWM_PCEN_CAPINEN_Msk) ;
}

/**
 * @brief Enables PWM output generation of selected channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Channel: 0~3
 * @return None
 * @details This function is used to enables PWM output generation of selected channel(s).
 */
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32Channel)
{
		switch (u32Channel)
		{
				case 0:
						(pwm)->PCEN |= PWM_PCEN_POEN0_Msk;
						break;
				case 1:
						(pwm)->PCEN |= PWM_PCEN_POEN1_Msk;
						break;
				case 2:
						(pwm)->PCEN |= PWM_PCEN_POEN2_Msk;
						break;
				case 3:
						(pwm)->PCEN |= PWM_PCEN_POEN3_Msk;
						break;
		}
}

/**
 * @brief Disables PWM output generation of selected channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Channel: 0~3
 * @return None
 * @details This function is used to disables PWM output generation of selected channel(s).
 */
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32Channel)
{
		switch (u32Channel)
		{
				case 0:
						(pwm)->PCEN &= (~PWM_PCEN_POEN0_Msk);
						break;
				case 1:
						(pwm)->PCEN &= (~PWM_PCEN_POEN1_Msk);
						break;
				case 2:
						(pwm)->PCEN &= (~PWM_PCEN_POEN2_Msk);
						break;
				case 3:
						(pwm)->PCEN &= (~PWM_PCEN_POEN3_Msk);
						break;
		}
}

/**
 * @brief Enable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~3
 * @param[in] u32Duration Dead Zone length in PWM clock count, valid values are between 0~0xFF, but 0 means there is no
 *                        dead zone.
 * @return None
 * @details This function is used to enable Dead zone of selected channel.
 */
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // set duration
    (pwm)->CLKPSC = ((pwm)->CLKPSC & ~(PWM_CLKPSC_DZI0_Msk << (8*u32ChannelNum))) | (u32Duration << (PWM_CLKPSC_DZI0_Pos + 8*u32ChannelNum));
    // enable dead zone
    (pwm)->CTL |= (PWM_CTL_DTEN0_Msk << u32ChannelNum);
}

/**
 * @brief Disable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~3
 * @return None
 * @details This function is used to disable Dead zone of selected channel.
 */
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // disable dead zone
    (pwm)->CTL &= ~(PWM_CTL_DTEN0_Msk << u32ChannelNum);
}

/**
 * @brief Enable capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to enable capture interrupt of selected channel.
 */
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32Edge)
{
    (pwm)->CAPCTL |= u32Edge ;

}

/**
 * @brief Disable capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to disable capture interrupt of selected channel.
 */
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32Edge)
{
    (pwm)->CAPCTL &= (~u32Edge) ;
}

/**
 * @brief Clear capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to clear capture interrupt of selected channel.
 */
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32Edge)
{
    //clear capture interrupt flag, and clear CRLR or CFLR latched indicator
    (pwm)->CAPCTL = ((pwm)->CAPCTL & PWM_CCR_MASK) | PWM_CAPCTL_CAPIF_Msk | u32Edge ;
}

/**
 * @brief Get capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 * @retval 0 No capture interrupt
 * @retval 1 Rising edge latch interrupt
 * @retval 2 Falling edge latch interrupt
 * @retval 3 Rising and falling latch interrupt
 * @details This function is used to get capture interrupt of selected channel.
 */
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm)
{
    return (((pwm)->CAPCTL & (PWM_CAPCTL_CRLIF_Msk | PWM_CAPCTL_CFLIF_Msk)) >> PWM_CAPCTL_CRLIF_Pos);
}

/**
 * @brief Enable interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to enable interrupt of selected channel.
 */
void PWM_EnableInt(PWM_T *pwm)
{
    (pwm)->INTEN |= PWM_INTEN_PIEN_Msk ;
}

/**
 * @brief Disable interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to disable interrupt of selected channel.
 */
void PWM_DisableInt(PWM_T *pwm)
{
    (pwm)->INTEN &= (~PWM_INTEN_PIEN_Msk);
}

/**
 * @brief Clear interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This function is used to clear interrupt flag of selected channel.
 */
void PWM_ClearIntFlag(PWM_T *pwm)
{
    (pwm)->INTSTS |= PWM_INTSTS_PIF_Msk ;
}

/**
 * @brief Get interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @retval 0 interrupt did not occur
 * @retval 1 interrupt occurred
 * @details This function is used to get interrupt flag of selected channel.
 */
uint32_t PWM_GetIntFlag(PWM_T *pwm)
{
    return ((pwm)->INTSTS & PWM_INTSTS_PIF_Msk);
}



/*@}*/ /* end of group ISD9000_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_PWM_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
