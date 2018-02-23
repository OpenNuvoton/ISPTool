/**************************************************************************//**
 * @file     pwm.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/10 2:52p $
 * @brief    I91200 series PWM driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_PWM_Driver PWM Driver
  @{
*/


/** @addtogroup I91200_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief Configure PWM capture and get the nearest unit time.
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~3
 * @param[in] u32UnitTimeNsec The unit time of counter
 * @param[in] u32CaptureEdge The condition to latch the counter. This parameter is not used
 * @return The nearest unit time in nano second.
 * @details This function is used to configure PWM capture and get the nearest unit time.
 */
uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge)
{
 	uint32_t u32Src = 0;
	uint32_t u32PWMClockSrc;
	uint32_t u32PWMClkTbl[4] = {__LIRC, __LXT, 0, __HIRC_48M};
	uint32_t u32NearestUnitTimeNsec;
	uint8_t  u8Divider = 1;
	/* this table is mapping divider value to register configuration */
	uint32_t u32PWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
	uint16_t u16Prescale = 2;
	//uint16_t u16CNR = 0xFFFF;

	if(u32ChannelNum < 2)
		u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_PWM0CH01SEL_Msk) >> CLK_CLKSEL1_PWM0CH01SEL_Pos;
	else
		u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_PWM0CH23SEL_Msk) >> CLK_CLKSEL1_PWM0CH23SEL_Pos;
	
	if(u32Src == 2)
	{
		SystemCoreClockUpdate();
		u32PWMClockSrc = CLK_GetHCLKFreq();
	}
	else
	{
		u32PWMClockSrc = u32PWMClkTbl[u32Src];
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
	PWM_SET_PRESCALER(pwm, u32ChannelNum, u16Prescale);
	PWM_SET_DIVIDER(pwm, u32ChannelNum, u8Divider);
	// set PWM to edge aligned type
	//(pwm)->CTL &= ~(PWM_PCR_PWM01TYPE_Msk << (u32ChannelNum >> 1));
	(pwm)->CTL |= PWM_CTL_CNTMODE0_Msk << (8 * u32ChannelNum);
	PWM_SET_CNR(pwm, u32ChannelNum, 0xFFFF);

	return (u32NearestUnitTimeNsec);
}

/**
 * @brief Configure PWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~3
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @details This function is used to configure PWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @note Since every two channels, (0 & 1), (2 & 3), shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequency,
                                 uint32_t u32DutyCycle)
{
	uint32_t u32Src = 0;
	uint32_t u32PWMClockSrc;
	uint32_t u32PWMClkTbl[4] = {__LIRC, __LXT, 0, __HIRC_48M};
	uint32_t i;
	uint8_t  u8Divider = 1, u8Prescale = 0xFF;
	/* this table is mapping divider value to register configuration */
	uint32_t u32PWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
	uint16_t u16CNR = 0xFFFF;

	if(u32ChannelNum < 2)
		u32Src = (CLK->CLKSEL1 & (CLK_CLKSEL1_PWM0CH01SEL_Msk)) >> CLK_CLKSEL1_PWM0CH01SEL_Pos;
	else
		u32Src = (CLK->CLKSEL1 & (CLK_CLKSEL1_PWM0CH23SEL_Msk)) >> CLK_CLKSEL1_PWM0CH23SEL_Pos;
	
	if(u32Src == 2)
	{
		SystemCoreClockUpdate();
		u32PWMClockSrc = CLK_GetHCLKFreq();
	}
	else
	{
		u32PWMClockSrc = u32PWMClkTbl[u32Src];
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
	PWM_SET_PRESCALER(pwm, u32ChannelNum, u8Prescale);
	PWM_SET_DIVIDER(pwm, u32ChannelNum, u8Divider);
	// set PWM to edge aligned type
	//(pwm)->CTL &= ~(PWM_PCR_PWM01TYPE_Msk << (u32ChannelNum >> 1));
	(pwm)->CTL |= PWM_CTL_CNTMODE0_Msk << (8 * u32ChannelNum);
	PWM_SET_CMR(pwm, u32ChannelNum, u32DutyCycle * (u16CNR + 1) / 100 - 1 );
	PWM_SET_CNR(pwm, u32ChannelNum, u16CNR);
    return(i);
}


/**
 * @brief Start PWM module
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to start PWM module.
 */
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask)
{
	uint32_t u32Mask = 0, i;
	for(i = 0; i < PWM_CHANNEL_NUM+1; i ++)
	{
		if(u32ChannelMask & (1 << i))
		{
			u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 8));
		}
	}

	(pwm)->CTL |= u32Mask;
}

/**
 * @brief Stop PWM module
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop PWM module.
 */
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask)
{
	uint32_t i;
	for(i = 0; i < PWM_CHANNEL_NUM; i ++)
	{
		if(u32ChannelMask & (1 << i))
		{
			*((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + i * 12))) = 0;
		}
	}
}

/**
 * @brief Stop PWM generation immediately by clear channel enable bit
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop PWM generation immediately by clear channel enable bit.
 */
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask)
{
	uint32_t u32Mask = 0, i;
	for(i = 0; i < PWM_CHANNEL_NUM; i ++)
	{
		if(u32ChannelMask & (1 << i))
		{
			u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 8));
		}
	}

	(pwm)->CTL &= ~u32Mask;
}

/**
 * @brief Enable capture of selected channel(s)
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to enable capture of selected channel(s).
 */
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask)
{
    /*uint32_t i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            if(i < 2)
            {
                (pwm)->CAPCTL01 |= PWM_CAPCTL01_CAPEN0_Msk << (i * 16);
            }
        }
    }
	(pwm)->CAPINEN |= u32ChannelMask;
	*/
	if ( (u32ChannelMask==BIT0) || (u32ChannelMask==BIT1) )
	{
		(pwm)->CAPCTL01 |= PWM_CAPCTL01_CAPEN0_Msk << ((u32ChannelMask - BIT0) * 16);
	}else if ( (u32ChannelMask==BIT2) || (u32ChannelMask==BIT3) )
	{
		(pwm)->CAPCTL23 |= PWM_CAPCTL23_CAPEN2_Msk << ((u32ChannelMask - BIT2) * 4);
	}
	
    (pwm)->CAPINEN |= u32ChannelMask;
}

/**
 * @brief Disable capture of selected channel(s)
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to disable capture of selected channel(s).
 */
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask)
{
    /*uint32_t i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            if(i < 2)
            {
                (pwm)->CAPCTL01 &= ~(PWM_CAPCTL01_CAPEN0_Msk << (i * 16));
            }
        }
    }
    (pwm)->CAPINEN &= ~u32ChannelMask;
	*/
	if ( (u32ChannelMask==BIT0) || (u32ChannelMask==BIT1) )
	{
		(pwm)->CAPCTL01 &= ~(PWM_CAPCTL01_CAPEN0_Msk << ((u32ChannelMask - BIT0) * 16));
	}else if ( (u32ChannelMask==BIT2) || (u32ChannelMask==BIT3) )
	{
		(pwm)->CAPCTL23 &= ~(PWM_CAPCTL23_CAPEN2_Msk << ((u32ChannelMask - BIT2) * 4));
	}
	
	(pwm)->CAPINEN &= ~u32ChannelMask;
	
}

/**
 * @brief Enables PWM output generation of selected channel(s)
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 * @details This function is used to enables PWM output generation of selected channel(s).
 */
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask)
{
	(pwm)->POEN |= u32ChannelMask;
}

/**
 * @brief Disables PWM output generation of selected channel(s)
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 * @details This function is used to disables PWM output generation of selected channel(s).
 */
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask)
{
	(pwm)->POEN &= ~u32ChannelMask;
}

/**
 * @brief Enable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
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
	(pwm)->CLKPSC = ((pwm)->CLKPSC & ~(PWM_CLKPSC_DTCNT01_Msk << (8 * u32ChannelNum))) | (u32Duration << (PWM_CLKPSC_DTCNT01_Pos + 8 * u32ChannelNum));
	// enable dead zone
	(pwm)->CTL |= (PWM_CTL_DTEN01_Msk << u32ChannelNum);
}

/**
 * @brief Disable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to disable Dead zone of selected channel.
 */
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum)
{
	// every two channels shares the same setting
	u32ChannelNum >>= 1;
	// enable dead zone
	(pwm)->CTL &= ~(PWM_CTL_DTEN01_Msk << u32ChannelNum);
}

/**
 * @brief Enable capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to enable capture interrupt of selected channel.
 */
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
	//if(u32ChannelNum < 2)
	//    (pwm)->CAPCTL01 |= u32Edge << (u32ChannelNum * 16);
	*((__IO uint32_t *)((((uint32_t) & ((pwm)->CAPCTL01)) + (u32ChannelNum>>1) * 4))) |= u32Edge << ((u32ChannelNum&0x1) * 16);
}

/**
 * @brief Disable capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to disable capture interrupt of selected channel.
 */
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
	// if(u32ChannelNum < 2)
	//    (pwm)->CAPCTL01 &= u32Edge << ~(u32ChannelNum * 16);
	*((__IO uint32_t *)((((uint32_t) & ((pwm)->CAPCTL01)) + (u32ChannelNum>>1) * 4))) &= ~(u32Edge << ((u32ChannelNum&0x1) * 16));
}

/**
 * @brief Clear capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref PWM_CAPTURE_INT_RISING_LATCH
 *              - \ref PWM_CAPTURE_INT_FALLING_LATCH
 * @return None
 * @details This function is used to clear capture interrupt of selected channel.
 */
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
	//clear capture interrupt flag, and clear CRLR or CFLR latched indicator
	//if(u32ChannelNum < 2)
	//    (pwm)->CAPCTL01 = ((pwm)->CAPCTL01 & PWM_CCR_MASK) | (PWM_CAPCTL01_CAPIF0_Msk << (u32ChannelNum * 16)) | (u32Edge << (u32ChannelNum * 16 + 5));
	*((__IO uint32_t *)((((uint32_t) & ((pwm)->CAPCTL01)) + (u32ChannelNum>>1) * 4))) = (*((__IO uint32_t *)((((uint32_t) & ((pwm)->CAPCTL01)) + (u32ChannelNum>>1) * 4))) & PWM_CCR_MASK) | (PWM_CAPCTL01_CAPIF0_Msk << ((u32ChannelNum &0x1)* 16)) | (u32Edge << ((u32ChannelNum&0x1) * 16 + 5));

	//*((__IO uint32_t *)((((uint32_t) & ((pwm)->CAPCTL01)) + (u32ChannelNum>>1) * 4))) |= u32Edge << ((u32ChannelNum&0x1) * 16);
}

/**
 * @brief Get capture interrupt of selected channel.
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @retval 0 No capture interrupt
 * @retval 1 Rising edge latch interrupt
 * @retval 2 Falling edge latch interrupt
 * @retval 3 Rising and falling latch interrupt
 * @details This function is used to get capture interrupt of selected channel.
 */
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
	/*if(u32ChannelNum < 2)
	{
			return (((pwm)->CAPCTL01 & ((PWM_CAPCTL01_CRLIF0_Msk | PWM_CAPCTL01_CFLIF0_Msk) << (u32ChannelNum * 16))) >> (PWM_CAPCTL01_CRLIF0_Pos + u32ChannelNum * 16));
	}
	return 0;
	*/
	return ((*((__IO uint32_t *)((((uint32_t) & ((pwm)->CAPCTL01)) + (u32ChannelNum>>1) * 4))))&(0xD0 << ((u32ChannelNum&0x1) * 16)));
}
/**
 * @brief Enable interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32IntType This parameter is not used
 * @return None
 * @details This function is used to enable interrupt of selected channel.
 *          Every two channels, (0 & 1), (2 & 3), shares the interrupt type setting.
 */
void PWM_EnableInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntType)
{
	(pwm)->INTEN |= (PWM_INTEN_PIEN0_Msk << u32ChannelNum);
}

/**
 * @brief Disable interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to disable interrupt of selected channel.
 */
void PWM_DisableInt(PWM_T *pwm, uint32_t u32ChannelNum)
{
	(pwm)->INTEN &= ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum);
}

/**
 * @brief Clear interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to clear interrupt flag of selected channel.
 */
void PWM_ClearIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
	(pwm)->INTSTS = PWM_INTSTS_PIF0_Msk << u32ChannelNum;
}

/**
 * @brief Get interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @retval 0 interrupt did not occur
 * @retval 1 interrupt occurred
 * @details This function is used to get interrupt flag of selected channel.
 */
uint32_t PWM_GetIntFlag(PWM_T *pwm, uint32_t u32ChannelNum)
{
	return (((pwm)->INTSTS & (PWM_INTSTS_PIF0_Msk << u32ChannelNum)) ? 1 : 0);
}



/*@}*/ /* end of group I91200_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_PWM_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
