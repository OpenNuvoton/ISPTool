/**************************************************************************//**
 * @file     timer.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/11/02 03:40p $
 * @brief    ISD9000 TIMER driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "ISD9000.h"
#include <stdlib.h>

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup ISD9000_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This API is used to configure timer to operate in specified mode
  *        and frequency. If timer cannot work in target frequency, a closest
  *        frequency will be chose and returned.
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @param[in] u32Mode Operation mode. Possible options are
  *                 - \ref TIMER_ONESHOT_MODE
  *                 - \ref TIMER_PERIODIC_MODE
  *                 - \ref TIMER_CONTINUOUS_MODE
  * @param[in] u32Freq Target working frequency
  * @return Real Timer working frequency
  * @note After calling this API, Timer is \b NOT running yet. But could start timer running be calling
  *       \ref TIMER_Start macro or program registers directly
  */
uint32_t TIMER_Open(TMR_T *timer, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    int32_t i32Cmpr = 2, i32Prescale = 0;
    uint32_t i, u32Diff=0xffffffff;
    int32_t i32Tmp, i32CmprTmp;
	
    for (i=1;i<=256;i++)
    {
        i32CmprTmp = u32Clk / u32Freq / i;
        if (i32CmprTmp>1 && i32CmprTmp<65536)
        {
            i32Tmp = u32Freq * i32CmprTmp * i - u32Clk;
            
			if (i32Tmp==0)
            {
                i32Prescale = i-1;
                i32Cmpr = i32CmprTmp;
                break;
            }
            if (abs(i32Prescale)<u32Diff)
            {
                u32Diff = abs(i32Prescale);
                i32Prescale = i-1;
                i32Cmpr = i32CmprTmp;
            }
        }
    }
	
    timer->CTL = u32Mode | i32Prescale;
    timer->CMP = i32Cmpr;

    return(u32Clk / (i32Cmpr * (i32Prescale + 1)));
}

/**
  * @brief This API stops Timer counting and disable the Timer interrupt function
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @return None
  */
void TIMER_Close(TMR_T *timer)
{
    timer->CTL = 0;
}

/**
  * @brief This API is used to create a delay loop for u32usec micro seconds
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @param[in] u32Usec Delay period in micro seconds with 10 usec every step. Valid values are between 10~349520 (10 micro second ~ 349.52 millisecond)
  * @return None
  * @note This API overwrites the register setting of the timer used to count the delay time.
  * @note This API use polling mode. So there is no need to enable interrupt for the timer module used to generate delay
  */
void TIMER_Delay(TMR_T *timer, uint32_t u32Usec)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    int32_t i32Cmpr = 2, i32Prescale = 0;
    uint32_t i;
    int32_t i32CmprTmp;
    uint32_t delay = SystemCoreClock / u32Clk + 1;

    // Clear current timer configuration
    timer->CTL = 0;
	
	if (u32Usec>349520) u32Usec=349520;
    if (u32Usec<10) u32Usec=10;
	
    for (i=1;i<=256;i++)
    {
        i32CmprTmp = (u32Clk/10000)*u32Usec / i / 100;
        if (i32CmprTmp>1 && i32CmprTmp<65536)
        {
            i32Prescale = i-1;
            i32Cmpr = i32CmprTmp;
            break;
        }
    }
	
    timer->CMP = i32Cmpr;
    timer->CTL = TMR_CTL_CNTEN_Msk | i32Prescale; // one shot mode

    // When system clock is faster than timer clock, it is possible timer active bit cannot set in time while we check it.
    // And the while loop below return immediately, so put a tiny delay here allowing timer start counting and raise active flag.
    for(; delay > 0; delay--) 
	{
        __NOP();
    }

    while(timer->CTL & TMR_CTL_ACTSTS_Msk);
}

/**
  * @brief This API is used to get the clock frequency of Timer
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @return Timer clock frequency
  * @note This API cannot return correct clock rate if timer source is external clock input.
  */
uint32_t TIMER_GetModuleClock(TMR_T *timer)
{
    uint32_t u32Src = 0;
	
    if (timer==TIMER0)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0SEL_Msk);
    if (timer==TIMER1)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR1SEL_Msk);
    if (timer==TIMER2)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR2SEL_Msk);

    switch (u32Src)
    {
        case CLK_CLKSEL1_TMR0SEL_HCLK:
			return(CLK_GetHCLKFreq());
        
		case CLK_CLKSEL1_TMR0SEL_LXT:
		case CLK_CLKSEL1_TMR1SEL_LXT:
		case CLK_CLKSEL1_TMR2SEL_LXT:
			return(__LXT);
		
		case CLK_CLKSEL1_TMR0SEL_LIRC:
		case CLK_CLKSEL1_TMR1SEL_LIRC:
		case CLK_CLKSEL1_TMR2SEL_LIRC:
			return(__LIRC); 

        case CLK_CLKSEL1_TMR0SEL_HIRC:
		case CLK_CLKSEL1_TMR1SEL_HIRC:
		case CLK_CLKSEL1_TMR2SEL_HIRC:
			return(__HIRC);
    }
    return 0;
}

/*@}*/ /* end of group ISD9000_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_TIMER_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
