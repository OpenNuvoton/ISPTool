/**************************************************************************//**
 * @file     timer.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/17 03:30a $
 * @brief    ISD9100 TIMER driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "ISD9100.h"

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup ISD9100_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This API is used to configure timer to operate in specified mode
  *        and frequency. If timer cannot work in target frequency, a closest
  *        frequency will be chose and returned.
  * @param[in] timer The base address of Timer module
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
    uint32_t u32Cmpr = 0, u32Prescale = 0;

    // Fastest possible timer working freq is u32Clk / 2. While cmpr = 2, pre-scale = 0
    if(u32Freq > (u32Clk / 2)) {
        u32Cmpr = 2;
    } else {
        if(u32Clk >= 0x4000000) {
            u32Prescale = 7;    // real prescaler value is 8
            u32Clk >>= 3;
        } else if(u32Clk >= 0x2000000) {
            u32Prescale = 3;    // real prescaler value is 4
            u32Clk >>= 2;
        } else if(u32Clk >= 0x1000000) {
            u32Prescale = 1;    // real prescaler value is 2
            u32Clk >>= 1;
        }

        u32Cmpr = u32Clk / u32Freq;
    }

    timer->CTL = u32Mode | u32Prescale;
    timer->CMP = u32Cmpr;

    return(u32Clk / (u32Cmpr * (u32Prescale + 1)));
}

/**
  * @brief This API stops Timer counting and disable the Timer interrupt function
  * @param[in] timer The base address of Timer module
  * @return None
  */
void TIMER_Close(TMR_T *timer)
{
    timer->CTL = 0;
}

/**
  * @brief This API is used to create a delay loop for u32usec micro seconds
  * @param[in] timer The base address of Timer module
  * @param[in] u32Usec Delay period in micro seconds with 10 usec every step. Valid values are between 10~1000000 (10 micro second ~ 1 second)
  * @return None
  * @note This API overwrites the register setting of the timer used to count the delay time.
  * @note This API use polling mode. So there is no need to enable interrupt for the timer module used to generate delay
  */
void TIMER_Delay(TMR_T *timer, uint32_t u32Usec)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    uint32_t u32Prescale = 0, delay = SystemCoreClock / u32Clk + 1;
    double fCmpr;

    // Clear current timer configuration
    timer->CTL = 0;

    if(u32Clk == 10000) {         // min delay is 100us if timer clock source is LIRC 10k
        u32Usec = ((u32Usec + 99) / 100) * 100;
    } else {    // 10 usec every step
        u32Usec = ((u32Usec + 9) / 10) * 10;
    }

    if(u32Clk >= 0x4000000) {
        u32Prescale = 7;    // real prescaler value is 8
        u32Clk >>= 3;
    } else if(u32Clk >= 0x2000000) {
        u32Prescale = 3;    // real prescaler value is 4
        u32Clk >>= 2;
    } else if(u32Clk >= 0x1000000) {
        u32Prescale = 1;    // real prescaler value is 2
        u32Clk >>= 1;
    }

    // u32Usec * u32Clk might overflow if using uint32_t
    fCmpr = (u32Usec/1000) * (u32Clk/1000);

    timer->CMP = (uint32_t)fCmpr;
    timer->CTL = TMR_CTL_CNTEN_Msk | u32Prescale; // one shot mode

    // When system clock is faster than timer clock, it is possible timer active bit cannot set in time while we check it.
    // And the while loop below return immediately, so put a tiny delay here allowing timer start counting and raise active flag.
    for(; delay > 0; delay--) {
        __NOP();
    }

    while(timer->CTL & TMR_CTL_ACTSTS_Msk);
}

/**
  * @brief This API is used to get the clock frequency of Timer
  * @param[in] timer The base address of Timer module
  * @return Timer clock frequency
  * @note This API cannot return correct clock rate if timer source is external clock input.
  */
uint32_t TIMER_GetModuleClock(TMR_T *timer)
{
    uint32_t u32Src;
    const uint32_t au32Clk[] = {__LIRC, __LXT, 0, 0, __HIRC};

    if(timer == TIMER0)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0SEL_Msk) >> CLK_CLKSEL1_TMR0SEL_Pos;
    else if(timer == TIMER1)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR1SEL_Msk) >> CLK_CLKSEL1_TMR1SEL_Pos;
	else
		return 0;

    if(u32Src == 2) 
        return(SystemCoreClock);

    return(au32Clk[u32Src]);
}
/**
  * @brief This function reports the current working frequency.
  * @param[in] timer The base address of Timer module
  * @return Timer working frequency.
  */
uint32_t TIMER_GetWorkingFreq(TMR_T *timer)
{
    return TIMER_GetModuleClock(timer)/((timer->CMP)*((timer->CTL&TMR_CTL_PSC_Msk)+1));
}

/*@}*/ /* end of group ISD9100_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_TIMER_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
