#include <stdio.h>
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @brief      Open Timer with Operate Mode and Frequency
  *
  * @param[in]  timer       The pointer of the specified Timer module. It could be TIMER0, TIMER1, TIMER2, TIMER3.
  * @param[in]  u32Mode     Operation mode. Possible options are
  *                         - \ref TIMER_ONESHOT_MODE
  *                         - \ref TIMER_PERIODIC_MODE
  *                         - \ref TIMER_TOGGLE_MODE
  *                         - \ref TIMER_CONTINUOUS_MODE
  * @param[in]  u32Freq     Target working frequency
  *
  * @return     Real timer working frequency
  *
  * @details    This API is used to configure timer to operate in specified mode and frequency.
  *             If timer cannot work in target frequency, a closest frequency will be chose and returned.
  * @note       After calling this API, Timer is \b NOT running yet. But could start timer running be calling
  *             \ref TIMER_Start macro or program registers directly.
  */
__weak uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = __HXT; // TIMER_GetModuleClock(timer);
    uint32_t u32Cmpr = 0UL, u32Prescale = 0UL;

    /* Fastest possible timer working freq is (u32Clk / 2). While cmpr = 2, prescaler = 0. */
    if (u32Freq > (u32Clk / 2UL)) {
        u32Cmpr = 2UL;
    } else {
        u32Cmpr = u32Clk / u32Freq;
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */

        if (u32Prescale > 0UL) {
            u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
        }
    }

    timer->CTL = u32Mode | u32Prescale;
    timer->CMP = u32Cmpr;
    return (u32Clk / (u32Cmpr * (u32Prescale + 1UL)));
}

void TIMER3_Test(void)
{
    static uint32_t sec = 1;
    printf("\nThis sample code use timer3 to generate interrupt every 1 second \n");

    while (1) {
        if (TIMER3->INTSTS & TIMER_INTSTS_TIF_Msk) {
            // clear timer interrupt flag
            TIMER_ClearIntFlag(TIMER3);
            printf("%d sec\n", sec++);
        }
    }
}

void TIMER3_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR3CKEN_Msk;
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR3SEL_Msk)) | CLK_CLKSEL1_TMR3SEL_HXT;
    // printf("\nThis sample code use timer3 to generate interrupt every 1 second \n");
    // Set timer frequency to 1HZ
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER3);
    // NVIC_EnableIRQ(TMR3_IRQn);
    // Start Timer 0
    TIMER_Start(TIMER3);
    // TIMER3_Test();
}

#ifdef __cplusplus
}
#endif
