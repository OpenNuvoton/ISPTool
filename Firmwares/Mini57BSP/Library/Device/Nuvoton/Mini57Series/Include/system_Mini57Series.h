/**************************************************************************//**
 * @file     system_Mini57Series.h
 * @version  V3.0
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series CMSIS System Header File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SYSTEM_MINI57_H
#define __SYSTEM_MINI57_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/* Using UART0 or UART1 */
#define DEBUG_PORT   UUART0
//#define DEBUG_PORT   UUART1

/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/
#define __HXT       (12000000UL)    /*!< External Crystal Clock Frequency: 4MHz~24MHz depend on crystal module */
#define __LXT       (   32768UL)    /*!< External Crystal Clock Frequency: 32.768KHz */
#define __HXT_LXT   (     __HXT)    /*!< External Crystal Clock Frequency, could be HXT or LXT depend on crystal module */
#define __HIRC      (48000000UL)    /*!< Internal 48M RC Oscillator Frequency */
#define __LIRC      (   10000UL)    /*!< Internal 10K RC Oscillator Frequency */

extern uint32_t SystemCoreClock;    /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;        /*!< Cycles per micro second             */

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize GPIO directions and values
 */
extern void SystemInit(void);


/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
