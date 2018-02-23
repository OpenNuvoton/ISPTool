/**************************************************************************//**
 * @file     system_I91200.h
 * @brief    CMSIS Cortex-M0 Device System Header File
 *           for CM0 Device Series
 * @version  V1.02
 * @date     22. August 2016
 *
 * @note
 * Copyright (C) ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef __SYSTEM_I91200_H__
#define __SYSTEM_I91200_H__

#ifdef __cplusplus
extern "C" {
#endif


/*----------------------------------------------------------------------------
  Define system clocks sources
 *----------------------------------------------------------------------------*/
#define __HXT         (12288000UL)  /*!< High Speed External Crystal Clock Frequency 12MHz*/
#define __LXT         (32768UL)     /*!< Low Speed External Crystal Clock Frequency 32.768kHz */
#define __HIRC_48M    (49152000UL)  /*!< High Speed Internal 48MHz RC Oscillator Frequency */
#define __HIRC_32M    (32768000UL)  /*!< High Speed Internal 32MHz RC Oscillator Frequency */
#define __LIRC        (10240UL)     /*!< Low Speed Internal 10kHz RC Oscillator Frequency */
#define __HSI         (__HIRC_48M)  /* Factory Default is internal 48MHz */

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */
extern uint32_t CyclesPerUs;         /*!< Cycles per micro second              */

/**
 * Initialize the system
 * @return none
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit (void);

/**
 * Update SystemCoreClock variable
 * @return none
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_I91200_H__ */
