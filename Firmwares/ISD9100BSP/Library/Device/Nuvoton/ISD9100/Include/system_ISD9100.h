/**************************************************************************//**
 * @file     system_ISD9100.h
 * @brief    CMSIS Cortex-M0 Device System Header File
 *           for CM0 Device Series
 * @version  V1.03
 * @date     17. July 2014
 *
 * @note
 * Copyright (C) 2010 ARM Limited. All rights reserved.
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


#ifndef __SYSTEM_ISD9100_H__
#define __SYSTEM_ISD9100_H__

#ifdef __cplusplus
extern "C" {
#endif


/*----------------------------------------------------------------------------
  Define system clocks sources
 *----------------------------------------------------------------------------*/
#define __HXT         (0UL)         /*!< High Speed External Crystal Clock Frequency */
#define __LXT         (32768UL)     /*!< Low Speed External Crystal Clock Frequency 32.768kHz */
#define __HIRC        (49152000UL)  /*!< High Speed Internal 48MHz RC Oscillator Frequency */
#define __LIRC        (16000UL)     /*!< Low Speed Internal 16kHz RC Oscillator Frequency */
#define __HSI         (__HIRC)      /* Factory Default is internal 48MHz */

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */
extern uint32_t CyclesPerUs;         /*!< Cycles per micro second              */
extern uint32_t gau32HiRCSrcTbl[];   /*!< HIRC(OSC48)Frequency Selection Table */

/**
 * Initialize the system
 * @return none
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 * @return none
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_ISD9100_H__ */
