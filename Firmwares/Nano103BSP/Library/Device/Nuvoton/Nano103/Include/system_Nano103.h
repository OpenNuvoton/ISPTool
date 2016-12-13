/**************************************************************************//**
 * @file     system_Nano103.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/12/03 5:44p $
 * @brief    Nano103 system clock definition file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __SYSTEM_NANO103_H__
#define __SYSTEM_NANO103_H__

#ifdef __cplusplus
extern "C" {
#endif


/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/

#define __HXT         (12000000UL)      /*!< High Speed External Crystal Clock Frequency 12MHz */
#define __LXT         (32768UL)         /*!< Low Speed External Crystal Clock Frequency 32.768kHz */
#define __HIRC12M     (12000000UL)      /*!< High Speed Inernal Crystal 0 Clock Frequency 12MHz */
#define __HIRC16M     (16000000UL)      /*!< High Speed Inernal Crystal 0 Clock Frequency 16MHz */
#define __LIRC        (10000UL)         /*!< Low Speed Internal 10kHz RC Oscillator Frequency */
#define __HIRC36M     (36000000UL)      /*!< High Speed Inernal Crystal 1 Clock Frequency 36MHz */
#define __MIRC        (4000000UL)       /*!< Medium Speed Inernal Crystal Clock Frequency 4MHz */
#define __HIRC         __HIRC12M        /* HIRC0 at 12MHz as default HIRC*/
#define __HSI         (__HIRC12M)       /* Factory Default is internal 12MHz */


extern uint32_t SystemCoreClock;        /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;            /*!< Cycles per micro second */

/**
 * Update SystemCoreClock variable
 *
 * @param  None
 * @return None
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */

extern void SystemCoreClockUpdate (void);
extern uint32_t SysGet_PLLClockFreq(void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_NANO103_H__


/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
