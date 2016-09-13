/**************************************************************************//**
 * @file     system_NUC029FAE.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/05/16 10:00a $
 * @brief    NUC029FAE system clock definition file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/


#ifndef __SYSTEM_NUC029FAE_H__
#define __SYSTEM_NUC029FAE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/

#define __XTAL12M        (12000000UL)
#define __XTAL32K        (32768UL)
#define __IRC22M        (22118400UL)
#define __IRC10K        (10000UL)
#define __XTAL            __XTAL12M
#define __HSI            (__IRC22M)      /* Factory Default is internal 22MHz */


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

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_NUC029FAE_H__


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
