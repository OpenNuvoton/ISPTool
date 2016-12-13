/**************************************************************************//**
 * @file     system_Mini58Series.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/05/26 10:19a $
 * @brief    Mini58 series system clock definition file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/


#ifndef __SYSTEM_MINI58SERIES_H__
#define __SYSTEM_MINI58SERIES_H__

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
#define __IRC10K         (10000UL)
#define __XTAL            __XTAL12M
#define __HXT             __XTAL
#define __HIRC           (22118400UL)    /* Clock output of Internal high speed RC 22.1184M */

extern uint32_t __HSI;
extern uint32_t SystemCoreClock;        /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;            /*!< Cycles per micro second */
extern uint32_t PllClock;               /*!< PLL Output Clock Frequency          */
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
extern void SystemInit (void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_MINI58SERIES_H__


/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
