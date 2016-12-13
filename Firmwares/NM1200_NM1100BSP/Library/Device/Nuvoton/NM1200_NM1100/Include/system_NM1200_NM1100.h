/**************************************************************************//**
 * @file     system_NM1200_NM1100.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/04/07 2:47p $ 
 * @brief    NM1200_NM1100 system clock definition file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/  
 

#ifndef __SYSTEM_NM1200_NM1100_H__
#define __SYSTEM_NM1200_NM1100_H__

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
#define __IRC44M         (44236800UL)    /* Internal high speed RC oscillator will be trimmed to 44.2368MHz */
#define __IRC48M         (48000000UL)    /* Internal high speed RC will be trimmed to 48MHz */
#define __IRC44M_DIV2    (22118400UL)    /* Clock output of Internal high speed RC 44.2368M divided by 2 */
#define __IRC48M_DIV2    (24000000UL)    /* Clock output of Internal high speed RC 48M divided by 2 */
#define __IRC10K         (10000UL)
#define __XTAL            __XTAL12M

extern uint32_t __HSI;
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
extern void SystemInit (void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_NM1200_NM1100_H__


/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
