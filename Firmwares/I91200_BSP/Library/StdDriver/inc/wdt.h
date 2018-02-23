/**************************************************************************//**
 * @file     wdt.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 16/08/01 5:00p $
 * @brief    I91200 WDT driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __WDT_H__
#define __WDT_H__

#ifdef  __cplusplus
extern "C"
{
#endif


/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_WDT_Driver WDT Driver
  @{
*/


/** @addtogroup I91200_WDT_EXPORTED_CONSTANTS WDT Exported Constants
  @{
*/
	
#define WDT_TIMEOUT_2POW4           (0UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^4 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW6           (1UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^6 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW8           (2UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^8 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW10          (3UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^10 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW12          (4UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^12 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW14          (5UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^14 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW16          (6UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^16 * WDT clocks  \hideinitializer */
#define WDT_TIMEOUT_2POW18          (7UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^18 * WDT clocks  \hideinitializer */

/*@}*/ /* end of group I91200_WDT_EXPORTED_CONSTANTS */


/** @addtogroup I91200_WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
  * @brief This macro clear WDT time-out reset system flag.
  * @return None
  * \hideinitializer
  */
#define WDT_CLEAR_RESET_FLAG()			WDT_ClearResetFlag()

/**
  * @brief This macro clear WDT time-out interrupt flag.
  * @return None
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_INT_FLAG() 	WDT_ClearTimeOutIntFlag()

/**
  * @brief This macro indicate WDT time-out to reset system or not.
  * @return WDT reset system or not
  * @retval 0 WDT did not cause system reset
  * @retval 1 WDT caused system reset
  * \hideinitializer
  */
#define WDT_GET_RESET_FLAG() 			(WDT->CTL & WDT_CTL_RSTF_Msk ? 1 : 0)

/**
  * @brief This macro indicate WDT time-out interrupt occurred or not.
  * @return WDT time-out interrupt occurred or not
  * @retval 0 WDT time-out interrupt did not occur
  * @retval 1 WDT time-out interrupt occurred
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_INT_FLAG() 		(WDT->CTL & WDT_CTL_IF_Msk ? 1 : 0)

/**
  * @brief This macro is used to reset 18-bit WDT counter.
  * @details If WDT is activated and enabled to reset system, software must reset WDT counter
  *  before WDT time-out plus reset delay reached. Or WDT generate a reset signal.
  * \hideinitializer
  */
#define WDT_RESET_COUNTER()				WDT_ResetCounter()

void WDT_Open( uint32_t u32TimeoutInterval, uint32_t u32EnableReset );
void WDT_Close(void);
void WDT_EnableInt(void);
void WDT_DisableInt(void);
void WDT_ClearResetFlag(void);
void WDT_ClearTimeOutIntFlag(void);
void WDT_ResetCounter(void);

/*@}*/ /* end of group I91200_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_WDT_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__WDT_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
