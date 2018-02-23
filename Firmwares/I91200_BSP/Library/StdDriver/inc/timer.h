/**************************************************************************//**
 * @file     timer.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/12/26 11:00a $
 * @brief    I91200 TIMER driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup I91200_TIMER_EXPORTED_CONSTANTS TIMER Exported Constants
  @{
*/

#define TIMER_ONESHOT_MODE                  (0UL)                          /*!< Timer working in one shot mode  \hideinitializer */
#define TIMER_PERIODIC_MODE                 (1UL << TMR_CTL_OPMODE_Pos)    /*!< Timer working in periodic mode  \hideinitializer */
#define TIMER_CONTINUOUS_MODE               (3UL << TMR_CTL_OPMODE_Pos)    /*!< Timer working in continuous mode  \hideinitializer */

/*@}*/ /* end of group I91200_TIMER_EXPORTED_CONSTANTS */


/** @addtogroup I91200_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This macro is used to set new Timer compared value
  * @param[in] timer The base address of Timer module
  * @param[in] u32Value  Timer compare value. Valid values are between 2 to 0xFFFFFF
  * @return None
  * \hideinitializer
  */
#define TIMER_SET_CMP_VALUE(timer, u32Value) ((timer)->CMP = (u32Value))

/**
  * @brief This macro is used to set new Timer prescale value
  * @param[in] timer The base address of Timer module
  * @param[in] u32Value  Timer prescale value. Valid values are between 0 to 0xFF
  * @return None
  * @note Clock input is divided by (prescale + 1) before it is fed into timer
  * \hideinitializer
  */
#define TIMER_SET_PRESCALE_VALUE(timer, u32Value) ((timer)->CTL = ((timer)->CTL & ~TMR_CTL_PSC_Msk) | (u32Value))

/**
  * @brief This macro is used to check if specify Timer is inactive or active
  * @return timer is activate or inactivate
  * @retval 0 Timer 24-bit up counter is inactive
  * @retval 1 Timer 24-bit up counter is active
  * \hideinitializer
  */
#define TIMER_IS_ACTIVE(timer) ((timer)->CTL & TMR_CTL_ACTSTS_Msk ? 1 : 0)

/**
  * @brief This function is used to start Timer counting
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Start(TMR_T *timer)
{
    timer->CTL |= TMR_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to stop Timer counting
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Stop(TMR_T *timer)
{
    timer->CTL &= ~TMR_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to enable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableInt(TMR_T *timer)
{
    timer->CTL |= TMR_CTL_INTEN_Msk;
}

/**
  * @brief This function is used to disable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableInt(TMR_T *timer)
{
    timer->CTL &= ~TMR_CTL_INTEN_Msk;
}

/**
  * @brief This function indicates Timer time-out interrupt occurred or not.
  * @param[in] timer The base address of Timer module
  * @return Timer time-out interrupt occurred or not
  * @retval 0 Timer time-out interrupt did not occur
  * @retval 1 Timer time-out interrupt occurred
  */
__STATIC_INLINE uint32_t TIMER_GetIntFlag(TMR_T *timer)
{
    return(timer->INTSTS & TMR_INTSTS_TIF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer time-out interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearIntFlag(TMR_T *timer)
{
    timer->INTSTS = TMR_INTSTS_TIF_Msk;
}
/**
  * @brief This function reports the current timer counter value.
  * @param[in] timer The base address of Timer module
  * @return Timer counter value
  */
__STATIC_INLINE uint32_t TIMER_GetCounter(TMR_T *timer)
{
    return timer->CNT;
}

uint32_t TIMER_Open(TMR_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void     TIMER_Close(TMR_T *timer);
void     TIMER_Delay(TMR_T *timer, uint32_t u32Usec);
uint32_t TIMER_GetModuleClock(TMR_T *timer);
uint32_t TIMER_GetWorkingFreq(TMR_T *timer);


/*@}*/ /* end of group I91200_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_TIMER_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__TIMER_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
