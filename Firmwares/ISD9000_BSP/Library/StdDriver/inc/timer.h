/**************************************************************************//**
 * @file     timer.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/11/02 05:00p $
 * @brief    ISD9000 TIMER driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup ISD9000_TIMER_EXPORTED_CONSTANTS TIMER Exported Constants
  @{
*/

#define TIMER_ONESHOT_MODE                  (0UL)                          /*!< Timer working in one shot mode  \hideinitializer */
#define TIMER_PERIODIC_MODE                 (1UL << TMR_CTL_OPMODE_Pos)    /*!< Timer working in periodic mode  \hideinitializer */
#define TIMER_CONTINUOUS_MODE               (3UL << TMR_CTL_OPMODE_Pos)    /*!< Timer working in continuous mode  \hideinitializer */

/*@}*/ /* end of group ISD9000_TIMER_EXPORTED_CONSTANTS */


/** @addtogroup ISD9000_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This macro is used to set new Timer compared value
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @param[in] u32Value  Timer compare value. Valid values are between 2 to 0xFFFF
  * @return None
  * \hideinitializer
  */
#define TIMER_SET_CMP_VALUE(timer, u32Value) ((timer)->CMP = (u32Value))

/**
  * @brief This macro is used to set new Timer prescale value
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @param[in] u32Value  Timer prescale value. Valid values are between 0 to 0xFF
  * @return None
  * @note Clock input is divided by (prescale + 1) before it is fed into timer
  * \hideinitializer
  */
#define TIMER_SET_PRESCALE_VALUE(timer, u32Value) ((timer)->CTL = ((timer)->CTL & ~TMR_CTL_PSC_Msk) | (u32Value))

/**
  * @brief This macro is used to check if specify Timer is inactive or active
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @return timer is activate or inactivate
  * @retval 0 Timer 16-bit up counter is inactive
  * @retval 1 Timer 16-bit up counter is active
  * \hideinitializer
  */
#define TIMER_IS_ACTIVE(timer) ((timer)->CTL & TMR_CTL_ACTSTS_Msk ? 1 : 0)

/**
  * @brief This function is used to start Timer counting
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @return None
  */
__STATIC_INLINE void TIMER_Start(TMR_T *timer)
{
    timer->CTL |= TMR_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to stop Timer counting
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @return None
  */
__STATIC_INLINE void TIMER_Stop(TMR_T *timer)
{
    timer->CTL &= ~TMR_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to enable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  *                 - \ref TIMERF
  * @return None
  */
__STATIC_INLINE void TIMER_EnableInt(void *timer)
{
	if (timer==TIMERF) 
		((TMRF_T *)timer)->INTSTS |= TMRF_INTSTS_IFIE_Msk;
    else 
		((TMR_T *)timer)->CTL |= TMR_CTL_INTEN_Msk;
}

/**
  * @brief This function is used to disable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  *                 - \ref TIMERF
  * @return None
  */
__STATIC_INLINE void TIMER_DisableInt(void *timer)
{
    if (timer==TIMERF) 
        ((TMRF_T *)timer)->INTSTS &= ~TMRF_INTSTS_IFIE_Msk;
	else 
        ((TMR_T *)timer)->CTL &= ~TMR_CTL_INTEN_Msk;
}

/**
  * @brief This function indicates Timer time-out interrupt occurred or not.
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  *                 - \ref TIMERF
  * @return Timer time-out interrupt occurred or not
  * @retval 0 Timer time-out interrupt did not occur
  * @retval 1 Timer time-out interrupt occurred
  */
__STATIC_INLINE uint32_t TIMER_GetIntFlag(void *timer)
{
    if (timer==TIMERF) 
        return(((TMRF_T *)timer)->INTSTS & TMRF_INTSTS_TFIF_Msk ? 1 : 0);
    else 
        return(((TMR_T *)timer)->INTSTS & TMR_INTSTS_TIF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer time-out interrupt flag.
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  *                 - \ref TIMERF
  * @return None
  */
__STATIC_INLINE void TIMER_ClearIntFlag(void *timer)
{
    if (timer==TIMERF) 
	    ((TMRF_T *)timer)->INTSTS |= TMRF_INTSTS_TFIF_Msk;
    else 
        ((TMR_T *)timer)->INTSTS |= TMR_INTSTS_TIF_Msk;
}

/**
  * @brief This function reports the current timer counter value.
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER0
  *                 - \ref TIMER1
  *                 - \ref TIMER2
  * @return Timer counter value
  */
__STATIC_INLINE uint32_t TIMER_GetCounter(TMR_T *timer)
{
    return timer->CNT;
}

/**
  * @brief This function is used to enable IR carrier output
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER_IR
  * @return None
  */
__STATIC_INLINE void TIMER_IRCarrierOutputEnable(TMRIR_T *timer)
{
    timer->CTL |= TMRIR_CTL_IRCEN_Msk;
}

/**
  * @brief This function is used to disable IR carrier output
  * @param[in] timer The base address of Timer module
  *                 - \ref TIMER_IR
  * @return None
  */
__STATIC_INLINE void TIMER_IRCarrierOutputDisable(TMRIR_T *timer)
{
    timer->CTL &= (~TMRIR_CTL_IRCEN_Msk);
}

uint32_t TIMER_Open(TMR_T *timer, uint32_t u32Mode, uint32_t u32Freq);

void     TIMER_Close(TMR_T *timer);

void     TIMER_Delay(TMR_T *timer, uint32_t u32Usec);

uint32_t TIMER_GetModuleClock(TMR_T *timer);

/*@}*/ /* end of group ISD9000_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_TIMER_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif 

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
