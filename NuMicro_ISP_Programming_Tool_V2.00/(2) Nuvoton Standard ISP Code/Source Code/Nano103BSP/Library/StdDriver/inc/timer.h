/**************************************************************************//**
 * @file     timer.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/12/03 5:44p $
 * @brief    NANO103 series TIMER driver header file
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


/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup NANO103_TIMER_EXPORTED_CONSTANTS TIMER Exported Constants
  @{
*/

#define TIMER_ONESHOT_MODE                      (0UL)                               /*!< Timer working in one shot mode   */
#define TIMER_PERIODIC_MODE                     (1UL << TIMER_CTL_OPMODE_Pos)       /*!< Timer working in periodic mode   */
#define TIMER_TOGGLE_MODE                       (2UL << TIMER_CTL_OPMODE_Pos)       /*!< Timer working in toggle mode     */
#define TIMER_CONTINUOUS_MODE                   (3UL << TIMER_CTL_OPMODE_Pos)       /*!< Timer working in continuous mode */

#define TIMER_CAPTURE_FREE_COUNTING_MODE        (0UL)                               /*!< Free counting mode    */
#define TIMER_CAPTURE_TRIGGER_COUNTING_MODE     (TIMER_CTL_CAPCNTMD_Msk)            /*!< Trigger counting mode */
#define TIMER_CAPTURE_COUNTER_RESET_MODE        (TIMER_CTL_CAPFUNCS_Msk)            /*!< Counter reset mode    */

#define TIMER_CAPTURE_FALLING_EDGE              (0UL)                               /*!< Falling edge trigger timer capture */
#define TIMER_CAPTURE_RISING_EDGE               (1UL << TIMER_CTL_CAPEDGE_Pos)      /*!< Rising edge trigger timer capture  */
#define TIMER_CAPTURE_FALLING_THEN_RISING_EDGE  (2UL << TIMER_CTL_CAPEDGE_Pos)      /*!< Falling edge then rising edge trigger timer capture */
#define TIMER_CAPTURE_RISING_THEN_FALLING_EDGE  (3UL << TIMER_CTL_CAPEDGE_Pos)      /*!< Rising edge then falling edge trigger timer capture */

#define TIMER_COUNTER_RISING_EDGE               (TIMER_CTL_CNTPHASE_Msk)            /*!< Counter increase on rising edge  */
#define TIMER_COUNTER_FALLING_EDGE              (0UL)                               /*!< Counter increase on falling edge */

#define TIMER_TIMEOUT_TRIGGER                   (0UL)                               /*!< Timer timeout trigger other modules */
#define TIMER_CAPTURE_TRIGGER                   (TIMER_CTL_TRGSSEL_Msk)             /*!< Timer capture trigger other modules */

/*@}*/ /* end of group NANO103_TIMER_EXPORTED_CONSTANTS */


/** @addtogroup NANO103_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
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
#define TIMER_SET_PRESCALE_VALUE(timer, u32Value) ((timer)->PRECNT = (u32Value))

/**
  * @brief This macro is used to check if specify Timer is inactive or active
  * @return timer is activate or inactivate
  * @retval 0 Timer 24-bit up counter is inactive
  * @retval 1 Timer 24-bit up counter is active
  * \hideinitializer
  */
#define TIMER_IS_ACTIVE(timer) ((timer)->CTL & TIMER_CTL_ACTSTS_Msk ? 1 : 0)


/**
  * @brief This function is used to start Timer counting
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to stop Timer counting
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to enable the Timer wake-up function
  * @param[in] timer The base address of Timer module
  * @return None
  * @note  To wake the system from power down mode, timer clock source must be ether LXT or LIRC
  */
__STATIC_INLINE void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_WKEN_Msk;
}

/**
  * @brief This function is used to disable the Timer wake-up function
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_WKEN_Msk;
}


/**
  * @brief This function is used to enable the capture pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_CAPDBEN_Msk;
}

/**
  * @brief This function is used to disable the capture pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_CAPDBEN_Msk;
}


/**
  * @brief This function is used to enable the counter pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_CNTDBEN_Msk;
}

/**
  * @brief This function is used to disable the counter pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_CNTDBEN_Msk;
}

/**
  * @brief This function is used to enable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableInt(TIMER_T *timer)
{
    timer->INTEN |= TIMER_INTEN_CNTIEN_Msk;
}

/**
  * @brief This function is used to disable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableInt(TIMER_T *timer)
{
    timer->INTEN &= ~TIMER_INTEN_CNTIEN_Msk;
}

/**
  * @brief This function is used to enable the Timer capture trigger interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->INTEN |= TIMER_INTEN_CAPIEN_Msk;
}

/**
  * @brief This function is used to disable the Timer capture trigger interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->INTEN &= ~TIMER_INTEN_CAPIEN_Msk;
}

/**
  * @brief This function indicates Timer time-out interrupt occurred or not.
  * @param[in] timer The base address of Timer module
  * @return Timer time-out interrupt occurred or not
  * @retval 0 Timer time-out interrupt did not occur
  * @retval 1 Timer time-out interrupt occurred
  */
__STATIC_INLINE uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->INTSTS & TIMER_INTSTS_CNTIF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer time-out interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = TIMER_INTSTS_CNTIF_Msk;
}

/**
  * @brief This function indicates Timer capture interrupt occurred or not.
  * @param[in] timer The base address of Timer module
  * @return Timer capture interrupt occurred or not
  * @retval 0 Timer capture interrupt did not occur
  * @retval 1 Timer capture interrupt occurred
  */
__STATIC_INLINE uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return(timer->INTSTS & TIMER_INTSTS_CAPIF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer capture interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->INTSTS = TIMER_INTSTS_CAPIF_Msk;
}

/**
  * @brief This function indicates Timer has waked up system or not.
  * @param[in] timer The base address of Timer module
  * @return Timer has waked up system or not
  * @retval 0 Timer did not wake up system
  * @retval 1 Timer wake up system
  */
__STATIC_INLINE uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & TIMER_INTSTS_TWKF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer wakeup interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = TIMER_INTSTS_TWKF_Msk;
}

/**
  * @brief This function gets the Timer capture data.
  * @param[in] timer The base address of Timer module
  * @return Timer capture data value
  */
__STATIC_INLINE uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}

/**
  * @brief This function reports the current timer counter value.
  * @param[in] timer The base address of Timer module
  * @return Timer counter value
  */
__STATIC_INLINE uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return (timer->CNT & 0xFFFFFF);
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_ResetCounter(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);

/*@}*/ /* end of group NANO103_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_TIMER_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__TIMER_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
