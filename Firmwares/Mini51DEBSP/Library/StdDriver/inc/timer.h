/**************************************************************************//**
 * @file     timer.h
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 14/01/13 7:43p $
 * @brief    Mini51 series TIMER driver header file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup MINI51_TIMER_EXPORTED_CONSTANTS TIMER Exported Constants
  @{
*/

#define TIMER_ONESHOT_MODE                  (0UL)                           /*!< Timer working in one shot mode */
#define TIMER_PERIODIC_MODE                 (1UL << TIMER_TCSR_MODE_Pos)    /*!< Timer working in periodic mode */
#define TIMER_TOGGLE_MODE                   (2UL << TIMER_TCSR_MODE_Pos)    /*!< Timer working in toggle mode */
#define TIMER_CONTINUOUS_MODE               (3UL << TIMER_TCSR_MODE_Pos)    /*!< Timer working in continuous mode */
#define TIMER_CAPTURE_FREE_COUNTING_MODE    (0UL)                           /*!< Free counting mode */
#define TIMER_CAPTURE_TRIGGER_COUNTING_MODE (TIMER_TEXCON_CAP_MODE_Msk)     /*!< Trigger counting mode */
#define TIMER_CAPTURE_COUNTER_RESET_MODE    (TIMER_TEXCON_RSTCAPSEL_Msk)    /*!< Counter reset mode */
#define TIMER_CAPTURE_FALLING_EDGE              (0UL)                               /*!< Falling edge trigger timer capture */
#define TIMER_CAPTURE_RISING_EDGE               (1UL << TIMER_TEXCON_TEX_EDGE_Pos)  /*!< Rising edge trigger timer capture */
#define TIMER_CAPTURE_FALLING_THEN_RISING_EDGE  (2UL << TIMER_TEXCON_TEX_EDGE_Pos)  /*!< Falling edge then rising edge trigger timer capture */
#define TIMER_CAPTURE_RISING_THEN_FALLING_EDGE  (3UL << TIMER_TEXCON_TEX_EDGE_Pos)  /*!< Rising edge then falling edge trigger timer capture */
#define TIMER_COUNTER_RISING_EDGE           (TIMER_TEXCON_TX_PHASE_Msk)     /*!< Counter increase on rising edge */
#define TIMER_COUNTER_FALLING_EDGE          (0UL)                           /*!< Counter increase on falling edge */
#define TIMER_TOGGLE_TX                     (0UL)                           /*!< Timer use Tx, x = 0, 1 pin as toggle output pin   */
#define TIMER_TOGGLE_TXEX                   (TIMER_TCSR_TOGGLE_PIN_Msk)     /*!< Timer use TxEX, x = 0, 1 pin as toggle output pin */

/*@}*/ /* end of group MINI51_TIMER_EXPORTED_CONSTANTS */


/** @addtogroup MINI51_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This macro is used to set new Timer compared value
  * @param[in] timer The base address of Timer module
  * @param[in] u32Value  Timer compare value. Valid values are between 2 to 0xFFFFFF
  * @return None
  * \hideinitializer
  */
#define TIMER_SET_CMP_VALUE(timer, u32Value) ((timer)->TCMPR = (u32Value))

/**
  * @brief This macro is used to set new Timer prescale value
  * @param[in] timer The base address of Timer module
  * @param[in] u32Value  Timer prescale value. Valid values are between 0 to 0xFF
  * @return None
  * @note Clock input is divided by (prescale + 1) before it is fed into timer
  * \hideinitializer
  */
#define TIMER_SET_PRESCALE_VALUE(timer, u32Value) ((timer)->TCSR = ((timer)->TCSR & ~TIMER_TCSR_PRESCALE_Msk) | (u32Value))

/**
  * @brief This macro is used to check if specify Timer is inactive or active
  * @param[in] timer The base address of Timer module
  * @return timer is activate or inactivate
  * @retval 0 Timer 24-bit up counter is inactive
  * @retval 1 Timer 24-bit up counter is active
  * \hideinitializer
  */
#define TIMER_IS_ACTIVE(timer) ((timer)->TCSR & TIMER_TCSR_CACT_Msk ? 1 : 0)

/**
  * @brief This macro is used to select Timer toggle output pin
  * @param[in] timer The base address of Timer module
  * @param[in] u32ToutSel Toggle output pin selection, valid values are
  *                 - \ref TIMER_TOGGLE_TX
  *                 - \ref TIMER_TOGGLE_TXEX
  * @return None
  * \hideinitializer
  */
#define TIMER_SELECT_TOUT_PIN(timer, u32ToutSel) ((timer)->TCSR = ((timer)->TCSR & ~TIMER_TCSR_TOGGLE_PIN_Msk) | (u32ToutSel))



/**
  * @brief This function is used to start Timer counting
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= TIMER_TCSR_CEN_Msk;
}

/**
  * @brief This function is used to stop Timer counting
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~TIMER_TCSR_CEN_Msk;
}

/**
  * @brief This function is used to enable the Timer wake-up function
  * @param[in] timer The base address of Timer module
  * @return None
  * @note  To wake the system from power down mode, timer clock source must be ether LXT or LIRC
  */
__STATIC_INLINE void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= TIMER_TCSR_WAKE_EN_Msk;
}

/**
  * @brief This function is used to disable the Timer wake-up function
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~TIMER_TCSR_WAKE_EN_Msk;
}


/**
  * @brief This function is used to enable the capture pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= TIMER_TEXCON_TEXDB_Msk;
}

/**
  * @brief This function is used to disable the capture pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~TIMER_TEXCON_TEXDB_Msk;
}


/**
  * @brief This function is used to enable the counter pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= TIMER_TEXCON_TCDB_Msk;
}

/**
  * @brief This function is used to disable the counter pin detection de-bounce function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~TIMER_TEXCON_TCDB_Msk;
}

/**
  * @brief This function is used to enable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= TIMER_TCSR_IE_Msk;
}

/**
  * @brief This function is used to disable the Timer time-out interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~TIMER_TCSR_IE_Msk;
}

/**
  * @brief This function is used to enable the Timer capture trigger interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= TIMER_TEXCON_TEXIEN_Msk;
}

/**
  * @brief This function is used to disable the Timer capture trigger interrupt function.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~TIMER_TEXCON_TEXIEN_Msk;
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
    return(timer->TISR & TIMER_TISR_TIF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer time-out interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = TIMER_TISR_TIF_Msk;
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
    return timer->TEXISR;
}

/**
  * @brief This function clears the Timer capture interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = TIMER_TEXISR_TEXIF_Msk;
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
    return (timer->TISR & TIMER_TISR_TWF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer wakeup interrupt flag.
  * @param[in] timer The base address of Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = TIMER_TISR_TWF_Msk;
}

/**
  * @brief This function gets the Timer capture data.
  * @param[in] timer The base address of Timer module
  * @return Timer capture data value
  */
__STATIC_INLINE uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}

/**
  * @brief This function reports the current timer counter value.
  * @param[in] timer The base address of Timer module
  * @return Timer counter value
  */
__STATIC_INLINE uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);


/*@}*/ /* end of group MINI51_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_TIMER_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__TIMER_H__

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
