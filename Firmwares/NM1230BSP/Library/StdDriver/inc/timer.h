/**************************************************************************//**
 * @file     timer.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 2018/08/02 16:36 $
 * @brief    NM1230 TIMER driver header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup NM1230_TIMER_EXPORTED_CONSTANTS TIMER Exported Constants
  @{
*/

#define TIMER_ONESHOT_MODE                     (0UL)                              /*!< Timer working in one shot mode */
#define TIMER_PERIODIC_MODE                    (1UL << TIMER_CTL_OPMODE_Pos)      /*!< Timer working in periodic mode */
#define TIMER_TOGGLE_MODE                      (2UL << TIMER_CTL_OPMODE_Pos)      /*!< Timer working in toggle mode */
#define TIMER_CONTINUOUS_MODE                  (3UL << TIMER_CTL_OPMODE_Pos)      /*!< Timer working in continuous mode */
#define TIMER_CAPTURE_FREE_COUNTING_MODE       (0UL)                              /*!< Free counting mode */
#define TIMER_CAPTURE_COUNTER_RESET_MODE       (TIMER_EXTCTL_CAPFUNCS_Msk)        /*!< Counter reset mode */
#define TIMER_TRIGGER_COUNTING_CAPTURE_MODE    (TIMER_EXTCTL_CAPMODE_Msk)         /*!< Trigger counting capture mode */
#define TIMER_CAPTURE_FALLING_EDGE             (0UL)                              /*!< Falling edge trigger timer capture */
#define TIMER_CAPTURE_RISING_EDGE              (1UL << TIMER_EXTCTL_CAPEDGE_Pos)  /*!< Rising edge trigger timer capture */
#define TIMER_CAPTURE_FALLING_AND_RISING_EDGE  (2UL << TIMER_EXTCTL_CAPEDGE_Pos)  /*!< Both falling and rising edge trigger timer capture */
#define TIMER_COUNTER_RISING_EDGE              (TIMER_EXTCTL_CNTPHASE_Msk)        /*!< Counter increase on rising edge */
#define TIMER_COUNTER_FALLING_EDGE             (0UL)                              /*!< Counter increase on falling edge */
#define TIMER_CCAP_CNT_TIMER0                  (0UL << TIMER_CCAPCTL_CNTSEL_Pos)  /*!< Timer0 as counter of continuous capture */
#define TIMER_CCAP_CNT_TIMER1                  (1UL << TIMER_CCAPCTL_CNTSEL_Pos)  /*!< Timer1 as counter of continuous capture */
#define TIMER_CCAP_CNT_SYSTICK                 (2UL << TIMER_CCAPCTL_CNTSEL_Pos)  /*!< SysTick as counter of continuous capture */
#define TIMER_CCAP_CNT_TIMER2                  ((3UL << TIMER_CCAPCTL_CNTSEL_Pos) | (0UL << TIMER_CCAPCTL_CNTSEL23_Pos))  /*!<  Timer2 as counter of continuous capture */
#define TIMER_CCAP_CNT_TIMER3                  ((3UL << TIMER_CCAPCTL_CNTSEL_Pos) | (1UL << TIMER_CCAPCTL_CNTSEL23_Pos))  /*!<  Timer3 as counter of continuous capture */
#define TIMER_CCAP_CH_P0                       (0UL << TIMER_CCAPCTL_CAPCHSEL_Pos)/*!<  CCAP_P0 as capture input pin */
#define TIMER_CCAP_CH_P1                       (1UL << TIMER_CCAPCTL_CAPCHSEL_Pos)/*!<  CCAP_P1 as capture input pin */
#define TIMER_CAP_FLAG_R1F                     (TIMER_CCAPCTL_CAPR1F_Msk)         /*!<  First rising edge flag */
#define TIMER_CAP_FLAG_F1F                     (TIMER_CCAPCTL_CAPF1F_Msk)         /*!<  First falling edge flag */
#define TIMER_CAP_FLAG_R2F                     (TIMER_CCAPCTL_CAPR2F_Msk)         /*!<  Secondary rising edge flag */
#define TIMER_CAP_FLAG_F2F                     (TIMER_CCAPCTL_CAPF2F_Msk)         /*!<  Secondary falling edge flag */
#define TIMER_CCAP_INT_DISABLE                 (0UL << TIMER_CCAPCTL_CCAPIEN_Pos) /*!<  Disable CCAP interrupt */
#define TIMER_CCAP_INT_R1_F1                   (1UL << TIMER_CCAPCTL_CCAPIEN_Pos) /*!<  CCAP interrupt trigger at first falling edge */
#define TIMER_CCAP_INT_R2_F1                   (2UL << TIMER_CCAPCTL_CCAPIEN_Pos) /*!<  CCAP interrupt trigger at seconadry rising edge */
#define TIMER_CCAP_INT_R2_F2                   (3UL << TIMER_CCAPCTL_CCAPIEN_Pos) /*!<  CCAP interrupt trigger at seconadry falling edge */
#define TIMER_CCAP_NFCLKS_DIV1                 (0UL << TIMER_CCAPCTL_NFCLKS_Pos)  /*!<  Noise filiter clock divid 1 */
#define TIMER_CCAP_NFCLKS_DIV2                 (1UL << TIMER_CCAPCTL_NFCLKS_Pos)  /*!<  Noise filiter clock divid 2 */
#define TIMER_CCAP_NFCLKS_DIV4                 (2UL << TIMER_CCAPCTL_NFCLKS_Pos)  /*!<  Noise filiter clock divid 4 */
#define TIMER_CCAP_NFCLKS_DIV16                (3UL << TIMER_CCAPCTL_NFCLKS_Pos)  /*!<  Noise filiter clock divid 16 */
#define TIMER_CAP_DAT_R1F                      (0)                                /*!<  CCAP data index of first rising flag */
#define TIMER_CAP_DAT_F1F                      (1)                                /*!<  CCAP data index of first falling flag */
#define TIMER_CAP_DAT_R2F                      (2)                                /*!<  CCAP data index of seconadry rising flag */
#define TIMER_CAP_DAT_F2F                      (3)                                /*!<  CCAP data index of seconadry falling flag */

/*@}*/ /* end of group NM1230_TIMER_EXPORTED_CONSTANTS */


/** @addtogroup NM1230_TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief This macro is used to set new Timer compared value
  * @param[in] timer The pointer of the specified Timer module
  * @param[in] u32Value  Timer compare value. Valid values are between 2 to 0xFFFFFF
  * @return None
  * \hideinitializer
  */
#define TIMER_SET_CMP_VALUE(timer, u32Value) ((timer)->CMP = (u32Value))

/**
  * @brief This macro is used to set new Timer prescale value
  * @param[in] timer The pointer of the specified Timer module
  * @param[in] u32Value  Timer prescale value. Valid values are between 0 to 0xFF
  * @return None
  * @note Clock input is divided by (prescale + 1) before it is fed into timer
  * \hideinitializer
  */
#define TIMER_SET_PRESCALE_VALUE(timer, u32Value) ((timer)->CTL = ((timer)->CTL & ~TIMER_CTL_PSC_Msk) | (u32Value))

/**
  * @brief This macro is used to check if specify Timer is inactive or active
  * @param[in] timer The pointer of the specified Timer module
  * @return Timer is activate or inactivate
  * @retval 0 Timer 24-bit up counter is inactive
  * @retval 1 Timer 24-bit up counter is active
  * \hideinitializer
  */
#define TIMER_IS_ACTIVE(timer) ((timer)->CTL & TIMER_CTL_ACTSTS_Msk ? 1 : 0)

/**
  * @brief This function is used to held timer while CPU is held by ICE.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ICEDEBUG_HOLD_ENABLE(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_ICEDEBUG_Msk;
}

/**
  * @brief This function is used to not held timer while CPU is held by ICE.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ICEDEBUG_HOLD_DISABLE(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_ICEDEBUG_Msk;
}

/**
  * @brief This function is used to reset prescale, counting and CNTEN.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Reset(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_RSTCNT_Msk;
}

/**
  * @brief This function is used to start Timer counting
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to stop Timer counting
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_CNTEN_Msk;
}

/**
  * @brief This function is used to enable the Timer wake-up function
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  * @note  To wake the system from power down mode, timer clock source must be LXT or LIRC
  */
__STATIC_INLINE void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_WKEN_Msk;
}

/**
  * @brief This function is used to disable the Timer wake-up function
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_WKEN_Msk;
}

/**
  * @brief This function is used to enable the counter pin detection de-bounce function.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= TIMER_EXTCTL_ECNTDBEN_Msk;
}

/**
  * @brief This function is used to disable the counter pin detection de-bounce function.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~TIMER_EXTCTL_ECNTDBEN_Msk;
}

/**
  * @brief This function is used to enable the Timer time-out interrupt function.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= TIMER_CTL_INTEN_Msk;
}

/**
  * @brief This function is used to disable the Timer time-out interrupt function.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_INTEN_Msk;
}

/**
  * @brief This function is used to enable the Timer capture trigger interrupt function.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= TIMER_EXTCTL_CAPIEN_Msk;
}

/**
  * @brief This function is used to disable the Timer capture trigger interrupt function.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~TIMER_EXTCTL_CAPIEN_Msk;
}

/**
  * @brief This function indicates Timer time-out interrupt occurred or not.
  * @param[in] timer The pointer of the specified Timer module
  * @return Timer time-out interrupt occurred or not
  * @retval 0 Timer time-out interrupt did not occur
  * @retval 1 Timer time-out interrupt occurred
  */
__STATIC_INLINE uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->INTSTS & TIMER_INTSTS_TIF_Msk ? 1 : 0);
}

/**
  * @brief This function clears the Timer time-out interrupt flag.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = TIMER_INTSTS_TIF_Msk;
}

/**
  * @brief This function indicates Timer capture interrupt occurred or not.
  * @param[in] timer The pointer of the specified Timer module
  * @return Timer capture interrupt occurred or not
  * @retval 0 Timer capture interrupt did not occur
  * @retval 1 Timer capture interrupt occurred
  */
__STATIC_INLINE uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}

/**
  * @brief This function clears the Timer capture interrupt flag.
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = TIMER_EINTSTS_CAPIF_Msk;
}

/**
  * @brief This function indicates Timer has waked up system or not.
  * @param[in] timer The pointer of the specified Timer module
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
  * @param[in] timer The pointer of the specified Timer module
  * @return None
  */
__STATIC_INLINE void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = TIMER_INTSTS_TWKF_Msk;
}

/**
  * @brief This function gets the Timer capture data.
  * @param[in] timer The pointer of the specified Timer module
  * @return Timer capture data value
  */
__STATIC_INLINE uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}

/**
  * @brief This function reports the current timer counter value.
  * @param[in] timer The pointer of the specified Timer module
  * @return Timer counter value
  */
__STATIC_INLINE uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}

/**
  * @brief This macro is used to enabled the continuous capture function
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Enable(ccap) ((ccap)->CCAPCTL |= TIMER_CCAPCTL_CCAPEN_Msk)

/**
  * @brief This macro is used to disabled the continuous capture function
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Disable(ccap) ((ccap)->CCAPCTL &= ~TIMER_CCAPCTL_CCAPEN_Msk)

/**
  * @brief This macro is used to enable invert the input signal which be captured
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Inverse_Enable(ccap) ((ccap)->CCAPCTL |= TIMER_CCAPCTL_INV_Msk)

/**
  * @brief This macro is used to disable invert the input signal which be captured
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Inverse_Disable(ccap) ((ccap)->CCAPCTL &= ~TIMER_CCAPCTL_INV_Msk)

/**
  * @brief This macro is used to select the counter to continuous capture the input signal
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] cnt_sel Counter of continuous capture function
  *             - \ref TIMER_CCAP_CNT_TIMER0
  *             - \ref TIMER_CCAP_CNT_TIMER1
  *             - \ref TIMER_CCAP_CNT_SYSTICK
  *             - \ref TIMER_CCAP_CNT_TIMER2
  *             - \ref TIMER_CCAP_CNT_TIMER3
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Counter_Select(ccap, cnt_sel) ((ccap)->CCAPCTL = \
                                 ((ccap)->CCAPCTL & ~(TIMER_CCAPCTL_CNTSEL_Msk|TIMER_CCAPCTL_CNTSEL23_Msk)) | cnt_sel)

/**
  * @brief This macro is used to select the channel to be the continuous capture event
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] cnt_sel Channel of continuous capture event
  *             - \ref TIMER_CCAP_CH_P0
  *             - \ref TIMER_CCAP_CH_P1
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Channel_Select(ccap, ch_sel) ((ccap)->CCAPCTL = \
                                ((ccap)->CCAPCTL & ~(TIMER_CCAPCTL_CAPCHSEL_Msk)) | ch_sel)

/**
  * @brief This macro is used to get CCAP interrupt flag
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] flag_msk Mask of capture flag
  *             - \ref TIMER_CAP_FLAG_R1F
  *             - \ref TIMER_CAP_FLAG_F1F
  *             - \ref TIMER_CAP_FLAG_R2F
  *             - \ref TIMER_CAP_FLAG_F2F
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Get_Flag(ccap, flag_msk) ((ccap)->CCAPCTL & flag_msk)

/**
  * @brief This macro is used to clear CCAP interrupt flag
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] flag_msk Mask of capture flag
  *             - \ref TIMER_CAP_FLAG_R1F
  *             - \ref TIMER_CAP_FLAG_F1F
  *             - \ref TIMER_CAP_FLAG_R2F
  *             - \ref TIMER_CAP_FLAG_F2F
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_CLR_Flag(ccap, flag_msk) ((ccap)->CCAPCTL |= (flag_msk) & \
                            (TIMER_CCAPCTL_CAPR1F_Msk | TIMER_CCAPCTL_CAPR1F_Msk |\
                             TIMER_CCAPCTL_CAPR2F_Msk | TIMER_CCAPCTL_CAPR2F_Msk))

/**
  * @brief This macro is used to select interrupt trigger type of continuous capture 
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] int_sel Interrupt trigger type
  *             - \ref TIMER_CCAP_INT_DISABLE
  *             - \ref TIMER_CCAP_INT_R1_F1   
  *             - \ref TIMER_CCAP_INT_R2_F1   
  *             - \ref TIMER_CCAP_INT_R2_F2   
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Interrupt_Type_Select(ccap, int_sel) ((ccap)->CCAPCTL = \
                                (((ccap)->CCAPCTL & ~(TIMER_CCAPCTL_CCAPIEN_Msk)) | int_sel))

/**
  * @brief This macro is used to enable noise filiter of CCAP capture clock
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] clk_div Divided option of the noise filter clock 
  *             - \ref TIMER_CCAP_NFCLKS_DIV1
  *             - \ref TIMER_CCAP_NFCLKS_DIV2
  *             - \ref TIMER_CCAP_NFCLKS_DIV4
  *             - \ref TIMER_CCAP_NFCLKS_DIV16
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_NFCLKS_Enable(ccap, clk_div) ((ccap)->CCAPCTL = \
                          (((ccap)->CCAPCTL & ~(TIMER_CCAPCTL_NFDIS_Msk | TIMER_CCAPCTL_NFCLKS_Msk)) | clk_div))

/**
  * @brief This macro is used to disable noise filiter of CCAP capture clock
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_NFCLKS_Disable(ccap) ((ccap)->CCAPCTL |= TIMER_CCAPCTL_NFDIS_Msk)

/**
  * @brief This macro is used to get capture data
  * @param[in] ccap The pointer of the specified Timer_CCAP module
  * @param[in] dat_idx Index of capture data
  *             - \ref TIMER_CAP_DAT_R1F
  *             - \ref TIMER_CAP_DAT_F1F
  *             - \ref TIMER_CAP_DAT_R2F
  *             - \ref TIMER_CAP_DAT_F2F
  * @return None
  * \hideinitializer
  */
#define TIMER_CCAP_Get_CAPDAT(ccap, dat_idx) ((ccap)->CCAP[dat_idx])


uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);


/*@}*/ /* end of group NM1230_TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_TIMER_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif 

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
