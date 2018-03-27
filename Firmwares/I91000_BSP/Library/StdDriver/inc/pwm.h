/**************************************************************************//**
 * @file     pwm.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 2:52p $
 * @brief    ISD9000 series PWM driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PWM_H__
#define __PWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_PWM_Driver PWM Driver
  @{
*/

/** @addtogroup ISD9000_PWM_EXPORTED_CONSTANTS PWM Exported Constants
  @{
*/
#define PWM_CHANNEL_NUM                     (4)   /*!< PWM channel number */
#define PWM_CLK_DIV_1                       (4UL) /*!< PWM clock divide by 1 */
#define PWM_CLK_DIV_2                       (0UL) /*!< PWM clock divide by 2 */
#define PWM_CLK_DIV_4                       (1UL) /*!< PWM clock divide by 4 */
#define PWM_CLK_DIV_8                       (2UL) /*!< PWM clock divide by 8 */
#define PWM_CLK_DIV_16                      (3UL) /*!< PWM clock divide by 16 */
#define PWM_EDGE_ALIGNED                    (0UL) /*!< PWM working in edge aligned type */
#define PWM_CENTER_ALIGNED                  (1UL) /*!< PWM working in center aligned type */
#define PWM_PERIOD_INT_UNDERFLOW            (0)                         /*!< PWM period interrupt triggered if counter underflow */
#define PWM_CAPTURE_INT_RISING_LATCH        (PWM_CAPCTL_CRLIEN_Msk)      /*!< PWM capture interrupt if channel has rising transition */
#define PWM_CAPTURE_INT_FALLING_LATCH       (PWM_CAPCTL_CFLIEN_Msk)      /*!< PWM capture interrupt if channel has falling transition */
/*---------------------------------------------------------------------------------------------------------*/
/*  PWM Group channel number constants definitions                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define PWM_CH0                             0x0                         /*!< PWM channel 0 */
#define PWM_CH1                             0x1                         /*!< PWM channel 1 */
#define PWM_CH2                             0x2                         /*!< PWM channel 2 */
#define PWM_CH3                             0x3                         /*!< PWM channel 3 */
#define PWM_CCR_MASK                        0x0000000F                  /*!< PWM CCR0/CCR2 bit0~3 and bit16~19 mask */

/*@}*/ /* end of group ISD9000_PWM_EXPORTED_CONSTANTS */


/** @addtogroup ISD9000_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief Enable output inverter of specified channel(s)
 * @param[in] pwm The base address of PWM module
 * @return None
 * @details This macro is used to enable capture input inverter
 * \hideinitializer
 */
#define PWM_ENABLE_OUTPUT_INVERTER(pwm) \
                (pwm)->CTL |= PWM_CTL_PINV_Msk

/**
 * @brief Get captured rising data 
 * @param[in] pwm The base address of PWM module
 * @return The timer counter, 0~0xFFFF
 * @details This macro is used to get captured rising data.
 */
#define PWM_GET_CAPTURE_RISING_DATA(pwm) (pwm)->RCAPDAT

/**
 * @brief Get captured falling data
 * @param[in] pwm The base address of PWM module
 * @return The timer counter, 0~0xFFFF
 * @details This macro is used to get captured falling data.
 */
#define PWM_GET_CAPTURE_FALLING_DATA(pwm) (pwm)->FCAPDAT

/**
 * @brief Set the prescaler
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Prescaler Clock prescaler. Valid values are between 1 ~ 0xFF
 * @return None
 * @details This macro is used to set timer pre-scale.
 * @note If u32Prescaler = 0, corresponding PWM-timer will be stopped.
 * @note If u32Prescaler = x (x not equal to 0), it means Clock input is divided by (x + 1) before it is fed to the corresponding PWM counter.
 */
#define PWM_SET_PRESCALER(pwm, u32Prescaler) \
    (pwm)->CLKPSC = ((pwm)->CLKPSC & (~PWM_CLKPSC_CLKPSC_Msk)) | (u32Prescaler) 

/**
 * @brief Set the divider
 * @param[in] pwm The base address of PWM module
 * @param[in] u32Divider Clock divider. Valid values are
 *              - \ref PWM_CLK_DIV_1
 *              - \ref PWM_CLK_DIV_2
 *              - \ref PWM_CLK_DIV_4
 *              - \ref PWM_CLK_DIV_8
 *              - \ref PWM_CLK_DIV_16
 * @return None
 * @details This macro is used to set Timer clock source divider selection.
 */
#define PWM_SET_DIVIDER(pwm, u32Divider) \
    (pwm)->CLKDIV = ((pwm)->CLKDIV & (~PWM_CLKDIV_CLKDIV_Msk)) | (u32Divider)


uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,uint32_t u32UnitTimeNsec,uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,uint32_t u32Frequency,uint32_t u32DutyCycle0, uint32_t u32DutyCycle1, uint32_t u32DutyCycle2, uint32_t u32DutyCycle3);
void PWM_SetCNR(PWM_T *pwm,uint16_t u16CNR);
void PWM_SetCMR(PWM_T *pwm,uint32_t u32ChannelNum,uint16_t u16CMR);
void PWM_Start(PWM_T *pwm);
void PWM_Stop(PWM_T *pwm);
void PWM_EnableCapture(PWM_T *pwm);
void PWM_DisableCapture(PWM_T *pwm);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32Channel);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32Channel);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm);
void PWM_EnableInt(PWM_T *pwm);
void PWM_DisableInt(PWM_T *pwm);
void PWM_ClearIntFlag(PWM_T *pwm);
uint32_t PWM_GetIntFlag(PWM_T *pwm);

/*@}*/ /* end of group ISD9000_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_PWM_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__PWM_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
