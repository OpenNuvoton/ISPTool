/**************************************************************************//**
 * @file     pwm.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/10 2:52p $
 * @brief    I91200 series PWM driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PWM_H__
#define __PWM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_PWM_Driver PWM Driver
  @{
*/

/** @addtogroup I91200_PWM_EXPORTED_CONSTANTS PWM Exported Constants
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
#define PWM_PERIOD_INT_UNDERFLOW            (0)                             /*!< PWM period interrupt triggered if counter underflow */
//#define PWM_PERIOD_INT_MATCH_CNR            (PWM_PIER_INT01TYPE_Msk)    /*!< PWM period interrupt triggered if counter match CNR */
#define PWM_CAPTURE_INT_RISING_LATCH        (PWM_CAPCTL01_CRLIEN0_Msk)      /*!< PWM capture interrupt if channel has rising transition */
#define PWM_CAPTURE_INT_FALLING_LATCH       (PWM_CAPCTL01_CFLIEN0_Msk)      /*!< PWM capture interrupt if channel has falling transition */
/*---------------------------------------------------------------------------------------------------------*/
/*  PWM Group channel number constants definitions                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define PWM_CH0                             0x0                         /*!< PWM Group 0/1 channel 0 */
#define PWM_CH1                             0x1                         /*!< PWM Group 0/1 channel 1 */
#define PWM_CH2                             0x2                         /*!< PWM Group 0/1 channel 2 */
#define PWM_CH3                             0x3                         /*!< PWM Group 0/1 channel 3 */
#define PWM_CCR_MASK                        0x000F000F                  /*!< PWM CCR0/CCR2 bit0~3 and bit16~19 mask */

/*@}*/ /* end of group I91200_PWM_EXPORTED_CONSTANTS */


/** @addtogroup I91200_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief Enable output inverter of specified channel(s) and disable others
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * @details This macro is used to enable capture input inverter for specified channel(s) and disable others.
 * \hideinitializer
 */
#define PWM_ENABLE_OUTPUT_INVERTER(pwm, u32ChannelMask) \
    { \
        int i;\
                (pwm)->CTL &= ~(PWM_CTL_PINV0_Msk|PWM_CTL_PINV1_Msk|PWM_CTL_PINV2_Msk|PWM_CTL_PINV3_Msk);\
        for(i = 0; i < PWM_CHANNEL_NUM; i++) { \
            if((u32ChannelMask) & (1 << i)) \
                (pwm)->CTL |= (PWM_CTL_PINV0_Msk << (PWM_CTL_PINV0_Pos * (i * 4))); \
        } \
    }

/**
 * @brief Get captured rising data of specified channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @return The timer counter, 0~0xFFFF
 * @details This macro is used to get captured rising data for specified channel.
 */
#define PWM_GET_CAPTURE_RISING_DATA(pwm, u32ChannelNum) (*((__IO uint32_t *) ((((uint32_t)&((pwm)->RCAPDAT0)) + (u32ChannelNum) * 8))))

/**
 * @brief Get captured falling data of specified channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @return The timer counter, 0~0xFFFF
 * @details This macro is used to get captured falling data for specified channel.
 */
#define PWM_GET_CAPTURE_FALLING_DATA(pwm, u32ChannelNum) (*((__IO uint32_t *) ((((uint32_t)&((pwm)->FCAPDAT0)) + (u32ChannelNum) * 8))))

/**
 * @brief Set the prescaler of the selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32Prescaler Clock prescaler of specified channel. Valid values are between 1 ~ 0xFF
 * @return None
 * @details This macro is used to set timer pre-scale for specified channel.
 * @note If u32Prescaler = 0, corresponding PWM-timer will be stopped.
 * @note If u32Prescaler = x (x not equal to 0), it means Clock input is divided by (x + 1) before it is fed to the corresponding PWM counter.
 */
#define PWM_SET_PRESCALER(pwm, u32ChannelNum, u32Prescaler) \
    ((pwm)->CLKPSC = ((pwm)->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << (((u32ChannelNum) >> 1) * 8))) | ((u32Prescaler) << (((u32ChannelNum) >> 1) * 8)))

/**
 * @brief Set the divider of the selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref PWM_CLK_DIV_1
 *              - \ref PWM_CLK_DIV_2
 *              - \ref PWM_CLK_DIV_4
 *              - \ref PWM_CLK_DIV_8
 *              - \ref PWM_CLK_DIV_16
 * @return None
 * @details This macro is used to set Timer clock source divider selection for specified channel.
 */
#define PWM_SET_DIVIDER(pwm, u32ChannelNum, u32Divider) \
    ((pwm)->CLKDIV = ((pwm)->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << ((u32ChannelNum) * 4))) | ((u32Divider) << ((u32ChannelNum) * 4)))

/**
 * @brief Set the duty of the selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set PWM Comparator value for specified channel.
 * @note This new setting will take effect on next PWM period.
 */
#define PWM_SET_CMR(pwm, u32ChannelNum, u32CMR) (*((__IO uint32_t *) ((((uint32_t)&((pwm)->CMPDAT0)) + (u32ChannelNum) * 12))) = u32CMR)

/**
 * @brief Set the period of the selected channel
 * @param[in] pwm The base address of PWM module
 *                - PWM0 : PWM Group 0
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~1
 * @param[in] u32CNR Period of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set timer loaded value(CNR) for specified channel.\n
 *          Loaded value determines the PWM period.
 * @note This new setting will take effect on next PWM period.
 * @note PWM counter will stop if period length set to 0.
 */
#define PWM_SET_CNR(pwm, u32ChannelNum, u32CNR)  (*((__IO uint32_t *) ((((uint32_t)&((pwm)->PERIOD0)) + (u32ChannelNum) * 12))) = u32CNR)



uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequncy,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntType);
void PWM_DisableInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);


/*@}*/ /* end of group I91200_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_PWM_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__PWM_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
