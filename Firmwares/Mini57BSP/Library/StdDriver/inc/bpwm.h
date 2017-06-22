/**************************************************************************//**
 * @file     bpwm.h
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series BPWM driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __BPWM_H__
#define __BPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_BPWM_Driver BPWM Driver
  @{
*/

/** @addtogroup Mini57_BPWM_EXPORTED_CONSTANTS BPWM Exported Constants
  @{
*/
#define BPWM_CHANNEL_NUM                     (2)   			                 /*!< BPWM channel number */
#define BPWM_CLK_DIV_1                       (4UL) 			                 /*!< BPWM clock divide by 1 */
#define BPWM_CLK_DIV_2                       (0UL) 			                 /*!< BPWM clock divide by 2 */
#define BPWM_CLK_DIV_4                       (1UL) 			                 /*!< BPWM clock divide by 4 */
#define BPWM_CLK_DIV_8                       (2UL)                       /*!< BPWM clock divide by 8 */
#define BPWM_CLK_DIV_16                      (3UL)                       /*!< BPWM clock divide by 16 */
#define BPWM_EDGE_ALIGNED                    (0UL)                       /*!< BPWM working in edge aligned type */
#define BPWM_CENTER_ALIGNED                  (1UL)                       /*!< BPWM working in center aligned type */
#define BPWM_PERIOD_INT_UNDERFLOW            (0)                         /*!< BPWM period interrupt triggered if counter underflow */
#define BPWM_PERIOD_INT_MATCH_CNR            (BPWM_INTEN_PINTTYPE_Msk)   /*!< BPWM period interrupt triggered if counter match PERIODn */
/*---------------------------------------------------------------------------------------------------------*/
/*  BPWM channel number constants definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define BPWM_CH0                             0x0                         /*!< BPWM channel 0 */
#define BPWM_CH1                             0x1                         /*!< BPWM channel 1 */

/*@}*/ /* end of group Mini57_BPWM_EXPORTED_CONSTANTS */


/** @addtogroup Mini57_BPWM_EXPORTED_FUNCTIONS BPWM Exported Functions
  @{
*/

/**
 * @brief Enable output inverter of specified channel(s)
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * @details This macro is used to enable output inverter for specified channel(s).
 * \hideinitializer
 */
#define BPWM_ENABLE_OUTPUT_INVERTER(bpwm, u32ChannelMask) \
    do{ \
        int i;\
                (bpwm)->CTL &= ~(BPWM_CTL_PINV0_Msk|BPWM_CTL_PINV1_Msk);\
        for(i = 0; i < 2; i++) { \
            if((u32ChannelMask) & (1 << i)) \
                (bpwm)->CTL |= (1 << (BPWM_CTL_PINV0_Pos + (i * 8))); \
        } \
    }while(0)

/**
 * @brief Set the prescaler of the selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32Prescaler Clock prescaler of specified channel. Valid values are between 1 ~ 0xFF
 * @return None
 * @details This macro is used to set timer pre-scale for specified channel.
 * @note If u32Prescaler = 0, corresponding PWM-timer will be stopped.
 * @note If u32Prescaler = x (x not equal to 0), it means Clock input is divided by (x + 1) before it is fed to the corresponding BPWM counter.
 * \hideinitializer
 */
#define BPWM_SET_PRESCALER(bpwm, u32ChannelNum, u32Prescaler) \
    ((bpwm)->CLKPSC = ((bpwm)->CLKPSC & ~(BPWM_CLKPSC_CLKPSC01_Msk << (((u32ChannelNum) >> 1) * 8))) | ((u32Prescaler) << (((u32ChannelNum) >> 1) * 8)))

/**
 * @brief Set the divider of the selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref BPWM_CLK_DIV_1
 *              - \ref BPWM_CLK_DIV_2
 *              - \ref BPWM_CLK_DIV_4
 *              - \ref BPWM_CLK_DIV_8
 *              - \ref BPWM_CLK_DIV_16
 * @return None
 * @details This macro is used to set Timer clock source divider selection for specified channel.
 * \hideinitializer
 */
#define BPWM_SET_DIVIDER(bpwm, u32ChannelNum, u32Divider) \
    ((bpwm)->CLKDIV = ((bpwm)->CLKDIV & ~(BPWM_CLKDIV_CLKDIV0_Msk << ((u32ChannelNum) * 4))) | ((u32Divider) << ((u32ChannelNum) * 4)))

/**
 * @brief Set the duty of the selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set BPWM Comparator value for specified channel.
 * @note This new setting will take effect on next BPWM period.
 * \hideinitializer
 */
#define BPWM_SET_CMR(bpwm, u32ChannelNum, u32CMR) (*((__IO uint32_t *) ((((uint32_t)&((bpwm)->CMPDAT0)) + (u32ChannelNum) * 12))) = (u32CMR))

/**
 * @brief Set the period of the selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32CNR Period of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set the period of specified channel.
 * @note This new setting will take effect on next BPWM period.
 * @note BPWM counter will stop if period length set to 0.
 * \hideinitializer
 */
#define BPWM_SET_CNR(bpwm, u32ChannelNum, u32CNR)  (*((__IO uint32_t *) ((((uint32_t)&((bpwm)->PERIOD0)) + (u32ChannelNum) * 12))) = (u32CNR))

/**
 * @brief Set the BPWM aligned type
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32AlignedType BPWM aligned type, valid values are:
 *                  - \ref BPWM_EDGE_ALIGNED
 *                  - \ref BPWM_CENTER_ALIGNED
 * @return None
 * @details This macro is used to set the BPWM aligned type.
 * \hideinitializer
 */
#define BPWM_SET_ALIGNED_TYPE(bpwm, u32ChannelMask, u32AlignedType) \
    do{ \
        int i; \
        for(i = 0; i < 2; i++) { \
            if((u32ChannelMask) & (1 << i)) \
                (bpwm)->CTL = ((bpwm)->CTL & ~(BPWM_CTL_CNTTYPE01_Msk << (i >> 1))) | ((u32AlignedType) << (BPWM_CTL_CNTTYPE01_Pos + (i >> 1))); \
        } \
    }while(0)


uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequncy,
                                  uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void BPWM_DisableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);



/*@}*/ /* end of group Mini57_BPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_BPWM_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif 

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
