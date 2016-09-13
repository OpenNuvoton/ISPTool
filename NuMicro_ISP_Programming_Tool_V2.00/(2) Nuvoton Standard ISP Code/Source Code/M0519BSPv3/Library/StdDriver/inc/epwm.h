/**************************************************************************//**
 * @file     epwm.h
 * @version  V3.00
 * $Revision: 12 $
 * $Date: 15/04/01 7:31p $
 * @brief    EPWM driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __EPWM_H__
#define __EPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup EPWM_Driver EPWM Driver
  @{
*/

/** @addtogroup EPWM_EXPORTED_CONSTANTS EPWM Exported Constants
  @{
*/
#define EPWM_CHANNEL_NUM                 (6)   /*!< EPWM channel number */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Mode Selection Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_INDEPENDENT_MODE            (0UL) /*!< Independent mode */
#define EPWM_COMPLEMENTARY_MODE          (1UL) /*!< Complementary mode */
#define EPWM_SYNCHRONIZED_MODE           (2UL) /*!< Synchronized mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Mode Clock Pre-Divider Selection Constant Definitions                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_CLK_DIV_1                   (0)                             /*!< EPWM clock divide by 1 */
#define EPWM_CLK_DIV_2                   (1UL << EPWM_PWMCON_PWMDIV_Pos) /*!< EPWM clock divide by 2 */
#define EPWM_CLK_DIV_4                   (2UL << EPWM_PWMCON_PWMDIV_Pos) /*!< EPWM clock divide by 4 */
#define EPWM_CLK_DIV_16                  (3UL << EPWM_PWMCON_PWMDIV_Pos) /*!< EPWM clock divide by 16 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Aligned Type Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_EDGE_ALIGNED                (0)                         /*!< EPWM working in edge aligned type */
#define EPWM_CENTER_ALIGNED              (EPWM_PWMCON_PWMTYPE_Msk)   /*!< EPWM working in center aligned type */

/*---------------------------------------------------------------------------------------------------------*/
/*  Edge Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_EDGE_INT_FALLING    (0)                          /*!< EPWM edge interrupt triggered if falling edge is detected at output channel */
#define EPWM_EDGE_INT_RISING     (EPWM_PWMEIC_EINT0_TYPE_Msk) /*!< EPWM edge interrupt triggered if rising edge is detected at output channel */

/*---------------------------------------------------------------------------------------------------------*/
/*  Period Interrupt Type Constant Definitions                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_PERIOD_INT_UNDERFLOW        (0)                         /*!< EPWM period interrupt triggered if counter underflow */
#define EPWM_PERIOD_INT_MATCH_CNR        (EPWM_PWMCON_INT_TYPE_Msk)  /*!< EPWM period interrupt triggered if counter match PWMP */

/*---------------------------------------------------------------------------------------------------------*/
/*  Fail Brake Control Constant Definitions                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_FB0_EDGE_BKP0     (EPWM_PWMCON_BK0_EN_Msk)                                    /*!< BKP0 pin as brake edge-detector 0 source */
#define EPWM_FB0_EDGE_ACMP0    (EPWM_PWMCON_CPO0BK_EN_Msk | EPWM_PWMCON_BK0_EN_Msk)        /*!< Comparator 0 as brake edge-detector 0 source */
#define EPWM_FB0_EDGE_ACMP1    (EPWM_PWMCON_CPO1BK_EN_Msk | EPWM_PWMCON_BK0_EN_Msk)        /*!< Comparator 1 as brake edge-detector 0 source */
#define EPWM_FB0_EDGE_ACMP2    (EPWM_PWMCON_CPO2BK_EN_Msk | EPWM_PWMCON_BK0_EN_Msk)        /*!< Comparator 2 as brake edge-detector 0 source */

#define EPWM_FB1_EDGE_BKP1     (EPWM_PWMCON_BK1_EN_Msk)                                    /*!< BKP1 pin as brake edge-detector 1 source */
#define EPWM_FB1_EDGE_ACMP0    ((1UL << EPWM_PWMCON_BK1SEL_Pos) | EPWM_PWMCON_BK1_EN_Msk)  /*!< Comparator 0 as brake edge-detector 1 source */
#define EPWM_FB1_EDGE_ACMP1    ((2UL << EPWM_PWMCON_BK1SEL_Pos) | EPWM_PWMCON_BK1_EN_Msk)  /*!< Comparator 1 as brake edge-detector 1 source */
#define EPWM_FB1_EDGE_ACMP2    ((3UL << EPWM_PWMCON_BK1SEL_Pos) | EPWM_PWMCON_BK1_EN_Msk)  /*!< Comparator 2 as brake edge-detector 1 source */

#define EPWM_FB1_LEVEL_BKP1    (EPWM_FB1_EDGE_BKP1 | EPWM_PWMCON_LVDBK_EN_Msk)            /*!< BKP1 pin as low-level brake detector 1 source */
#define EPWM_FB1_LEVEL_ACMP0   (EPWM_FB1_EDGE_ACMP0 | EPWM_PWMCON_LVDBK_EN_Msk)           /*!< Comparator 0 as low-level brake detector 1 source */
#define EPWM_FB1_LEVEL_ACMP1   (EPWM_FB1_EDGE_ACMP1 | EPWM_PWMCON_LVDBK_EN_Msk)           /*!< Comparator 1 as low-level brake detector 1 source */
#define EPWM_FB1_LEVEL_ACMP2   (EPWM_FB1_EDGE_ACMP2 | EPWM_PWMCON_LVDBK_EN_Msk)           /*!< Comparator 2 as low-level brake detector 1 source */

/*---------------------------------------------------------------------------------------------------------*/
/*  Noise Filter Clock Divide Select Constant Definitions                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_NF_CLK_DIV_1                (0UL)    /*!< Noise filter clock is HCLK divide by 1 */
#define EPWM_NF_CLK_DIV_2                (1UL)    /*!< Noise filter clock is HCLK divide by 2 */
#define EPWM_NF_CLK_DIV_4                (2UL)    /*!< Noise filter clock is HCLK divide by 4 */
#define EPWM_NF_CLK_DIV_16               (3UL)    /*!< Noise filter clock is HCLK divide by 16 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Fail Brake Interrupt Flag Constant Definitions                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_IF_BKLK0           (EPWM_PWMSTS_BKLK0_Msk)   /*!< Interrupt flag of brake detector 0 state is locked */
#define EPWM_IF_BK0             (EPWM_PWMSTS_BKF0_Msk)    /*!< Interrupt flag of brake detector 0 detects a falling signal */
#define EPWM_IF_BK1             (EPWM_PWMSTS_BKF1_Msk)    /*!< Interrupt flag of brake detector 1 detects a falling signal */

/*---------------------------------------------------------------------------------------------------------*/
/*  Load Mode Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_LOAD_MODE_AUTOLOAD          (EPWM_PWMCON_AUTOLD_Msk)    /*!< EPWM auto load mode */
#define EPWM_LOAD_MODE_CENTER            (EPWM_PWMCON_CLDMD_Msk)     /*!< EPWM center load mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Group channel number constants definitions                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_CH0                         0x0                         /*!< EPWM Group 0/1 channel 0 */
#define EPWM_CH1                         0x1                         /*!< EPWM Group 0/1 channel 1 */
#define EPWM_CH2                         0x2                         /*!< EPWM Group 0/1 channel 2 */
#define EPWM_CH3                         0x3                         /*!< EPWM Group 0/1 channel 3 */
#define EPWM_CH4                         0x4                         /*!< EPWM Group 0/1 channel 4 */
#define EPWM_CH5                         0x5                         /*!< EPWM Group 0/1 channel 5 */

/*@}*/ /* end of group EPWM_EXPORTED_CONSTANTS */


/** @addtogroup EPWM_EXPORTED_FUNCTIONS EPWM Exported Functions
  @{
*/
/**
 * @brief This macro enable complementary mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @return None.
 * \hideinitializer
 */
#define EPWM_ENABLE_COMPLEMENTARY_MODE(epwm) ((epwm)->PWMCON = ((epwm)->PWMCON & ~EPWM_PWMCON_PWMMOD_Msk) | EPWM_COMPLEMENTARY_MODE)

/**
 * @brief This macro disable complementary mode, and enable independent mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @return None.
 * \hideinitializer
 */
#define EPWM_DISABLE_COMPLEMENTARY_MODE(epwm) ((epwm)->PWMCON &= ~EPWM_PWMCON_PWMMOD_Msk)

/**
 * @brief This macro enable group mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @return None.
 * \hideinitializer
 */
#define EPWM_ENABLE_GROUP_MODE(epwm) ((epwm)->PWMCON |= EPWM_PWMCON_GRP_Msk)

/**
 * @brief This macro disable group mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @return None.
 * \hideinitializer
 */
#define EPWM_DISABLE_GROUP_MODE(epwm) ((epwm)->PWMCON &= ~EPWM_PWMCON_GRP_Msk)

/**
 * @brief This macro enable synchronous mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @return None.
 * \hideinitializer
 */
#define EPWM_ENABLE_SYNC_MODE(epwm) ((epwm)->PWMCON = ((epwm)->PWMCON & ~EPWM_PWMCON_PWMMOD_Msk) | EPWM_SYNCHRONIZED_MODE)

/**
 * @brief This macro disable synchronous mode, and enable independent mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @return None.
 * \hideinitializer
 */
#define EPWM_DISABLE_SYNC_MODE(epwm) ((epwm)->PWMCON &= ~EPWM_PWMCON_PWMMOD_Msk)

/**
 * @brief Enable output inverter for all channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Valid value is 0 and 1.\n
 *                           0 represents disable and 1 represents enable output inverter function.
 * @return None.
 * @details This macro is used to enable output inverter for all channels.\n
 *          When enable this function the EPWM comparator output signals will be inverted.
 * \hideinitializer
 */
#define EPWM_ENABLE_OUTPUT_INVERTER(epwm, u32ChannelMask) ((epwm)->PWMCON = ((epwm)->PWMCON & ~EPWM_PWMCON_PWMINV_Msk) | (u32ChannelMask) << EPWM_PWMCON_PWMINV_Pos)

/**
 * @brief This macro mask output output logic to high or low.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32LevelMask Output logic to high or low.
 * @return None.
 * @details This macro is used to mask output logic to high or low of specified channel(s).
 * @note If u32ChannelMask parameter is 0, then mask function will be disabled.
 * \hideinitializer
 */
#define EPWM_MASK_OUTPUT(epwm, u32ChannelMask, u32LevelMask) \
    { \
        (epwm)->PMSKE = (u32ChannelMask); \
        (epwm)->PMSKD = (u32LevelMask); \
    }

/**
 * @brief Set the divider of the selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref EPWM_CLK_DIV_1
 *              - \ref EPWM_CLK_DIV_2
 *              - \ref EPWM_CLK_DIV_4
 *              - \ref EPWM_CLK_DIV_16
 * @return None.
 * @details This macro is used to set Timer clock source divider selection for specified channel.
 * \hideinitializer
 */
#define EPWM_SET_DIVIDER(epwm, u32ChannelNum, u32Divider) ((epwm)->PWMCON = ((epwm)->PWMCON & ~(EPWM_PWMCON_PWMDIV_Msk)) | (u32Divider))

/**
 * @brief Set the duty of the selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF.
 * @return None.
 * @details This macro is used to set EPWM comparator value for specified channel.
 * @note This new setting will take effect on next EPWM period.
 * \hideinitializer
 */
#define EPWM_SET_CMR(epwm, u32ChannelNum, u32CMR) (*((__IO uint32_t *) ((((uint32_t)&((epwm)->PWM0)) + ((u32ChannelNum) >> 1) * 4))) = (u32CMR))

/**
 * @brief Set the period of the selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @param[in] u32CNR Period of specified channel. Valid values are between 0~0xFFFF
 * @return None.
 * @details This macro is used to set timer loaded value(CNR) for specified channel.\n
 *          Loaded value determines the EPWM period.
 * @note This new setting will take effect on next EPWM period.
 * @note EPWM counter will stop if period length set to 0.
 * \hideinitializer
 */
#define EPWM_SET_CNR(epwm, u32ChannelNum, u32CNR) ((epwm)->PWMP = (u32CNR))

/**
 * @brief Set the EPWM aligned type.
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. This parameter is not used.
 * @param[in] u32AlignedType EPWM aligned type, valid values are:
 *                  - \ref EPWM_EDGE_ALIGNED
 *                  - \ref EPWM_CENTER_ALIGNED
 * @return None
 * @details This macro is used to set the EPWM aligned type.
  * \hideinitializer
 */
#define EPWM_SET_ALIGNED_TYPE(epwm, u32ChannelMask, u32AlignedType) ((epwm)->PWMCON = ((epwm)->PWMCON & ~(EPWM_PWMCON_PWMTYPE_Msk)) | (u32AlignedType))

/**
 * @brief Clear counter of all channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. This parameter is not used.
 * @return None.
 * @details This macro is used to clear counter of all channels.
 * \hideinitializer
 */
#define EPWM_CLR_COUNTER(epwm, u32ChannelMask) ((epwm)->PWMCON = ((epwm)->PWMCON & ~(EPWM_PWMCON_LOAD_Msk)) | (EPWM_PWMCON_CLRPWM_Msk))


uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequncy, uint32_t u32DutyCycle);
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableEdgeInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntEdgeType);
void EPWM_DisableEdgeInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearEdgeIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetEdgeIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAcc(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntFlagCnt, uint32_t u32IntAccSrc);
void EPWM_DisableAcc(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void EPWM_DisableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void EPWM_EnableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void EPWM_DisableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_EnableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_DisableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum);


/*@}*/ /* end of group EPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group EPWM_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__EPWM_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
