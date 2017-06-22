/**************************************************************************//**
 * @file     epwm.h
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series EPWM driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __EPWM_H__
#define __EPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_EPWM_Driver EPWM Driver
  @{
*/

/** @addtogroup Mini57_EPWM_EXPORTED_CONSTANTS EPWM Exported Constants
  @{
*/
#define EPWM_CHANNEL_NUM                     (6)        /*!< EPWM channel number */
#define EPWM_CH_0_MASK                       (0x1UL)    /*!< EPWM channel 0 mask */
#define EPWM_CH_1_MASK                       (0x2UL)    /*!< EPWM channel 1 mask */
#define EPWM_CH_2_MASK                       (0x4UL)    /*!< EPWM channel 2 mask */
#define EPWM_CH_3_MASK                       (0x8UL)    /*!< EPWM channel 3 mask */
#define EPWM_CH_4_MASK                       (0x10UL)   /*!< EPWM channel 4 mask */
#define EPWM_CH_5_MASK                       (0x20UL)   /*!< EPWM channel 5 mask */

/*---------------------------------------------------------------------------------------------------------*/
/*  Clock Divider Constant Definitions                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_CLK_DIV_1                       (0UL)    /*!< EPWM clock divide by 1 */
#define EPWM_CLK_DIV_2                       (1UL)    /*!< EPWM clock divide by 2 */
#define EPWM_CLK_DIV_4                       (2UL)    /*!< EPWM clock divide by 4 */
#define EPWM_CLK_DIV_8                       (3UL)    /*!< EPWM clock divide by 8 */
#define EPWM_CLK_DIV_16                      (4UL)    /*!< EPWM clock divide by 16 */
#define EPWM_CLK_DIV_32                      (5UL)    /*!< EPWM clock divide by 32 */
#define EPWM_CLK_DIV_64                      (6UL)    /*!< EPWM clock divide by 64 */
#define EPWM_CLK_DIV_128                     (7UL)    /*!< EPWM clock divide by 128 */
#define EPWM_CLK_DIV_256                     (8UL)    /*!< EPWM clock divide by 256 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Aligned Type Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_EDGE_ALIGNED                    (0UL)                       /*!< EPWM working in edge aligned type(down count) */
#define EPWM_CENTER_ALIGNED                  (EPWM_CTL_CNTTYPE_Msk)      /*!< EPWM working in center aligned type */

/*---------------------------------------------------------------------------------------------------------*/
/*  Fail brake Control Constant Definitions                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_BRK0_BRKP0                      (EPWM_BRKCTL_BRK0PEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)       /*!< Brake0 signal source from external pin */
#define EPWM_BRK0_ACMP0                      (EPWM_BRKCTL_BRK0A0EN_Msk | EPWM_BRKCTL_BRK0EN_Msk)      /*!< Brake0 signal source from analog comparator 0 output */
#define EPWM_BRK0_ACMP1                      (EPWM_BRKCTL_BRK0A1EN_Msk | EPWM_BRKCTL_BRK0EN_Msk)      /*!< Brake0 signal source from analog comparator 1 output */
#define EPWM_BRK0_EADC                       (EPWM_BRKCTL_BK0ADCEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)      /*!< Brake0 signal source from analog EADC output */
#define EPWM_BRK1_BRKP1                      (EPWM_BRKCTL_BRK1PEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)       /*!< Brake1 signal source from external pin */
#define EPWM_BRK1_ACMP0                      (EPWM_BRKCTL_BRK1A0EN_Msk | EPWM_BRKCTL_BRK1EN_Msk)      /*!< Brake1 signal source from analog comparator 0 output */
#define EPWM_BRK1_ACMP1                      (EPWM_BRKCTL_BRK1A1EN_Msk | EPWM_BRKCTL_BRK1EN_Msk)      /*!< Brake1 signal source from analog comparator 1 output */
#define EPWM_BRK1_EADC                       (EPWM_BRKCTL_BK1ADCEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)      /*!< Brake1 signal source from analog EADC output */
#define EPWM_BRK1_LVDBK                      (EPWM_BRKCTL_LVDBKEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)       /*!< Brake1 signal source from level detect */

/*---------------------------------------------------------------------------------------------------------*/
/*  Duty Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_DUTY_INT_DOWN_COUNT_MATCH_CMP   (EPWM_INTEN_CMPDIEN0_Msk)   /*!< EPWM duty interrupt triggered if down count match comparator */
#define EPWM_DUTY_INT_UP_COUNT_MATCH_CMP     (EPWM_INTEN_CMPUIEN0_Msk)   /*!< EPWM duty interrupt triggered if up down match comparator */


/*@}*/ /* end of group Mini57_EPWM_EXPORTED_CONSTANTS */


/** @addtogroup Mini57_EPWM_EXPORTED_FUNCTIONS EPWM Exported Functions
  @{
*/

/**
 * @brief This macro enable complementary mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable complementary mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_ENABLE_COMPLEMENTARY_MODE(epwm) (epwm->CTL = (epwm->CTL & ~EPWM_CTL_MODE_Msk) |(1UL << EPWM_CTL_MODE_Pos))

/**
 * @brief This macro disable complementary mode, and enable independent mode.
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable complementary mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_DISABLE_COMPLEMENTARY_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_MODE_Msk)

/**
 * @brief This macro enable group mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable group mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_ENABLE_GROUP_MODE(epwm) (epwm->CTL |= EPWM_CTL_GROUPEN_Msk)

/**
 * @brief This macro disable group mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable group mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_DISABLE_GROUP_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_GROUPEN_Msk)

/**
 * @brief This macro enable synchronous mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable synchronous mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_ENABLE_SYNC_MODE(epwm) (epwm->CTL = (epwm->CTL & ~EPWM_CTL_MODE_Msk) |(2UL << EPWM_CTL_MODE_Pos))

/**
 * @brief This macro disable synchronous mode, and enable independent mode.
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable synchronous mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_DISABLE_SYNC_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_MODE_Msk)

/**
 * @brief This macro enable output inverter of specified channel(s)
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * @details This macro is used to enable output inverter of specified channel(s).
 * \hideinitializer
 */
#define EPWM_ENABLE_OUTPUT_INVERTER(epwm, u32ChannelMask) ((epwm)->NPCTL = (u32ChannelMask))

/**
 * @brief This macro mask output logic to high or low
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32LevelMask Output logic to high or low
 * @return None
 * @details This macro is used to mask output logic to high or low of specified channel(s).
 * @note If u32ChannelMask parameter is 0, then mask function will be disabled.
 * \hideinitializer
 */
#define EPWM_MASK_OUTPUT(epwm, u32ChannelMask, u32LevelMask) \
    { \
        (epwm)->PHCHG = ((epwm)->PHCHG & ~(0x3F << EPWM_PHCHG_MSKEN0_Pos)) | (u32ChannelMask << EPWM_PHCHG_MSKEN0_Pos); \
        (epwm)->PHCHG = ((epwm)->PHCHG & ~(0x3F << EPWM_PHCHG_MSKDAT0_Pos)) | (u32LevelMask << EPWM_PHCHG_MSKDAT0_Pos); \
    }

/**
 * @brief This macro set the divider of the EPWM channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref EPWM_CLK_DIV_1
 *              - \ref EPWM_CLK_DIV_2
 *              - \ref EPWM_CLK_DIV_4
 *              - \ref EPWM_CLK_DIV_8
 *              - \ref EPWM_CLK_DIV_16
 *              - \ref EPWM_CLK_DIV_32
 *              - \ref EPWM_CLK_DIV_64
 *              - \ref EPWM_CLK_DIV_128
 *              - \ref EPWM_CLK_DIV_256
 * @return None
 * @details This macro is used to set the divider of the EPWM channel.
 * \hideinitializer
 */
#define EPWM_SET_DIVIDER(epwm, u32ChannelNum, u32Divider) \
    (epwm->CLKDIV = (epwm->CLKDIV & ~(EPWM_CLKDIV_CLKDIV_Msk)) | (u32Divider))

/**
 * @brief This macro set the comparator of the selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @param[in] u32CMR Comparator of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set the comparator of specified channel.
 * @note This new setting will take effect on next EPWM period.
 * \hideinitializer
 */
#define EPWM_SET_CMR(epwm, u32ChannelNum, u32CMR) ((epwm)->CMPDAT[(u32ChannelNum)]= (u32CMR))

/**
 * @brief This macro set the period of the EPWM channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used
 * @param[in] u32CNR Period of EPWM channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set the period of EPWM channel.
 * @note This new setting will take effect on next EPWM period.
 * @note EPWM counter will stop if period length set to 0.
 * \hideinitializer
 */
#define EPWM_SET_CNR(epwm, u32ChannelNum, u32CNR)  ((epwm)->PERIOD = (u32CNR))

/**
 * @brief This macro set the EPWM aligned type
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask This parameter is not used
 * @param[in] u32AlignedType EPWM aligned type, valid values are:
 *              - \ref EPWM_EDGE_ALIGNED
 *              - \ref EPWM_CENTER_ALIGNED
 * @return None
 * @details This macro is used to set the EPWM aligned type.
 * \hideinitializer
 */
#define EPWM_SET_ALIGNED_TYPE(epwm, u32ChannelMask, u32AlignedType) \
   (epwm->CTL = (epwm->CTL & ~EPWM_CTL_CNTTYPE_Msk) | (u32AlignedType))


/*---------------------------------------------------------------------------------------------------------*/
/* Define EPWM functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void EPWM_DisableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);


/*@}*/ /* end of group Mini57_EPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_EPWM_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
