/**************************************************************************//**
 * @file     epwm.c
 * @version  V3.00
 * $Revision: 13 $
 * $Date: 15/04/01 7:32p $
 * @brief    EPWM driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "M0519.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup EPWM_Driver EPWM Driver
  @{
*/


/** @addtogroup EPWM_EXPORTED_FUNCTIONS EPWM Exported Functions
  @{
*/

/**
 * @brief Configure EPWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @param[in] u32Frequency Target generator frequency.
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second.
 * @details This function is used to configure EPWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @note Since all channels shares a divider. Call this API to configure EPWM frequency may affect existing frequency of other channel.
 */
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t u32PWMClockSrc;
    uint32_t i;
    uint8_t  u8Divider = 1;
    /* this table is mapping divider value to register configuration */
    uint32_t u32PWMDividerToRegTbl[17] = {NULL, 0, 1, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
    uint16_t u16CNR = 0xFFFF;

    SystemCoreClockUpdate();
    u32PWMClockSrc = SystemCoreClock;

    for(; u8Divider < 17; u8Divider <<= 1)
    {
        if(u8Divider == 8) /* clk divider could only be 1, 2, 4, 16 */
            continue;

        i = (u32PWMClockSrc / u32Frequency) / u8Divider;
        /* If target value is larger than CNR, need to use a larger divider */
        if(i > 0x10000)
            continue;

        if(i == 1)
            u16CNR = 1; /* Too fast, and PWM cannot generate expected frequency. */
        else
            u16CNR = i;
        break;
    }
    /* Store return value here 'cos we're gonna change u8Divider & u16CNR to the real value to fill into register */
    i = u32PWMClockSrc / (u8Divider * u16CNR);

    u16CNR -= 1;
    /* convert to real register value */
    u8Divider = u32PWMDividerToRegTbl[u8Divider];
    (epwm)->PWMCON = (epwm)->PWMCON & ~EPWM_PWMCON_PWMDIV_Msk | (u8Divider << EPWM_PWMCON_PWMDIV_Pos);
    /* set EPWM to edge aligned type */
    (epwm)->PWMCON = (epwm)->PWMCON & ~EPWM_PWMCON_PWMTYPE_Msk | EPWM_EDGE_ALIGNED;
    if(u32DutyCycle)
    {
        *((__IO uint32_t *)((((uint32_t) & ((epwm)->PWM0)) + ((u32ChannelNum) >> 1) * 4))) = u32DutyCycle * (u16CNR + 1) / 100 - 1;
    }
    else
    {
        *((__IO uint32_t *)((((uint32_t) & ((epwm)->PWM0)) + ((u32ChannelNum >> 1)) * 4))) = 0;
    }
    (epwm)->PWMP = u16CNR;

    return(i);
}

/**
 * @brief Start EPWM module.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. This parameter is not used.
 * @return None.
 * @details This function is used to start EPWM module.
 */
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    (epwm)->PWMCON |= EPWM_PWMCON_PWMRUN_Msk;
}

/**
 * @brief Stop EPWM module.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. This parameter is not used.
 * @return None.
 * @details This function is used to stop EPWM module.
 */
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    (epwm)->PWMP = 0;
}

/**
 * @brief Stop EPWM generation immediately by clear channel enable bit.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. This parameter is not used.
 * @return None.
 * @details This function is used to stop EPWM generation immediately by clear channel enable bit.
 */
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    (epwm)->PWMCON &= ~EPWM_PWMCON_PWMRUN_Msk;
}

/**
 * @brief This function enables fault brake.
 * @param[in] epwm The base address of EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels.\n
 *                           Valid values are 0 and 1. 0 represents disable and 1 represents enable brake function.
 * @param[in] u32LevelMask Output high or low while fault brake occurs, each bit represent the level of a channel
 *                         while fault brake occur. Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32BrakeSource Fault brake source, could be one of following source
 *                  - \ref EPWM_FB0_EDGE_BKP0
 *                  - \ref EPWM_FB0_EDGE_ACMP0
 *                  - \ref EPWM_FB0_EDGE_ACMP1
 *                  - \ref EPWM_FB0_EDGE_ACMP2
 *                  - \ref EPWM_FB1_EDGE_BKP1
 *                  - \ref EPWM_FB1_EDGE_ACMP0
 *                  - \ref EPWM_FB1_EDGE_ACMP1
 *                  - \ref EPWM_FB1_EDGE_ACMP2
 *                  - \ref EPWM_FB1_LEVEL_BKP1
 *                  - \ref EPWM_FB1_LEVEL_ACMP0
 *                  - \ref EPWM_FB1_LEVEL_ACMP1
 *                  - \ref EPWM_FB1_LEVEL_ACMP2
 * @return None.
 */
void EPWM_EnableFaultBrake(EPWM_T *epwm,
                           uint32_t u32ChannelMask,
                           uint32_t u32LevelMask,
                           uint32_t u32BrakeSource)
{
    if(u32ChannelMask)
    {
        switch(u32BrakeSource)
        {
            case EPWM_FB0_EDGE_BKP0:
            case EPWM_FB0_EDGE_ACMP0:
            case EPWM_FB0_EDGE_ACMP1:
            case EPWM_FB0_EDGE_ACMP2:
                (epwm)->PWMCON |= u32BrakeSource;
                break;
            case EPWM_FB1_EDGE_BKP1:
            case EPWM_FB1_EDGE_ACMP0:
            case EPWM_FB1_EDGE_ACMP1:
            case EPWM_FB1_EDGE_ACMP2:
            case EPWM_FB1_LEVEL_BKP1:
            case EPWM_FB1_LEVEL_ACMP0:
            case EPWM_FB1_LEVEL_ACMP1:
            case EPWM_FB1_LEVEL_ACMP2:
                (epwm)->PWMCON = ((epwm)->PWMCON & ~(EPWM_PWMCON_BK1SEL_Msk)) | u32BrakeSource;
                break;
            default:
                ;
        }
    }
    else
    {
        (epwm)->PWMCON &= ~(u32BrakeSource);
    }
    (epwm)->PWMB = u32LevelMask;
}

/**
 * @brief Enables EPWM output generation of even and odd channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels.
 *                           Set bit 0 to 1 enables even channels output, set bit 1 to 1 enables odd channels output.
 * @return None.
 * @details This function is used to enables EPWM output generation of even and odd channels.
 */
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    if(epwm == EPWM0)
    {
        GPIO->PWMPOEN &= ~(u32ChannelMask);
    }
    else if(epwm == EPWM1)
    {
        GPIO->PWMPOEN &= ~((u32ChannelMask) << GPIO_PWMPOEN_HZ_Even1_Pos);
    }
}

/**
 * @brief Disables EPWM output generation of even and odd channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels.
 *                           Set bit 0 to 1 disables even channels output, set bit 1 to 1 disables odd channels output.
 * @return None.
 * @details This function is used to disables EPWM output generation of even and odd channels.
 * @note If disable EPWM output then the driving mode of EPWM ports are forced in tri-state (Hi-Z) all the time.
 */
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    if(epwm == EPWM0)
    {
        GPIO->PWMPOEN |= u32ChannelMask;
    }
    else if(epwm == EPWM1)
    {
        GPIO->PWMPOEN |= ((u32ChannelMask) << GPIO_PWMPOEN_HZ_Even1_Pos);
    }
}

/**
 * @brief Enable Dead zone of selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @param[in] u32Duration Dead Zone length in EPWM clock count, valid values are between 0~0xFF, but 0 means there is no
 *                        dead zone.
 * @return None.
 * @details This function is used to enable Dead zone of selected channel.
 */
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    u32ChannelNum >>= 1;
    /* set duration */
    (epwm)->PDTC = ((epwm)->PDTC & ~(EPWM_PDTC_DTCNT_Msk)) | (u32Duration << (EPWM_PDTC_DTCNT_Pos));
    /* enable dead zone */
    (epwm)->PDTC |= (EPWM_PDTC_DTEN0_Msk << u32ChannelNum);
}

/**
 * @brief Disable Dead zone of selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @return None.
 * @details This function is used to disable Dead zone of selected channel.
 */
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    u32ChannelNum >>= 1;
    /* enable dead zone */
    (epwm)->PDTC &= ~(EPWM_PDTC_DTEN0_Msk << u32ChannelNum);
}

/**
 * @brief Enable edge interrupt of selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @param[in] u32IntEdgeType Edge interrupt type, could be either
 *              - \ref EPWM_EDGE_INT_FALLING
 *              - \ref EPWM_EDGE_INT_RISING
 * @return None.
 * @details This function is used to enable edge interrupt of selected channel with selected interrupt type.
 */
void EPWM_EnableEdgeInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntEdgeType)
{
    u32ChannelNum >>= 1;
    (epwm)->PWMEIC = ((epwm)->PWMEIC & ~(EPWM_PWMEIC_EINT0_TYPE_Msk << u32ChannelNum)) | ((u32IntEdgeType | EPWM_PWMEIC_PWM0EI_EN_Msk) << u32ChannelNum);
}

/**
 * @brief Disable edge interrupt of selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @return None.
 * @details This function is used to disable edge interrupt of selected channel.
 */
void EPWM_DisableEdgeInt(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    u32ChannelNum >>= 1;
    (epwm)->PWMEIC &= ~(EPWM_PWMEIC_PWM0EI_EN_Msk << u32ChannelNum);
}

/**
 * @brief Clear edge interrupt flag of selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @return None.
 * @details This function is used to clear edge interrupt flag of selected channel.
 */
void EPWM_ClearEdgeIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    u32ChannelNum >>= 1;
    (epwm)->PWMSTS = EPWM_PWMSTS_PWM0EF_Msk << u32ChannelNum;
}

/**
 * @brief Get edge interrupt flag of selected channel.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. Valid values are 0, 2 and 4.
 * @retval 0 Edge interrupt did not occur.
 * @retval 1 Edge interrupt occurred.
 * @details This function is used to get edge interrupt flag of selected channel.
 */
uint32_t EPWM_GetEdgeIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    u32ChannelNum >>= 1;
    return (((epwm)->PWMSTS & (EPWM_PWMSTS_PWM0EF_Msk << u32ChannelNum)) ? 1 : 0);
}

/**
 * @brief This function enable fault brake interrupt.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakeSource Fault brake source. This parameter is not used.
 * @return None.
 * @details This function is used to enable fault brake interrupt.
 */
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    (epwm)->PWMCON |= (EPWM_PWMCON_BRKI_EN_Msk);
}

/**
 * @brief This function disable fault brake interrupt.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakeSource Fault brake source. This parameter is not used.
 * @return None.
 * @details This function is used to disable fault brake interrupt.
 */
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    (epwm)->PWMCON &= ~(EPWM_PWMCON_BRKI_EN_Msk);
}

/**
 * @brief This function clear fault brake interrupt of selected source.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref EPWM_IF_BKLK0
 *              - \ref EPWM_IF_BK0
 *              - \ref EPWM_IF_BK1
 * @return None.
 * @details This function is used to clear fault brake interrupt of selected source.
 */
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    (epwm)->PWMSTS = (u32BrakeSource);
}

/**
 * @brief This function get fault brake interrupt flag of selected source.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakeSource Fault brake source, could be either
 *              - \ref EPWM_IF_BKLK0
 *              - \ref EPWM_IF_BK0
 *              - \ref EPWM_IF_BK1
 * @return Fault brake interrupt flag of specified source.
 * @retval 0 Fault brake interrupt did not occurred
 * @retval 1 Fault brake interrupt occurred
 * @details This function is used to get fault brake interrupt flag of selected source.
 */
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    return (((epwm)->PWMSTS & (u32BrakeSource)) ? 1 : 0);
}

/**
 * @brief Enable period interrupt for all channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @param[in] u32IntPeriodType Period interrupt type, could be either
 *              - \ref EPWM_PERIOD_INT_UNDERFLOW
 *              - \ref EPWM_PERIOD_INT_MATCH_CNR
 * @return None.
 * @details This function is used to enable period interrupt for all channels.
 * @note Period interrupt type is effective when EPWM is in Center-aligned mode only.
 */
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType)
{
    (epwm)->PWMCON = ((epwm)->PWMCON & ~(EPWM_PWMCON_INT_TYPE_Msk)) | EPWM_PWMCON_PWMI_EN_Msk | (u32IntPeriodType);
}

/**
 * @brief Disable period interrupt for all channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @return None.
 * @details This function is used to disable period interrupt for all channels.
 */
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->PWMCON &= ~(EPWM_PWMCON_PWMI_EN_Msk);
}

/**
 * @brief Clear period interrupt flag.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @return None.
 * @details This function is used to clear period interrupt flag.
 */
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->PWMSTS = (EPWM_PWMSTS_PWMF_Msk);
}

/**
 * @brief Get period interrupt flag.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @retval 0 Period interrupt did not occur.
 * @retval 1 Period interrupt occurred.
 * @details This function is used to get period interrupt flag.
 */
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    return (((epwm)->PWMSTS & (EPWM_PWMSTS_PWMF_Msk)) ? 1 : 0);
}

/**
 * @brief Enable period interrupt flag accumulator.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @param[in] u32IntFlagCnt Interrupt flag counter. Valid values are between 0~15.
 * @param[in] u32IntAccSrc Interrupt flag accumulator source selection. This parameter is not used.
 * @return None.
 * @details This function is used to enable period interrupt flag accumulator.
 */
void EPWM_EnableAcc(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntFlagCnt, uint32_t u32IntAccSrc)
{
    (epwm)->PWMFCNT = (u32IntFlagCnt);
}

/**
 * @brief Disable period interrupt flag accumulator.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @return None.
 * @details This function is used to disable period interrupt flag accumulator.
 */
void EPWM_DisableAcc(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->PWMFCNT = 0;
}

/**
 * @brief Enable load mode for all channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @param[in] u32LoadMode EPWM counter loading mode.
 *              - \ref EPWM_LOAD_MODE_AUTOLOAD
 *              - \ref EPWM_LOAD_MODE_CENTER
 * @return None.
 * @details This function is used to enable load mode for all channels.
 */
void EPWM_EnableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode)
{
    (epwm)->PWMCON |= (u32LoadMode);
}

/**
 * @brief Disable load mode for all channels.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32ChannelNum EPWM channel number. This parameter is not used.
 * @param[in] u32LoadMode EPWM counter loading mode.
 *              - \ref EPWM_LOAD_MODE_AUTOLOAD
 *              - \ref EPWM_LOAD_MODE_CENTER
 * @return None.
 * @details This function is used to disable load mode for all channels.
 */
void EPWM_DisableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode)
{
    (epwm)->PWMCON &= ~(u32LoadMode);
}

/**
 * @brief Enable EPWM brake noise filter function.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @param[in] u32ClkCnt Brake edge detector filter count. This parameter is not used.
 * @param[in] u32ClkDivSel Brake edge detector filter clock selection.
 *              - \ref EPWM_NF_CLK_DIV_1
 *              - \ref EPWM_NF_CLK_DIV_2
 *              - \ref EPWM_NF_CLK_DIV_4
 *              - \ref EPWM_NF_CLK_DIV_16
 * @return None.
 * @details This function is used to enable EPWM brake noise filter function.
 */
void EPWM_EnableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel)
{
    (epwm)->PWMCON = ((epwm)->PWMCON & ~((EPWM_PWMCON_BK0NF_DIS_Msk << u32BrakePinNum) | (EPWM_PWMCON_BK0FILT_Msk << (u32BrakePinNum << 1)))) | \
                     ((u32ClkDivSel << EPWM_PWMCON_BK0FILT_Pos) << (u32BrakePinNum << 1));
}

/**
 * @brief Disable EPWM brake noise filter function.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @return None.
 * @details This function is used to disable EPWM brake noise filter function.
 */
void EPWM_DisableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum)
{
    (epwm)->PWMCON |= (EPWM_PWMCON_BK0NF_DIS_Msk << (u32BrakePinNum));
}

/**
 * @brief Enable EPWM brake pin inverse function.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @return None.
 * @details This function is used to enable EPWM brake pin inverse function.
 */
void EPWM_EnableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum)
{
    (epwm)->PWMCON |= (EPWM_PWMCON_INVBKP0_Msk << (u32BrakePinNum));
}

/**
 * @brief Disable EPWM brake pin inverse function.
 * @param[in] epwm The pointer of the specified EPWM module.
 * @param[in] u32BrakePinNum Brake pin selection. Valid values are 0 or 1.
 * @return None.
 * @details This function is used to disable EPWM brake pin inverse function.
 */
void EPWM_DisableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum)
{
    (epwm)->PWMCON &= ~(EPWM_PWMCON_INVBKP0_Msk << (u32BrakePinNum));
}

/*@}*/ /* end of group EPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group EPWM_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
