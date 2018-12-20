/**************************************************************************//**
 * @file     epwm.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series EPWM driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini57Series.h"

/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_EPWM_Driver EPWM Driver
  @{
*/


/** @addtogroup Mini57_EPWM_EXPORTED_FUNCTIONS EPWM Exported Functions
  @{
*/

/**
 * @brief This function Configure EPWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 */
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle)
{
    uint32_t i = SystemCoreClock / u32Frequency, j = 0;
    uint16_t  u16Divider = 1;
    uint16_t u16CNR = 0xFFFF;

    for(; u16Divider < 512; u16Divider <<= 1, j++)    /* clk divider could only be 1, 2, 4, 8, 16, 32, 64, 128, 256 */
    {
        i = (SystemCoreClock / u32Frequency) / u16Divider;
        /* If target value is larger than CNR , need to use a larger divider */
        if(i > (0x10000))
            continue;

        if(i <= 0x10000)
        {
            if(i == 1)
                u16CNR = 1;     /* Too fast, and PWM cannot generate expected frequency... */
            else
                u16CNR = i;
            break;
        }

    }
    /* Store return value here 'cos we're gonna change u8Divider & u16CNR to the real value to fill into register */
    i = SystemCoreClock / (u16Divider * u16CNR);

    u16CNR -= 1;
    /* convert to real register value */
    u16Divider = j;

    epwm->CLKDIV = u16Divider;
    epwm->CTL = (epwm->CTL & ~EPWM_CTL_CNTTYPE_Msk) | (EPWM_CTL_CNTMODE_Msk);
    if(u32DutyCycle == 0)
        epwm->CMPDAT[u32ChannelNum] = 0;
    else
        epwm->CMPDAT[u32ChannelNum] = u32DutyCycle * (u16CNR + 1) / 100;
    epwm->PERIOD = u16CNR;

    return(i);
}

/**
 * @brief Start EPWM module
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to start EPWM module.
 */
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    (epwm)->CTL |= u32ChannelMask;
}

/**
 * @brief Stop EPWM module
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask This parameter is not used.
 * @return None
 * @details This function is used to stop EPWM module.
 */
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    (epwm)->PERIOD = 0;
}

/**
 * @brief Stop EPWM generation immediately by clear channel enable bit
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop EPWM generation immediately by clear channel enable bit.
 */
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    (epwm)->CTL &= ~u32ChannelMask;
}

/**
 * @brief This function enable fault brake of selected channel(s)
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask This parameter is not used.
 * @param[in] u32LevelMask Output high or low while fault brake occurs, each bit represent the level of a channel
 *                         while fault brake occurs. Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32BrakeSource Fault brake source, could be one of following source
 *                  - \ref EPWM_BRK0_BRKP0
 *                  - \ref EPWM_BRK0_ACMP0
 *                  - \ref EPWM_BRK0_ACMP1
 *                  - \ref EPWM_BRK0_EADC
 *                  - \ref EPWM_BRK1_BRKP1
 *                  - \ref EPWM_BRK1_ACMP0
 *                  - \ref EPWM_BRK1_ACMP1
 *                  - \ref EPWM_BRK1_EADC
 *                  - \ref EPWM_BRK1_LVDBK
 * @return None
 * @details This function is used to enable fault brake of selected channel(s).
 */
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource)
{
    epwm->BRKCTL = (u32LevelMask << EPWM_BRKCTL_BKOD0_Pos) | u32BrakeSource;
}

/**
 * @brief Enables EPWM output generation of selected channel(s)
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 * @details This function is used to enable EPWM output generation of selected channel(s).
 */
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    uint32_t field_len = 2;

    for (i=0; i<EPWM_CHANNEL_NUM; i++)
    {
        if (u32ChannelMask & (1 << i))
        {
            PA->MODE = (PA->MODE & ~(0x3 << (i*field_len))) | (GPIO_MODE_OUTPUT << (i*field_len));
        }
    }
}

/**
 * @brief Disables EPWM output generation of selected channel(s)
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 * @details This function is used to disable EPWM output generation of selected channel(s).
 */
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    uint32_t field_len = 2;

    for (i=0; i<EPWM_CHANNEL_NUM; i++)
    {
        if (u32ChannelMask & (1 << i))
        {
            PA->MODE = (PA->MODE & ~(0x3 << (i*field_len))) | (GPIO_MODE_INPUT << (i*field_len));
        }
    }
}

/**
 * @brief Enable Dead zone of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @param[in] u32Duration Dead zone length in EPWM clock count, valid values are between 0~0xFF, but 0 means there is no Dead zone.
 * @return None
 * @details This function is used to enable Dead zone of selected channel.
 * @note Every two channels share the same setting.
 */
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    /* every two channels shares the same setting */
    u32ChannelNum >>= 1;
    /* set duration */
    epwm->DTCTL = (epwm->DTCTL & ~(EPWM_DTCTL_DTCNT01_Msk << (8 * u32ChannelNum))) | (u32Duration << (8 * u32ChannelNum));
    /* enable dead zone */
    epwm->CTL |= (EPWM_CTL_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief Disable Dead zone of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable Dead zone of selected channel.
 */
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    /* every two channels shares the same setting */
    u32ChannelNum >>= 1;
    /* enable dead zone */
    epwm->CTL &= ~(EPWM_CTL_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief Enable duty interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @param[in] u32IntDutyType Duty interrupt type, could be either
 *              - \ref EPWM_DUTY_INT_DOWN_COUNT_MATCH_CMP
 *              - \ref EPWM_DUTY_INT_UP_COUNT_MATCH_CMP
 * @return None
 * @details This function is used to enable duty interrupt of selected channel.
 */
void EPWM_EnableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType)
{
    (epwm)->INTEN |= (u32IntDutyType << u32ChannelNum);
}

/**
 * @brief Disable duty interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to disable duty interrupt of selected channel.
 */
void EPWM_DisableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTEN &= ~((EPWM_DUTY_INT_DOWN_COUNT_MATCH_CMP | EPWM_DUTY_INT_UP_COUNT_MATCH_CMP) << u32ChannelNum);
}

/**
 * @brief Clear duty interrupt flag of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @return None
 * @details This function is used to clear duty interrupt flag of selected channel.
 */
void EPWM_ClearDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTSTS = (EPWM_INTSTS_CMPUIF0_Msk | EPWM_INTSTS_CMPDIF0_Msk) << u32ChannelNum;
}

/**
 * @brief Get duty interrupt flag of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @return Duty interrupt flag of specified channel
 * @retval 0 Duty interrupt did not occur
 * @retval 1 Duty interrupt occurred
 * @details This function is used to get duty interrupt flag of selected channel.
 */
uint32_t EPWM_GetDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    return ((((epwm)->INTSTS & ((EPWM_INTSTS_CMPDIF0_Msk | EPWM_INTSTS_CMPUIF0_Msk) << u32ChannelNum))) ? 1 : 0);
}

/**
 * @brief This function enable fault brake interrupt
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref EPWM_INTEN_BRK0IEN_Msk
 *              - \ref EPWM_INTEN_BRK1IEN_Msk
 * @return None
 * @details This function is used to enable fault brake interrupt.
 */
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    (epwm)->INTEN |= (u32BrakeSource);
}

/**
 * @brief This function disable fault brake interrupt
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref EPWM_INTEN_BRK0IEN_Msk
 *              - \ref EPWM_INTEN_BRK1IEN_Msk
 * @return None
 * @details This function is used to disable fault brake interrupt.
 */
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    (epwm)->INTEN &= ~(u32BrakeSource);
}

/**
 * @brief This function clear fault brake interrupt of selected source
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32BrakeSource Fault brake source.
 *              - \ref EPWM_INTSTS_BRK0IF_Msk
 *              - \ref EPWM_INTSTS_BRK1IF_Msk
 * @return None
 * @details This function is used to clear fault brake interrupt of selected source.
 */
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    (epwm)->INTSTS = (u32BrakeSource);
}

/**
 * @brief This function get fault brake interrupt flag of selected source
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32BrakeSource Fault brake source, could be either
 *              - \ref EPWM_INTSTS_BRK0IF_Msk
 *              - \ref EPWM_INTSTS_BRK1IF_Msk
 * @return Fault brake interrupt flag of specified source
 * @retval 0 Fault brake interrupt did not occurred
 * @retval 1 Fault brake interrupt occurred
 * @details This function is used to get fault brake interrupt flag of selected source.
 */
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource)
{
    return (((epwm)->INTSTS & (u32BrakeSource)) ? 1 : 0);
}

/**
 * @brief Enable period interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @param[in] u32IntPeriodType Period interrupt type. This parameter is not used.
 * @return None
 * @details This function is used to enable period interrupt of selected channel.
 */
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
    (epwm)->INTEN |= (EPWM_INTEN_PIEN_Msk);
}

/**
 * @brief Disable period interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return None
 * @details This function is used to disable period interrupt of selected channel.
 */
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTEN &= ~(EPWM_INTEN_PIEN_Msk);
}

/**
 * @brief Clear period interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return None
 * @details This function is used to clear period interrupt of selected channel.
 */
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTSTS = (EPWM_INTSTS_PIF_Msk);
}

/**
 * @brief Get period interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occur
 * @retval 1 Period interrupt occurred
 * @details This function is used to get period interrupt of selected channel.
 */
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    return ((((epwm)->INTSTS & (EPWM_INTSTS_PIF_Msk))) ? 1 : 0);
}

/**
 * @brief Enable zero interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return None
 * @details This function is used to enable zero interrupt of selected channel.
 */
void EPWM_EnableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTEN |= (EPWM_INTEN_CIEN_Msk);
}

/**
 * @brief Disable zero interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return None
 * @details This function is used to disable zero interrupt of selected channel.
 */
void EPWM_DisableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTEN &= ~(EPWM_INTEN_CIEN_Msk);
}

/**
 * @brief Clear zero interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return None
 * @details This function is used to clear zero interrupt of selected channel.
 */
void EPWM_ClearZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    (epwm)->INTSTS = (EPWM_INTSTS_CIF_Msk);
}

/**
 * @brief Get zero interrupt of selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used.
 * @return Zero interrupt flag of specified channel
 * @retval 0 Zero interrupt did not occur
 * @retval 1 Zero interrupt occurred
 * @details This function is used to get zero interrupt of selected channel.
 */
uint32_t EPWM_GetZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum)
{
    return ((((epwm)->INTSTS & (EPWM_INTSTS_CIF_Msk))) ? 1 : 0);
}

/*@}*/ /* end of group Mini57_EPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_EPWM_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
