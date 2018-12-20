/**************************************************************************//**
 * @file     bpwm.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series BPWM driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini57Series.h"

/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_BPWM_Driver BPWM Driver
  @{
*/


/** @addtogroup Mini57_BPWM_EXPORTED_FUNCTIONS BPWM Exported Functions
  @{
*/

/**
 * @brief Configure BPWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @details This function is used to configure BPWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @note Since every two channels, (0 & 1), shares a prescaler. Call this API to configure BPWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t u32BPWMClockSrc;
    uint32_t i;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    /* this table is mapping divider value to register configuration */
    uint32_t u32BPWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
    uint16_t u16CNR = 0xFFFF;

    SystemCoreClockUpdate();
    u32BPWMClockSrc = SystemCoreClock;

    for(; u8Divider < 17; u8Divider <<= 1)    /* clk divider could only be 1, 2, 4, 8, 16 */
    {
        i = (u32BPWMClockSrc / u32Frequency) / u8Divider;
        /* If target value is larger than CNR * prescale, need to use a larger divider */
        if(i > (0x10000 * 0x100))
            continue;

        /* CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF */
        u8Prescale = (i + 0xFFFF) / 0x10000;

        /* u8Prescale must at least be 2, otherwise the output stop */
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000)
        {
            if(i == 1)
                u16CNR = 1;     /* Too fast, and BPWM cannot generate expected frequency... */
            else
                u16CNR = i;
            break;
        }
    }
    /* Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register */
    i = u32BPWMClockSrc / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR -= 1;
    /* convert to real register value */
    u8Divider = u32BPWMDividerToRegTbl[u8Divider];

    /* every two channels share a prescaler */
    (bpwm)->CLKPSC = ((bpwm)->CLKPSC & ~(BPWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    (bpwm)->CLKDIV = ((bpwm)->CLKDIV & ~(BPWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
    /* set BPWM to edge aligned type */
    (bpwm)->CTL &= ~(BPWM_CTL_CNTTYPE01_Msk << (u32ChannelNum >> 1));
    (bpwm)->CTL |= BPWM_CTL_CNTMODE0_Msk << (8 * u32ChannelNum);

    if(u32DutyCycle)
    {
        *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CMPDAT0)) + u32ChannelNum * 12))) = u32DutyCycle * (u16CNR + 1) / 100 - 1;
    }
    else
    {
        *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CMPDAT0)) + u32ChannelNum * 12))) = 0;
    }
    *((__IO uint32_t *)((((uint32_t) & ((bpwm)->PERIOD0)) + (u32ChannelNum) * 12))) = u16CNR;

    return(i);
}


/**
 * @brief Start BPWM module
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to start BPWM module.
 */
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            u32Mask |= (BPWM_CTL_CNTEN0_Msk << (i * 8));
        }
    }

    (bpwm)->CTL |= u32Mask;
}

/**
 * @brief Stop BPWM module
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop BPWM module.
 */
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            *((__IO uint32_t *)((((uint32_t) & ((bpwm)->PERIOD0)) + i * 12))) = 0;
        }
    }
}

/**
 * @brief Stop BPWM generation immediately by clear channel enable bit
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 * @details This function is used to stop BPWM generation immediately by clear channel enable bit.
 */
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            u32Mask |= (BPWM_CTL_CNTEN0_Msk << (i * 8));
        }
    }

    (bpwm)->CTL &= ~u32Mask;
}

/**
 * @brief Enables BPWM output generation of selected channel(s)
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 * @details This function is used to enables BPWM output generation of selected channel(s).
 */
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    (bpwm)->POEN |= u32ChannelMask;
}

/**
 * @brief Disables BPWM output generation of selected channel(s)
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 * @details This function is used to disables BPWM output generation of selected channel(s).
 */
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    (bpwm)->POEN &= ~u32ChannelMask;
}

/**
 * @brief Enable Dead zone of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32Duration Dead Zone length in BPWM clock count, valid values are between 0~0xFF, but 0 means there is no
 *                        dead zone.
 * @return None
 * @details This function is used to enable Dead zone of selected channel.
 */
void BPWM_EnableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    /* every two channels shares the same setting */
    u32ChannelNum >>= 1;
    /* set duration */
    (bpwm)->CLKPSC = ((bpwm)->CLKPSC & ~(BPWM_CLKPSC_DTI01_Msk << (8 * u32ChannelNum))) | (u32Duration << (BPWM_CLKPSC_DTI01_Pos + 8 * u32ChannelNum));
    /* enable dead zone */
    (bpwm)->CTL |= (BPWM_CTL_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief Disable Dead zone of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to disable Dead zone of selected channel.
 */
void BPWM_DisableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    /* every two channels shares the same setting */
    u32ChannelNum >>= 1;
    /* enable dead zone */
    (bpwm)->CTL &= ~(BPWM_CTL_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief Enable duty interrupt of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32IntDutyType This parameter is not used
 * @return None
 * @details This function is used to enable duty interrupt of selected channel.
 */
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType)
{
    (bpwm)->INTEN |= (BPWM_INTEN_DIEN0_Msk << u32ChannelNum);
}

/**
 * @brief Disable duty interrupt of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to disable duty interrupt of selected channel.
 */
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->INTEN &= ~(BPWM_INTEN_DIEN0_Msk << u32ChannelNum);
}

/**
 * @brief Clear duty interrupt flag of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to clear duty interrupt flag of selected channel.
 */
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->INTSTS = BPWM_INTSTS_DIF0_Msk << u32ChannelNum;
}

/**
 * @brief Get duty interrupt flag of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return Duty interrupt flag of specified channel
 * @retval 0 Duty interrupt did not occur
 * @retval 1 Duty interrupt occurred
 * @details This function is used to get duty interrupt flag of selected channel.
 */
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    return (((bpwm)->INTSTS & (BPWM_INTSTS_DIF0_Msk << u32ChannelNum)) ? 1 : 0);
}

/**
 * @brief Enable period interrupt of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @param[in] u32IntPeriodType Period interrupt type, could be either
 *              - \ref BPWM_PERIOD_INT_UNDERFLOW
 *              - \ref BPWM_PERIOD_INT_MATCH_CNR
 * @return None
 * @details This function is used to enable period interrupt of selected channel.
 *          Every two channels, (0 & 1), shares the period interrupt type setting.
 */
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
    (bpwm)->INTEN = ((bpwm)->INTEN & ~(BPWM_INTEN_PINTTYPE_Msk << (u32ChannelNum >> 1))) | \
                    (BPWM_INTEN_PIEN0_Msk << u32ChannelNum) | (u32IntPeriodType << (u32ChannelNum >> 1));
}

/**
 * @brief Disable period interrupt of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to disable period interrupt of selected channel.
 */
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->INTEN &= ~(BPWM_INTEN_PIEN0_Msk << u32ChannelNum);
}

/**
 * @brief Clear period interrupt of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return None
 * @details This function is used to clear period interrupt of selected channel.
 */
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->INTSTS = (BPWM_INTSTS_PIF0_Msk << u32ChannelNum);
}

/**
 * @brief Get period interrupt of selected channel
 * @param[in] bpwm The pointer of the specified BPWM module
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occur
 * @retval 1 Period interrupt occurred
 * @details This function is used to get period interrupt of selected channel.
 */
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    return (((bpwm)->INTSTS & (BPWM_INTSTS_PIF0_Msk << (u32ChannelNum))) ? 1 : 0);
}



/*@}*/ /* end of group Mini57_BPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_BPWM_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
