/**************************************************************************//**
 * @file     bpwm.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/04/01 7:32p $
 * @brief    BPWM driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "M0519.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup BPWM_Driver BPWM Driver
  @{
*/


/** @addtogroup BPWM_EXPORTED_FUNCTIONS BPWM Exported Functions
  @{
*/

/**
 * @brief Configure BPWM capture and get the nearest unit time.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @param[in] u32UnitTimeNsec The unit time of counter.
 * @param[in] u32CaptureEdge The condition to latch the counter. This parameter is not used.
 * @return The nearest unit time in nano second.
 * @details This function is used to configure BPWM capture and get the nearest unit time.
 * @note Since Channel 0 and channel 1 shares a prescaler. Call this API to configure BPWM capture frequency may affect
 *       existing frequency of other channel.
 */
uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm,
                                   uint32_t u32ChannelNum,
                                   uint32_t u32UnitTimeNsec,
                                   uint32_t u32CaptureEdge)
{
    uint32_t u32PWMClockSrc;
    uint32_t u32NearestUnitTimeNsec;
    uint8_t  u8Divider = 1;
    /* this table is mapping divider value to register configuration */
    uint32_t u32PWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
    uint16_t u16Prescale = 2;
    uint16_t u16CNR = 0xFFFF;

    SystemCoreClockUpdate();
    u32PWMClockSrc = SystemCoreClock;

    u32PWMClockSrc /= 1000;
    for(; u16Prescale <= 0x100; u16Prescale++)
    {
        u32NearestUnitTimeNsec = (1000000 * u16Prescale * u8Divider) / u32PWMClockSrc;
        if(u32NearestUnitTimeNsec < u32UnitTimeNsec)
        {
            if((u16Prescale == 0x100) && (u8Divider == 16)) /* limit to the maximum unit time(nano second) */
                break;
            if(u16Prescale == 0x100)
            {
                u16Prescale = 2;
                u8Divider <<= 1; /* clk divider could only be 1, 2, 4, 8, 16 */
                continue;
            }
            if(!((1000000  * ((u16Prescale * u8Divider) + 1)) > (u32NearestUnitTimeNsec * u32PWMClockSrc)))
                break;
            continue;
        }
        break;
    }

    /* Store return value here 'cos we're gonna change u8Divider & u16Prescale & u16CNR to the real value to fill into register */
    u16Prescale -= 1;

    /* convert to real register value */
    u8Divider = u32PWMDividerToRegTbl[u8Divider];

    /* every two channels share a prescaler */
    (bpwm)->PPR = ((bpwm)->PPR & ~(BPWM_PPR_CP01_Msk)) | (u16Prescale);
    (bpwm)->CSR = ((bpwm)->CSR & ~(BPWM_CSR_CSR0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
    /* set PWM to edge aligned type */
    (bpwm)->PCR &= ~(BPWM_PCR_PWM01TYPE_Msk);
    (bpwm)->PCR |= BPWM_PCR_CH0MOD_Msk << (8 * u32ChannelNum);
    *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CNR0)) + (u32ChannelNum) * 12))) = u16CNR;

    return (u32NearestUnitTimeNsec);
}

/**
 * @brief Configure BPWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @param[in] u32Frequency Target generator frequency.
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second.
 * @details This function is used to configure BPWM generator and get the nearest frequency in edge aligned auto-reload mode.
 * @note Since Channel 0 and channel 1 shares a prescaler. Call this API to configure BPWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t u32PWMClockSrc;
    uint32_t i;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    /* this table is mapping divider value to register configuration */
    uint32_t u32PWMDividerToRegTbl[17] = {NULL, 4, 0, NULL, 1, NULL, NULL, NULL, 2, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 3};
    uint16_t u16CNR = 0xFFFF;

    SystemCoreClockUpdate();
    u32PWMClockSrc = SystemCoreClock;

    for(; u8Divider < 17; u8Divider <<= 1) /* clk divider could only be 1, 2, 4, 8, 16 */
    {
        i = (u32PWMClockSrc / u32Frequency) / u8Divider;
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
                u16CNR = 1; /* Too fast, and PWM cannot generate expected frequency. */
            else
                u16CNR = i;
            break;
        }
    }
    /* Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register */
    i = u32PWMClockSrc / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR -= 1;
    /* convert to real register value */
    u8Divider = u32PWMDividerToRegTbl[u8Divider];

    /* every two channels share a prescaler */
    (bpwm)->PPR = ((bpwm)->PPR & ~(BPWM_PPR_CP01_Msk)) | (u8Prescale);
    (bpwm)->CSR = ((bpwm)->CSR & ~(BPWM_CSR_CSR0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
    /* set PWM to edge aligned type */
    (bpwm)->PCR &= ~(BPWM_PCR_PWM01TYPE_Msk);
    (bpwm)->PCR |= BPWM_PCR_CH0MOD_Msk << (8 * u32ChannelNum);

    if(u32DutyCycle)
    {
        *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CMR0)) + u32ChannelNum * 12))) = u32DutyCycle * (u16CNR + 1) / 100 - 1;
    }
    else
    {
        *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CMR0)) + u32ChannelNum * 12))) = 0;
    }
    *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CNR0)) + (u32ChannelNum) * 12))) = u16CNR;

    return(i);
}


/**
 * @brief Start BPWM module.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1.
 * @return None.
 * @details This function is used to start BPWM module.
 */
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            u32Mask |= (BPWM_PCR_CH0EN_Msk << (i * 8));
        }
    }

    (bpwm)->PCR |= u32Mask;
}

/**
 * @brief Stop BPWM module.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1.
 * @return None.
 * @details This function is used to stop BPWM module.
 */
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            *((__IO uint32_t *)((((uint32_t) & ((bpwm)->CNR0)) + i * 12))) = 0;
        }
    }
}

/**
 * @brief Stop BPWM generation immediately by clear channel enable bit.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1.
 * @return None.
 * @details This function is used to stop BPWM generation immediately by clear channel enable bit.
 */
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            u32Mask |= (BPWM_PCR_CH0EN_Msk << (i * 8));
        }
    }

    (bpwm)->PCR &= ~u32Mask;
}

/**
 * @brief Enable capture of selected channel(s).
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1.
 * @return None.
 * @details This function is used to enable capture of selected channel(s).
 */
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            (bpwm)->CCR |= BPWM_CCR_CAPCH0EN_Msk << (i * 16);
        }
    }
    (bpwm)->CAPENR |= u32ChannelMask;
}

/**
 * @brief Disable capture of selected channel(s).
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1.
 * @return None.
 * @details This function is used to disable capture of selected channel(s).
 */
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < BPWM_CHANNEL_NUM; i ++)
    {
        if(u32ChannelMask & (1 << i))
        {
            (bpwm)->CCR &= ~(BPWM_CCR_CAPCH0EN_Msk << (i * 16));
        }
    }
    (bpwm)->CAPENR &= ~u32ChannelMask;
}

/**
 * @brief Enables BPWM output generation of selected channel(s).
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output.
 * @return None.
 * @details This function is used to enables BPWM output generation of selected channel(s).
 */
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    (bpwm)->POE |= u32ChannelMask;
    GPIO->PWMPOEN &= ~GPIO_PWMPOEN_HZ_BPWM_Msk;
}

/**
 * @brief Disables BPWM output generation of selected channel(s).
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output.
 * @return None.
 * @details This function is used to disables BPWM output generation of selected channel(s).
 * @note If u32ChannelMask = 0 then the driving mode of Basic BPWM ports are forced in tri-state (Hi-Z) all the time.
 */
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask)
{
    (bpwm)->POE &= ~u32ChannelMask;
    if(u32ChannelMask == 0)
    {
        GPIO->PWMPOEN |= GPIO_PWMPOEN_HZ_BPWM_Msk;
    }
}

/**
 * @brief Enable Dead zone of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. This parameter is not used.
 * @param[in] u32Duration Dead Zone length in BPWM clock count, valid values are between 0~0xFF, but 0 means there is no dead zone.
 * @return None.
 * @details This function is used to enable Dead zone of selected channel.
 * @note Channel 0 and channel 1 share the same configuration.
 */
void BPWM_EnableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    /* set duration */
    (bpwm)->PPR = ((bpwm)->PPR & ~(BPWM_PPR_DZI01_Msk)) | (u32Duration << BPWM_PPR_DZI01_Pos);
    /* enable dead zone */
    (bpwm)->PCR |= BPWM_PCR_DZEN01_Msk;
}

/**
 * @brief Disable Dead zone of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. This parameter is not used.
 * @return None.
 * @details This function is used to disable Dead zone of selected channel.
 * @note Channel 0 and channel 1 share the same configuration.
 */
void BPWM_DisableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    /* disable dead zone */
    (bpwm)->PCR &= ~(BPWM_PCR_DZEN01_Msk);
}

/**
 * @brief Enable capture interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref BPWM_CAPTURE_INT_RISING_LATCH
 *              - \ref BPWM_CAPTURE_INT_FALLING_LATCH
 * @return None.
 * @details This function is used to enable capture interrupt of selected channel.
 */
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
    (bpwm)->CCR |= u32Edge << (u32ChannelNum * 16);
}

/**
 * @brief Disable capture interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref BPWM_CAPTURE_INT_RISING_LATCH
 *              - \ref BPWM_CAPTURE_INT_FALLING_LATCH
 * @return None.
 * @details This function is used to disable capture interrupt of selected channel.
 */
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
    (bpwm)->CCR &= u32Edge << ~(u32ChannelNum * 16);
}

/**
 * @brief Clear capture interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @param[in] u32Edge Rising or falling edge to latch counter.
 *              - \ref BPWM_CAPTURE_INT_RISING_LATCH
 *              - \ref BPWM_CAPTURE_INT_FALLING_LATCH
 * @return None.
 * @details This function is used to clear capture interrupt of selected channel.
 */
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge)
{
    /* clear capture interrupt flag, and clear CRLR or CFLR latched indicator */
    (bpwm)->CCR = ((bpwm)->CCR & BPWM_CCR_MASK) | (BPWM_CCR_CAPIF0_Msk << (u32ChannelNum * 16)) | (u32Edge << (u32ChannelNum * 16 + 5));
}

/**
 * @brief Get capture interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @retval 0 No capture interrupt.
 * @retval 1 Rising edge latch interrupt.
 * @retval 2 Falling edge latch interrupt.
 * @retval 3 Rising and falling latch interrupt.
 * @details This function is used to get capture interrupt of selected channel.
 */
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    return (((bpwm)->CCR & ((BPWM_CCR_CRLRI0_Msk | BPWM_CCR_CFLRI0_Msk) << (u32ChannelNum * 16))) >> (BPWM_CCR_CRLRI0_Pos + u32ChannelNum * 16));
}
/**
 * @brief Enable duty interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @param[in] u32IntDutyType This parameter is not used.
 * @return None.
 * @details This function is used to enable duty interrupt of selected channel.
 *          Channel 0 and channel 1 shares the duty interrupt type setting.
 * @note If CMR equal to CNR, this flag is not working in Edge-aligned type selection.
 */
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType)
{
    (bpwm)->PIER |= (BPWM_PIER_PWMDIE0_Msk << u32ChannelNum);
}

/**
 * @brief Disable duty interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @return None.
 * @details This function is used to disable duty interrupt of selected channel.
 */
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->PIER &= ~(BPWM_PIER_PWMDIE0_Msk << u32ChannelNum);
}

/**
 * @brief Clear duty interrupt flag of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @return None.
 * @details This function is used to clear duty interrupt flag of selected channel.
 */
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->PIIR = BPWM_PIIR_PWMDIF0_Msk << u32ChannelNum;
}

/**
 * @brief Get duty interrupt flag of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @retval 0 Duty interrupt did not occur.
 * @retval 1 Duty interrupt occurred.
 * @details This function is used to get duty interrupt flag of selected channel.
 */
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    return (((bpwm)->PIIR & (BPWM_PIIR_PWMDIF0_Msk << u32ChannelNum)) ? 1 : 0);
}

/**
 * @brief Enable period interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1.
 * @param[in] u32IntPeriodType Period interrupt type, could be either
 *              - \ref BPWM_PERIOD_INT_UNDERFLOW
 *              - \ref BPWM_PERIOD_INT_MATCH_CNR
 * @return None.
 * @details This function is used to enable period interrupt of selected channel.
 *          Channel 0 and channel 1 shares the period interrupt type setting.
 * @note This function is effective when BPWM works in Center-aligned type only.
 */
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
    (bpwm)->PIER = ((bpwm)->PIER & ~(BPWM_PIER_INTTYPE_Msk)) | (BPWM_PIER_PWMPIE0_Msk << u32ChannelNum) | (u32IntPeriodType);
}

/**
 * @brief Disable period interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @return None.
 * @details This function is used to disable period interrupt of selected channel.
 */
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->PIER &= ~(BPWM_PIER_PWMPIE0_Msk << u32ChannelNum);
}

/**
 * @brief Clear period interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @return None.
 * @details This function is used to clear period interrupt of selected channel.
 */
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    (bpwm)->PIIR = (BPWM_PIIR_PWMIF0_Msk << u32ChannelNum);
}

/**
 * @brief Get period interrupt of selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are 0 and 1.
 * @retval 0 Period interrupt did not occur.
 * @retval 1 Period interrupt occurred.
 * @details This function is used to get period interrupt of selected channel.
 */
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum)
{
    return (((bpwm)->PIIR & (BPWM_PIIR_PWMIF0_Msk << (u32ChannelNum))) ? 1 : 0);
}



/*@}*/ /* end of group BPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group BPWM_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
