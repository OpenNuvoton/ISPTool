/**************************************************************************//**
 * @file     rtc.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/12/29 2:45p $
 * @brief    Nano 103 RTC driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#include <stdio.h>
#include "Nano103.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Includes of local headers                                                                               */
/*---------------------------------------------------------------------------------------------------------*/



/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_RTC_Driver RTC Driver
  @{
*/
/// @cond HIDDEN_SYMBOLS

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define RTC_GLOBALS

/*---------------------------------------------------------------------------------------------------------*/
/* Global file scope (static) variables                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32Reg, g_u32Reg1,g_u32hiYear,g_u32loYear,g_u32hiMonth,g_u32loMonth,g_u32hiDay,g_u32loDay;
static volatile uint32_t g_u32hiHour,g_u32loHour,g_u32hiMin,g_u32loMin,g_u32hiSec,g_u32loSec;

void RTC_RWEN(void)
{
    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk)) RTC->RWEN = RTC_WRITE_KEY;

    while(RTC->RWEN & RTC_RWEN_RTCBUSY_Msk);
}

/// @endcond HIDDEN_SYMBOLS

/** @addtogroup NANO103_RTC_EXPORTED_FUNCTIONS RTC Exported Functions
  @{
*/


/**
 *  @brief    Set Frequency Compensation Data
 *
 *  @param[in]    i32FrequencyX10000    Specify the RTC clock X10000, ex: 327736512 means 32773.6512.
 *
 *  @return   None
 *
 */
void RTC_32KCalibration(int32_t i32FrequencyX10000)
{
    uint64_t u64Compensate;

    //u64Compensate = (uint64_t)(0x64000000000);
    u64Compensate = (uint64_t)(0x2710000000000);
    u64Compensate = (uint64_t)(u64Compensate / (uint64_t)i32FrequencyX10000);

    if(u64Compensate >= 0x400000)
    {
        u64Compensate = 0x3FFFFF;
    }

    RTC_RWEN();
    RTC->FREQADJ = (uint32_t)u64Compensate;

}

/**
 *  @brief    This function is used to: \n
 *            1. Write initial key to let RTC start count.  \n
 *            2. Input parameter indicates start time.      \n
 *            Null pointer for using default starting time. \n
 *
 *  @param[in]    sPt \n
 *                     Specify the time property and current time. It includes:                          \n
 *                     u32Year: Year value.                                                               \n
 *                     u32Month: Month value.                                                             \n
 *                     u32Day: Day value.                                                                 \n
 *                     u32DayOfWeek: Day of week. [ \ref RTC_SUNDAY / \ref RTC_MONDAY / \ref RTC_TUESDAY /
 *                                                  \ref RTC_WEDNESDAY / \ref RTC_THURSDAY / \ref RTC_FRIDAY /
 *                                                  \ref RTC_SATURDAY]                                       \n
 *                     u32Hour: Hour value.                                                               \n
 *                     u32Minute: Minute value.                                                           \n
 *                     u32Second: Second value.                                                           \n
 *                     u32TimeScale: [ \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24]                                  \n
 *                     u8AmPm: [ \ref RTC_AM / \ref RTC_PM]                                                    \n
 *
 *  @return   None
 *
 */
void RTC_Open (S_RTC_TIME_DATA_T *sPt)
{
    uint32_t u32Reg;

    RTC->INIT = RTC_INIT_KEY;

    if(RTC->INIT != 0x1)
    {
        RTC->INIT = RTC_INIT_KEY;

        while(RTC->INIT != 0x1);
    }

    if(sPt == NULL)
        return;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Second, set RTC 24/12 hour setting                                                                  */
    /*-----------------------------------------------------------------------------------------------------*/
    if (sPt->u32TimeScale == RTC_CLOCK_12)
    {
        RTC_RWEN();
        RTC->CLKFMT &= ~RTC_CLKFMT_24HEN_Msk;

        /*-------------------------------------------------------------------------------------------------*/
        /* important, range of 12-hour PM mode is 21 up to 32                                              */
        /*-------------------------------------------------------------------------------------------------*/
        if (sPt->u32AmPm == RTC_PM)
            sPt->u32Hour += 20;
    }
    else
    {
        RTC_RWEN();
        RTC->CLKFMT |= RTC_CLKFMT_24HEN_Msk;
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set RTC Calender Loading                                                                            */
    /*-----------------------------------------------------------------------------------------------------*/
    u32Reg     = ((sPt->u32Year - RTC_YEAR2000) / 10) << 20;
    u32Reg    |= (((sPt->u32Year - RTC_YEAR2000) % 10) << 16);
    u32Reg    |= ((sPt->u32Month  / 10) << 12);
    u32Reg    |= ((sPt->u32Month  % 10) << 8);
    u32Reg    |= ((sPt->u32Day    / 10) << 4);
    u32Reg    |= (sPt->u32Day     % 10);
    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->CAL = (uint32_t)g_u32Reg;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set RTC Time Loading                                                                                */
    /*-----------------------------------------------------------------------------------------------------*/
    u32Reg     = ((sPt->u32Hour / 10) << 20);
    u32Reg    |= ((sPt->u32Hour % 10) << 16);
    u32Reg    |= ((sPt->u32Minute / 10) << 12);
    u32Reg    |= ((sPt->u32Minute % 10) << 8);
    u32Reg    |= ((sPt->u32Second / 10) << 4);
    u32Reg    |= (sPt->u32Second % 10);
    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->TIME = (uint32_t)g_u32Reg;

    RTC_RWEN();
    RTC->WEEKDAY = sPt->u32DayOfWeek;

}

/**
 *  @brief    Read current date/time from RTC setting
 *
 *  @param[out]    sPt \n
 *                     Specify the time property and current time. It includes: \n
 *                     u32Year: Year value                                      \n
 *                     u32Month: Month value                                    \n
 *                     u32Day: Day value                                        \n
 *                     u32DayOfWeek: Day of week                                \n
 *                     u32Hour: Hour value                                      \n
 *                     u32Minute: Minute value                                  \n
 *                     u32Second: Second value                                  \n
 *                     u32TimeScale: \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24          \n
 *                     u8AmPm: \ref RTC_AM / \ref RTC_PM                            \n
 *
 *  @return   None
 *
 */
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt)
{
    uint32_t u32Tmp;

    sPt->u32TimeScale = RTC->CLKFMT & RTC_CLKFMT_24HEN_Msk;    /* 12/24-hour */
    sPt->u32DayOfWeek = RTC->WEEKDAY & RTC_WEEKDAY_WEEKDAY_Msk;          /* Day of week */

    g_u32hiYear  = (RTC->CAL & RTC_CAL_TENYEAR_Msk) >> RTC_CAL_TENYEAR_Pos;
    g_u32loYear  = (RTC->CAL & RTC_CAL_YEAR_Msk) >> RTC_CAL_YEAR_Pos;
    g_u32hiMonth = (RTC->CAL & RTC_CAL_TENMON_Msk) >> RTC_CAL_TENMON_Pos;
    g_u32loMonth = (RTC->CAL & RTC_CAL_MON_Msk) >> RTC_CAL_MON_Pos;
    g_u32hiDay   = (RTC->CAL & RTC_CAL_TENDAY_Msk) >> RTC_CAL_TENDAY_Pos;
    g_u32loDay   = (RTC->CAL & RTC_CAL_DAY_Msk);

    g_u32hiHour =  (RTC->TIME & RTC_TIME_TENHR_Msk) >> RTC_TIME_TENHR_Pos;
    g_u32loHour =  (RTC->TIME & RTC_TIME_HR_Msk) >> RTC_TIME_HR_Pos;
    g_u32hiMin  =  (RTC->TIME & RTC_TIME_TENMIN_Msk) >> RTC_TIME_TENMIN_Pos;
    g_u32loMin  =  (RTC->TIME & RTC_TIME_MIN_Msk) >> RTC_TIME_MIN_Pos;
    g_u32hiSec  =  (RTC->TIME & RTC_TIME_TENSEC_Msk) >> RTC_TIME_TENSEC_Pos;
    g_u32loSec  =  (RTC->TIME & RTC_TIME_SEC_Msk);

    u32Tmp  = (g_u32hiYear * 10);              /* Compute to 20XX year */
    u32Tmp += g_u32loYear;
    sPt->u32Year = u32Tmp + RTC_YEAR2000;

    u32Tmp = (g_u32hiMonth * 10);              /* Compute 0~12 month */
    sPt->u32Month = u32Tmp + g_u32loMonth;

    u32Tmp = (g_u32hiDay * 10);                /* Compute 0~31 day */
    sPt->u32Day   =  u32Tmp  + g_u32loDay;

    if (sPt->u32TimeScale == RTC_CLOCK_12)   /* Compute12/24 hour */
    {
        u32Tmp = (g_u32hiHour * 10);
        u32Tmp+= g_u32loHour;
        sPt->u32Hour = u32Tmp;                 /* AM: 1~12. PM: 21~32. */

        if (sPt->u32Hour >= 21)
        {
            sPt->u32AmPm = RTC_PM;
            sPt->u32Hour -= 20;
        }
        else
        {
            sPt->u32AmPm = RTC_AM;
        }

        u32Tmp = (g_u32hiMin  * 10);
        u32Tmp+= g_u32loMin;
        sPt->u32Minute = u32Tmp;

        u32Tmp = (g_u32hiSec  * 10);
        u32Tmp+= g_u32loSec;
        sPt->u32Second = u32Tmp;

    }
    else
    {
        u32Tmp  = (g_u32hiHour * 10);
        u32Tmp += g_u32loHour;
        sPt->u32Hour = u32Tmp;

        u32Tmp  = (g_u32hiMin * 10);
        u32Tmp +=  g_u32loMin;
        sPt->u32Minute = u32Tmp;

        u32Tmp  = (g_u32hiSec * 10);
        u32Tmp += g_u32loSec;
        sPt->u32Second = u32Tmp;
    }

}



/**
 *  @brief    Read alarm date/time from RTC setting
 *
 *  @param[out]    sPt \n
 *                     Specify the time property and current time. It includes: \n
 *                     u32Year: Year value                                      \n
 *                     u32Month: Month value                                    \n
 *                     u32Day: Day value                                        \n
 *                     u32DayOfWeek: Day of week                                \n
 *                     u32Hour: Hour value                                      \n
 *                     u32Minute: Minute value                                  \n
 *                     u32Second: Second value                                  \n
 *                     u32TimeScale: \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24          \n
 *                     u8AmPm: \ref RTC_AM / \ref RTC_PM                            \n
 *
 *  @return   None
 *
 */
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt)
{
    uint32_t u32Tmp;

    sPt->u32TimeScale = RTC->CLKFMT & RTC_CLKFMT_24HEN_Msk;  /* 12/24-hour */
    sPt->u32DayOfWeek = RTC->WEEKDAY & RTC_WEEKDAY_WEEKDAY_Msk;        /* Day of week */

    g_u32hiYear  = (RTC->CALM & RTC_CALM_TENYEAR_Msk) >> RTC_CALM_TENYEAR_Pos;
    g_u32loYear  = (RTC->CALM & RTC_CALM_YEAR_Msk)  >> RTC_CALM_YEAR_Pos;
    g_u32hiMonth = (RTC->CALM & RTC_CALM_TENMON_Msk)  >> RTC_CALM_TENMON_Pos;
    g_u32loMonth = (RTC->CALM & RTC_CALM_MON_Msk)   >> RTC_CALM_MON_Pos;
    g_u32hiDay   = (RTC->CALM & RTC_CALM_TENDAY_Msk)  >> RTC_CALM_TENDAY_Pos;
    g_u32loDay   = (RTC->CALM & RTC_CALM_DAY_Msk);

    g_u32hiHour   =  (RTC->TALM & RTC_TALM_TENHR_Msk)  >> RTC_TALM_TENHR_Pos;
    g_u32loHour   =  (RTC->TALM & RTC_TALM_HR_Msk)   >> RTC_TALM_HR_Pos;
    g_u32hiMin    =  (RTC->TALM & RTC_TALM_TENMIN_Msk) >> RTC_TALM_TENMIN_Pos;
    g_u32loMin    =  (RTC->TALM & RTC_TALM_MIN_Msk)  >> RTC_TALM_MIN_Pos;
    g_u32hiSec    =  (RTC->TALM & RTC_TALM_TENSEC_Msk) >> RTC_TALM_TENSEC_Pos;
    g_u32loSec    =  (RTC->TALM & RTC_TALM_SEC_Msk);

    u32Tmp  = (g_u32hiYear * 10);                                    /* Compute to 20XX year */
    u32Tmp += g_u32loYear;
    sPt->u32Year = u32Tmp + RTC_YEAR2000;

    u32Tmp = (g_u32hiMonth * 10);                                    /* Compute 0~12 month */
    sPt->u32Month = u32Tmp + g_u32loMonth;

    u32Tmp = (g_u32hiDay * 10);                                        /* Compute 0~31 day */
    sPt->u32Day = u32Tmp + g_u32loDay;

    if (sPt->u32TimeScale == RTC_CLOCK_12)                  /* Compute12/24 hour */
    {
        u32Tmp  = (g_u32hiHour * 10);
        u32Tmp += g_u32loHour;
        sPt->u32Hour = u32Tmp;                                        /* AM: 1~12. PM: 21~32. */

        if (sPt->u32Hour >= 21)
        {
            sPt->u32AmPm  = RTC_PM;
            sPt->u32Hour -= 20;
        }
        else
        {
            sPt->u32AmPm = RTC_AM;
        }

        u32Tmp  = (g_u32hiMin * 10);
        u32Tmp += g_u32loMin;
        sPt->u32Minute = u32Tmp;

        u32Tmp  = (g_u32hiSec * 10);
        u32Tmp += g_u32loSec;
        sPt->u32Second = u32Tmp;

    }
    else
    {
        u32Tmp  = (g_u32hiHour * 10);
        u32Tmp +=  g_u32loHour;
        sPt->u32Hour = u32Tmp;

        u32Tmp = (g_u32hiMin * 10);
        u32Tmp+= g_u32loMin;
        sPt->u32Minute = u32Tmp;

        u32Tmp  = (g_u32hiSec * 10);
        u32Tmp += g_u32loSec;
        sPt->u32Second = u32Tmp;
    }

}



/**
 *  @brief    This function is used to update date/time to RTC.
 *
 *  @param[in]    sPt \n
 *                     Specify the time property and current time. It includes:                          \n
 *                     u32Year: Year value.                                                               \n
 *                     u32Month: Month value.                                                             \n
 *                     u32Day: Day value.                                                                 \n
 *                     u32DayOfWeek: Day of week. [ \ref RTC_SUNDAY / \ref RTC_MONDAY / \ref RTC_TUESDAY /
 *                                                  \ref RTC_WEDNESDAY / \ref RTC_THURSDAY / \ref RTC_FRIDAY /
 *                                                  \ref RTC_SATURDAY]                                       \n
 *                     u32Hour: Hour value.                                                               \n
 *                     u32Minute: Minute value.                                                           \n
 *                     u32Second: Second value.                                                           \n
 *                     u32TimeScale: [ \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24]                                  \n
 *                     u8AmPm: [ \ref RTC_AM / \ref RTC_PM]                                                    \n
 *
 *
 *  @return   None
 *
 *
 */
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt)
{
    uint32_t u32Reg;

    if (sPt->u32TimeScale == RTC_CLOCK_12)
    {
        RTC->CLKFMT &= ~RTC_CLKFMT_24HEN_Msk;

        /*-----------------------------------------------------------------------------------------*/
        /* important, range of 12-hour PM mode is 21 upto 32                                       */
        /*-----------------------------------------------------------------------------------------*/
        if (sPt->u32AmPm == RTC_PM)
            sPt->u32Hour += 20;
    }
    else
    {
        RTC_RWEN();
        RTC->CLKFMT |= RTC_CLKFMT_24HEN_Msk;
    }

    RTC_RWEN();
    RTC->WEEKDAY = sPt->u32DayOfWeek & RTC_WEEKDAY_WEEKDAY_Msk;

    u32Reg     = ((sPt->u32Year - RTC_YEAR2000) / 10) << 20;
    u32Reg    |= (((sPt->u32Year - RTC_YEAR2000) % 10) << 16);
    u32Reg    |= ((sPt->u32Month  / 10) << 12);
    u32Reg    |= ((sPt->u32Month  % 10) << 8);
    u32Reg    |= ((sPt->u32Day    / 10) << 4);
    u32Reg    |=  (sPt->u32Day    % 10);
    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->CAL = (uint32_t)g_u32Reg;

    u32Reg     = ((sPt->u32Hour   / 10) << 20);
    u32Reg    |= ((sPt->u32Hour   % 10) << 16);
    u32Reg    |= ((sPt->u32Minute / 10) << 12);
    u32Reg    |= ((sPt->u32Minute % 10) << 8);
    u32Reg    |= ((sPt->u32Second / 10) << 4);
    u32Reg    |=  (sPt->u32Second % 10);
    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->TIME = (uint32_t)g_u32Reg;

}

/**
 *  @brief    This function is used to set alarm date/time to RTC.
 *
 *  @param[in]    sPt \n
 *                     Specify the time property and current time. It includes:                          \n
 *                     u32Year: Year value.                                                               \n
 *                     u32Month: Month value.                                                             \n
 *                     u32Day: Day value.                                                                 \n
 *                     u32DayOfWeek: Day of week. [ \ref RTC_SUNDAY / \ref RTC_MONDAY / \ref RTC_TUESDAY /
 *                                                  \ref RTC_WEDNESDAY / \ref RTC_THURSDAY / \ref RTC_FRIDAY /
 *                                                  \ref RTC_SATURDAY ]                                       \n
 *                     u32Hour: Hour value.                                                               \n
 *                     u32Minute: Minute value.                                                           \n
 *                     u32Second: Second value.                                                           \n
 *                     u32TimeScale: [ \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24]                                  \n
 *                     u8AmPm: [ \ref RTC_AM / \ref RTC_PM ]                                                    \n
 *
 *  @return   None
 *
 */
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt)
{
    uint32_t u32Reg;

    RTC_RWEN();

    if (sPt->u32TimeScale == RTC_CLOCK_12)
    {
        RTC->CLKFMT &= ~RTC_CLKFMT_24HEN_Msk;

        /*-----------------------------------------------------------------------------------------*/
        /* important, range of 12-hour PM mode is 21 up to 32                                      */
        /*-----------------------------------------------------------------------------------------*/
        if (sPt->u32AmPm == RTC_PM)
            sPt->u32Hour += 20;
    }
    else
    {
        RTC->CLKFMT |= RTC_CLKFMT_24HEN_Msk;
    }

    RTC_RWEN();
    RTC->WEEKDAY = sPt->u32DayOfWeek & RTC_WEEKDAY_WEEKDAY_Msk;


    u32Reg     = ((sPt->u32Year - RTC_YEAR2000) / 10) << 20;
    u32Reg    |= (((sPt->u32Year - RTC_YEAR2000) % 10) << 16);
    u32Reg    |= ((sPt->u32Month  / 10) << 12);
    u32Reg    |= ((sPt->u32Month  % 10) << 8);
    u32Reg    |= ((sPt->u32Day     / 10) << 4);
    u32Reg    |=  (sPt->u32Day    % 10);
    g_u32Reg   = u32Reg;

    RTC_RWEN();
    RTC->CALM = (uint32_t)g_u32Reg;

    u32Reg     = ((sPt->u32Hour   / 10) << 20);
    u32Reg    |= ((sPt->u32Hour   % 10) << 16);
    u32Reg    |= ((sPt->u32Minute / 10) << 12);
    u32Reg    |= ((sPt->u32Minute % 10) << 8);
    u32Reg    |= ((sPt->u32Second / 10) << 4);
    u32Reg    |=  (sPt->u32Second % 10);
    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->TALM = (uint32_t)g_u32Reg;

}


/**
 *  @brief    This function is used to update date to RTC
 *
 *  @param[in]    u32Year       The Year Calendar Digit of Alarm Setting
 *  @param[in]    u32Month      The Month Calendar Digit of Alarm Setting
 *  @param[in]    u32Day        The Day Calendar Digit of Alarm Setting
 *  @param[in]    u32DayOfWeek  The Day of Week. [ \ref RTC_SUNDAY / \ref RTC_MONDAY / \ref RTC_TUESDAY /
 *                                                 \ref RTC_WEDNESDAY / \ref RTC_THURSDAY / \ref RTC_FRIDAY /
 *                                                 \ref RTC_SATURDAY ]
 *
 *  @return   None
 *
 */
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek)
{
    __IO uint32_t u32Reg;

    RTC_RWEN();
    RTC->WEEKDAY = u32DayOfWeek & RTC_WEEKDAY_WEEKDAY_Msk;

    u32Reg     = ((u32Year - RTC_YEAR2000) / 10) << 20;
    u32Reg    |= (((u32Year - RTC_YEAR2000) % 10) << 16);
    u32Reg    |= ((u32Month  / 10) << 12);
    u32Reg    |= ((u32Month  % 10) << 8);
    u32Reg    |= ((u32Day    / 10) << 4);
    u32Reg    |=  (u32Day    % 10);
    g_u32Reg   = u32Reg;

    RTC_RWEN();
    RTC->CAL = (uint32_t)g_u32Reg;

}

/**
 *  @brief    This function is used to update time to RTC.
 *
 *  @param[in]    u32Hour     The Hour Time Digit of Alarm Setting.
 *  @param[in]    u32Minute   The Month Calendar Digit of Alarm Setting
 *  @param[in]    u32Second   The Day Calendar Digit of Alarm Setting
 *  @param[in]    u32TimeMode The 24-Hour / 12-Hour Time Scale Selection. [ \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24]
 *  @param[in]    u32AmPm     12-hour time scale with AM and PM indication. Only Time Scale select 12-hour used. [ \ref RTC_AM / \ref RTC_PM]
 *
 *  @return   None
 *
 */
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm)
{
    __IO uint32_t u32Reg;

    RTC_RWEN();

    if (u32TimeMode == RTC_CLOCK_12)
    {
        RTC->CLKFMT &= ~RTC_CLKFMT_24HEN_Msk;

        if (u32AmPm == RTC_PM)    /* important, range of 12-hour PM mode is 21 upto 32 */
            u32Hour += 20;
    }
    else if(u32TimeMode == RTC_CLOCK_24)
    {
        RTC->CLKFMT |= RTC_CLKFMT_24HEN_Msk;
    }

    u32Reg     = ((u32Hour   / 10) << 20);
    u32Reg    |= ((u32Hour   % 10) << 16);
    u32Reg    |= ((u32Minute / 10) << 12);
    u32Reg    |= ((u32Minute % 10) << 8);
    u32Reg    |= ((u32Second / 10) << 4);
    u32Reg    |=  (u32Second % 10);

    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->TIME = (uint32_t)g_u32Reg;

}

/**
 *  @brief    This function is used to set alarm date to RTC
 *
 *  @param[in]    u32Year    The Year Calendar Digit of Alarm Setting
 *  @param[in]    u32Month   The Month Calendar Digit of Alarm Setting
 *  @param[in]    u32Day     The Day Calendar Digit of Alarm Setting
 *
 *  @return   None
 *
 */
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day)
{
    __IO uint32_t u32Reg;

    u32Reg       = ((u32Year - RTC_YEAR2000) / 10) << 20;
    u32Reg      |= (((u32Year - RTC_YEAR2000) % 10) << 16);
    u32Reg      |= ((u32Month  / 10) << 12);
    u32Reg      |= ((u32Month  % 10) << 8);
    u32Reg      |= ((u32Day    / 10) << 4);
    u32Reg      |=  (u32Day    % 10);
    g_u32Reg   = u32Reg;

    RTC_RWEN();
    RTC->CALM = (uint32_t)g_u32Reg;

}

/**
 *  @brief    This function is used to set alarm date to RTC
 *
 *  @param[in]     u32Hour     The Hour Time Digit of Alarm Setting.
 *  @param[in]     u32Minute   The Month Calendar Digit of Alarm Setting
 *  @param[in]     u32Second   The Day Calendar Digit of Alarm Setting
 *  @param[in]     u32TimeMode The 24-Hour / 12-Hour Time Scale Selection. [ \ref RTC_CLOCK_12 / \ref RTC_CLOCK_24]
 *  @param[in]     u32AmPm     12-hour time scale with AM and PM indication. Only Time Scale select 12-hour used. [ \ref RTC_AM / \ref RTC_PM]
 *
 *  @return   None
 *
 */
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm)
{
    __IO uint32_t u32Reg;

    RTC_RWEN();

    if (u32TimeMode == RTC_CLOCK_12)
    {
        RTC->CLKFMT &= ~RTC_CLKFMT_24HEN_Msk;

        if (u32AmPm == RTC_PM)    /* important, range of 12-hour PM mode is 21 up to 32 */
            u32Hour += 20;
    }
    else if(u32TimeMode == RTC_CLOCK_24)
    {
        RTC->CLKFMT |= RTC_CLKFMT_24HEN_Msk;
    }

    u32Reg     = ((u32Hour   / 10) << 20);
    u32Reg    |= ((u32Hour   % 10) << 16);
    u32Reg    |= ((u32Minute / 10) << 12);
    u32Reg    |= ((u32Minute % 10) <<  8);
    u32Reg    |= ((u32Second / 10) <<  4);
    u32Reg    |=  (u32Second % 10);

    g_u32Reg = u32Reg;

    RTC_RWEN();
    RTC->TALM = (uint32_t)g_u32Reg;

}


/**
 *  @brief    This function is used to:                       \n
 *            1. Enable tamper detection function.            \n
 *            2. Set tamper control register, interrupt.      \n
 *
 *  @param[in]    u32PinCondition set tamper detection condition: 1=Falling detect, 0=Rising detect
 *
 *  @return   None
 *
 */
void RTC_EnableTamperDetection(uint32_t u32PinCondition)
{
    RTC_RWEN();

    /* detection edge select */
    if(u32PinCondition)
        RTC->SPRCTL |= RTC_SPRCTL_SNPTYPE0_Msk;
    else
        RTC->SPRCTL &= ~RTC_SPRCTL_SNPTYPE0_Msk;

    RTC_RWEN();
    /* enable snooper pin event detection */
    RTC->SPRCTL |= RTC_SPRCTL_SNPDEN_Msk;
}

/**
 *  @brief    This function is used to disable tamper detection function.
 *
 *  @param    None
 *
 *  @return   None
 *
 */
void RTC_DisableTamperDetection(void)
{
    RTC_RWEN();

    RTC->SPRCTL &= ~RTC_SPRCTL_SNPDEN_Msk;
}

/**
 *  @brief    This function is used to get day of week.
 *
 *  @param    None
 *
 *  @return   Day of week
 *
 */
uint32_t RTC_GetDayOfWeek(void)
{
    return (RTC->WEEKDAY & RTC_WEEKDAY_WEEKDAY_Msk);
}


/**
 *  @brief    The function is used to set time tick period for periodic time tick Interrupt.
 *
 *  @param[in]    u32TickSelection
 *                       It is used to set the RTC time tick period for Periodic Time Tick Interrupt request.
 *                       It consists of: \n
 *                       \ref RTC_TICK_1_SEC : Time tick is 1 second        \n
 *                       \ref RTC_TICK_1_2_SEC : Time tick is 1/2 second    \n
 *                       \ref RTC_TICK_1_4_SEC : Time tick is 1/4 second    \n
 *                       \ref RTC_TICK_1_8_SEC : Time tick is 1/8 second    \n
 *                       \ref RTC_TICK_1_16_SEC : Time tick is 1/16 second  \n
 *                       \ref RTC_TICK_1_32_SEC : Time tick is 1/32 second  \n
 *                       \ref RTC_TICK_1_64_SEC : Time tick is 1/64 second  \n
 *                       \ref RTC_TICK_1_128_SEC : Time tick is 1/128 second
 *
 *  @return   None
 *
 */
void RTC_SetTickPeriod(uint32_t u32TickSelection)
{
    RTC_RWEN();

    RTC->TICK = RTC->TICK & ~RTC_TICK_TICK_Msk | u32TickSelection;
}

/**
 *  @brief    The function is used to enable specified interrupt.
 *
 *  @param[in]    u32IntFlagMask      The structure of interrupt source. It consists of: \n
 *                                \ref RTC_INTEN_ALMIEN_Msk : Alarm interrupt                  \n
 *                                \ref RTC_INTEN_TICKIEN_Msk : Tick interrupt                    \n
 *                                \ref RTC_INTEN_SNPDIEN_Msk : Snooper Pin Event Detection Interrupt
 *
 *  @return   None
 *
 */
void RTC_EnableInt(uint32_t u32IntFlagMask)
{
    RTC_RWEN();

    RTC->INTEN |= u32IntFlagMask;
}

/**
 *  @brief    The function is used to disable specified interrupt.
 *
 *  @param[in]    u32IntFlagMask      The structure of interrupt source. It consists of: \n
 *                                \ref RTC_INTEN_ALMIEN_Msk : Alarm interrupt                  \n
 *                                \ref RTC_INTEN_TICKIEN_Msk : Tick interrupt                    \n
 *                                \ref RTC_INTEN_SNPDIEN_Msk : Snooper Pin Event Detection Interrupt\n
 *
 *  @return  None
 *
 */
void RTC_DisableInt(uint32_t u32IntFlagMask)
{
    RTC_RWEN();
    if(u32IntFlagMask & RTC_INTEN_TICKIEN_Msk)
    {
        RTC->INTEN &= ~RTC_INTEN_TICKIEN_Msk;
        RTC->INTSTS = RTC_INTSTS_TICKIF_Msk;
    }

    RTC_RWEN();
    if(u32IntFlagMask & RTC_INTEN_ALMIEN_Msk)
    {
        RTC->INTEN &= ~RTC_INTEN_ALMIEN_Msk;
        RTC->INTSTS = RTC_INTSTS_ALMIF_Msk;
    }

    RTC_RWEN();
    if(u32IntFlagMask & RTC_INTEN_SNPDIEN_Msk)
    {
        RTC->INTEN &= ~RTC_INTEN_SNPDIEN_Msk;
        RTC->INTSTS = RTC_INTSTS_SNPDIF_Msk;
    }

}

/**
 *  @brief    Disable RTC clock.
 *
 *  @return   None
 *
 */
void RTC_Close (void)
{
    CLK->APBCLK  &= ~CLK_APBCLK_RTCCKEN_Msk;
}


/*@}*/ /* end of group NANO103_RTC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_RTC_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


