/****************************************************************************//**
 * @file     NuEdu-Basic01_PWM_Capture.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/17 4:41p $
 * @brief    Nano100 series PWM and Capture driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_PWM_Capture_H__
#define __NuEdu_Basic01_PWM_Capture_H__
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS PWM_CAPTURE driver Functions
  @{
*/
#define _PWM_Source_Clock   12000000        //XTAL's 12 MHz
#define _PWM_Resolution     65536
#define _PWM_Prescale_Max   256
#define _PWM_Duty_Scale     100             //0 ~ 100 %

typedef struct {
    __IO uint32_t Capture_Rising[2];
    __IO uint32_t Capture_Falling[2];
    __IO uint32_t Last_Edge;
    __IO uint32_t High_Period;
    __IO uint32_t Low_Period;
    __IO uint32_t Signal_Period;
    __IO float Signal_Frequency;
} PWM_Capture_T;

typedef enum {
    Rising = 0,
    Falling = 1
} E_Edge;

extern uint32_t PWM67_Clock;
extern uint32_t Open_PWM6_OUT_Fail;
extern uint32_t Open_PWM3_Capture_Fail;
extern uint32_t Open_PWM7_Capture_Fail;
extern PWM_Capture_T PWM3;
extern PWM_Capture_T PWM7;

void Open_PWM6_OUT(uint32_t PWM_Frequency, uint32_t PWM_Duty);
void Close_PWM6_OUT(void);

void Open_PWM7_Capture(void);
void Close_PWM7_Capture(void);
void Get_PWM7_Capture_Data(void);

void Open_PWM3_Capture(void);
void Close_PWM3_Capture(void);
void Get_PWM3_Capture_Data(void);

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */
#endif

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
