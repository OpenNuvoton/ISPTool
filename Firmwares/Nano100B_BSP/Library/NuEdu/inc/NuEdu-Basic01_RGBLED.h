/**************************************************************************//**
 * @file     NuEdu-Basic01_RGBLED.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 RGB LED driver header file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_RGBLED_H__
#define __NuEdu_Basic01_RGBLED_H__
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS RGB LED Functions
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern void Initial_PWM_LED(void);
extern void PWM_LED(void);

#endif
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
