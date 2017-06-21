/**************************************************************************//**
 * @file     NuEdu-Basic01_System.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/17 4:45p $
 * @brief    NuEdu-Basic01 Volume_Knob driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NuEdu_Basic01_Volume_Knob_H__
#define __NuEdu_Basic01_Volume_Knob_H__

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS System Exported Functions
  @{
*/

#define _ADC_Clock  300000          //ADC_F_Max = 16M or 8M (AVDD = 5V or 3V)

extern uint32_t Open_Volume_Knob_Fail;

extern void Open_Volume_Knob(void);
extern void Close_Volume_Knob(void);
extern uint32_t Get_Volume_Knob(void);

#endif
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS System Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
