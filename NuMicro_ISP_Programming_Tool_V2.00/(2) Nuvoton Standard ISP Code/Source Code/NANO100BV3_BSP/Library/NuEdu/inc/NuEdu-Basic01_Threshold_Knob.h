/**************************************************************************//**
 * @file     NuEdu-Basic01_Threshold_Knob.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 threshold knob driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __NuEdu_Basic01_Threshold_Knob_H__
#define __NuEdu_Basic01_Threshold_Knob_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Threshold Knob Exported Functions
  @{
*/

extern void Open_Threshold_Knob(void);
extern void Close_Threshold_Knob(void);
extern uint32_t Get_Threshold_Knob(void);

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Threshold Knob Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

#ifdef __cplusplus
}
#endif

#endif //__NuEdu_Basic01_Threshold_Knob_H__

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
