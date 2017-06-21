/**************************************************************************//**
 * @file     NuEdu-Basic01_Buzzer.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 Buzzer driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NuEdu_Basic01_Buzzer_H__
#define __NuEdu_Basic01_Buzzer_H__

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

/** @addtogroup Nano130_Basic01_FUNCTIONS Buzzer Exported Functions
  @{
*/

extern void Open_Buzzer(void);
extern void Close_Buzzer(void);
extern void Write_Buzzer(unsigned int frequence, unsigned int duty);


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Buzzer Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */


#ifdef __cplusplus
}
#endif

#endif //__NuEdu_Basic01_Buzzer_H__

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
