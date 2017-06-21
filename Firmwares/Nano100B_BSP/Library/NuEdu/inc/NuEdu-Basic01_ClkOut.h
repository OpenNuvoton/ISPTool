/**************************************************************************//**
 * @file     NuEdu-Basic01_ClkOut.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 ClkOut driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NuEdu_Basic01_ClkOut_H__
#define __NuEdu_Basic01_ClkOut_H__

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

/** @addtogroup Nano130_Basic01_FUNCTIONS ClkOut Exported Functions
  @{
*/

void Open_CLK_OUT(uint32_t Clock_Source, uint32_t FRQDIV_FSEL);
void Close_CLK_OUT(void);


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS ClkOut Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */


#ifdef __cplusplus
}
#endif

#endif //__NuEdu_Basic01_ClkOut_H__

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
