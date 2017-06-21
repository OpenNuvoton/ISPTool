/**************************************************************************//**
 * @file     NuEdu-Basic01_7_Segment.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 7_Segment LED driver header file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_7_Segment_H__
#define __NuEdu_Basic01_7_Segment_H__
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Seven Segment LED Functions
    @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern void Show_Seven_Segment(unsigned char no, unsigned char number);
extern void Close_Seven_Segment(void);
extern void Open_Seven_Segment(void);

#endif
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
