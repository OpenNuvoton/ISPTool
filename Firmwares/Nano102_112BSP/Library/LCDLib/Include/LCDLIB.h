/**************************************************************************//**
 * @file     LCDLIB.h
 * @version  V1.01
 * $Revision: 4 $
 * $Date: 14/11/28 1:48p $
 * @brief    Nano 102/112 series LCDLIB header file
 *
 * @note
 * Copyright (C) 2013~2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NANO1X2_LCDLIB_H
#define __NANO1X2_LCDLIB_H

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO1X2_Library NANO102/112 Library
  @{
*/

/** @addtogroup NANO1X2_LCDLIB_Driver LCD Library
  @{
*/


/** @addtogroup NANO1X2_LCDLIB_EXPORTED_STRUCTS LCDLIB Exported Structs
  @{
*/

typedef struct {
    uint32_t Sub_Zone_Num;      ///< Sub zone number
    uint32_t Zone_Digit_SegNum; ///< Segment number
} ZoneInfo_TypeDef;

/*@}*/ /* end of group NANO1X2_LCDLIB_EXPORTED_STRUCTS */


/** @addtogroup NANO1X2_LCDLIB_EXPORTED_FUNCTIONS LCDLIB Exported Functions
  @{
*/

void LCDLIB_Printf(uint32_t  u32Zone, char *string);
void LCDLIB_PutChar(uint32_t u32Zone, uint32_t u32Index, uint8_t u8Ch);
void LCDLIB_SetSymbol(uint32_t u32Zone, uint32_t u32Index, uint32_t u32OnOff);
void LCDLIB_PrintNumber(uint32_t  u32Zone, long long value);


/*@}*/ /* end of group NANO1X2_LCDLIB_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO1X2_LCDLIB_Driver */

/*@}*/ /* end of group NANO1X2_Library */



#ifdef __cplusplus
}
#endif

#endif  /* __NANO1X2_LCDLIB_H */



/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

