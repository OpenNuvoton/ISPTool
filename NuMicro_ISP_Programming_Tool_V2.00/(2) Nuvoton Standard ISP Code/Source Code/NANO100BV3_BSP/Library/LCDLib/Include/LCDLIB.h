/******************************************************************************
 * @file     LCDLIB.h
 * @brief    NANO100 LCDLIB header file
 * @version  1.0.1
 * @date     04, September, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __NANO100_LCDLIB_H
#define __NANO100_LCDLIB_H

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NANO100_LCDLIB_Driver LCD Library
  @{
*/


/** @addtogroup NANO100_LCDLIB_EXPORTED_STRUCTS LCDLIB Exported Structs
  @{
*/

typedef struct {
    uint32_t Sub_Zone_Num;      /*!< Sub zone number */
    uint32_t Zone_Digit_SegNum; /*!< Segment number */
} ZoneInfo_TypeDef;

/*@}*/ /* end of group NANO100_LCDLIB_EXPORTED_STRUCTS */


/** @addtogroup NANO100_LCDLIB_EXPORTED_FUNCTIONS LCDLIB Exported Functions
  @{
*/

void LCDLIB_Printf(uint32_t  u32Zone, char *string);
void LCDLIB_PutChar(uint32_t u32Zone, uint32_t u32Index, uint8_t u8Ch);
void LCDLIB_SetSymbol(uint32_t u32Zone, uint32_t u32Index, uint32_t u32OnOff);
void LCDLIB_PrintNumber(uint32_t  u32Zone, long long value);


/*@}*/ /* end of group NANO100_LCDLIB_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO100_LCDLIB_Driver */

/*@}*/ /* end of group NANO100_Library */



#ifdef __cplusplus
}
#endif

#endif  /* __NANO100_LCDLIB_H */



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

