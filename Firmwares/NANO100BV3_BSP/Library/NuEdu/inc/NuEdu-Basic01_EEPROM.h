/**************************************************************************//**
 * @file     NuEdu-Basic01_EEPROM.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 EEPROM driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NuEdu_Basic01_EEPROM_H__
#define __NuEdu_Basic01_EEPROM_H__

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

/** @addtogroup Nano130_Basic01_FUNCTIONS EEPROM Exported Functions
  @{
*/

void I2C_EEPROM_Init(uint8_t u8Divider);
void I2C_EEPROM_Write(uint16_t u16Address, uint8_t u8Data);
uint8_t I2C_EEPROM_Read(uint16_t u16Address);


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS EEPROM Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */


#ifdef __cplusplus
}
#endif

#endif //__NuEdu_Basic01_EEPROM_H__

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
