/**************************************************************************//**
 * @file     NuEdu-Basic01_I2C_EEPROM.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01_I2C_EEPROM I2C driver header file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_I2C_EEPROM_H__
#define __NuEdu_Basic01_I2C_EEPROM_H__

#include "Nano100Series.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS I2C Functions
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_EEPROM_Init(uint8_t u8Divider);
void I2C_EEPROM_Write(uint16_t u16Address, uint8_t u8Data);
uint8_t I2C_EEPROM_Read(uint16_t u16Address);

#endif
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
