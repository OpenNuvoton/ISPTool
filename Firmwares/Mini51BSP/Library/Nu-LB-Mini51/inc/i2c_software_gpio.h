/**************************************************************************//**
 * @file     i2c_software_gpio.h
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 13/09/30 6:48p $
 * @brief    This is the header file of i2c_software_gpio.c
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __I2C_SOFTWARE_GPIO_H__
#define __I2C_SOFTWARE_GPIO_H__

#include "Mini51Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t I2C_SW_Open(uint32_t u32BusClock);
int32_t I2C_SW_Send(uint8_t u8Address, uint8_t* p8Data, uint32_t u32ByteSize);
int32_t I2C_SW_Get(uint8_t u8Address, uint8_t* p8Data, uint32_t u32ByteSize);
#endif

