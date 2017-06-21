/**************************************************************************//**
 * @file     EEPROM_24LC64.c
 * @version  V0.10
 * $Revision: 6 $
 * $Date: 13/09/30 6:48p $
 * @brief    MINI51 series 24LC64 EEPROM library header file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __LCD_DRIVER_H__
#define __LCD_DRIVER_H__

extern void LCD_ClearScreen(void);
extern void LCD_Print(uint8_t line, char *str);
extern void LCD_EnableBackLight(void);
extern void LCD_DisableBackLight(void);
extern void LCD_Init(void);
#endif
