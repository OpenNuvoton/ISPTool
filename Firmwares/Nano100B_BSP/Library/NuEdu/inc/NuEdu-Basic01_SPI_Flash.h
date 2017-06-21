/**************************************************************************//**
 * @file     NuEdu-Basic01_SPI_Flash.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 SPI Flash driver header file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_SPI_FLASH_H__
#define __NuEdu_Basic01_SPI_FLASH_H__
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS SPI Flash Functions
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern void Open_SPI_Flash(void);
extern unsigned int SpiFlash_ReadMidDid(void);
extern void SpiFlash_ChipErase(void);
extern unsigned int SpiFlash_ReadStatusReg1(void);
extern unsigned int SpiFlash_ReadStatusReg2(void);
extern void SpiFlash_WaitReady(void);
extern void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount);
extern void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount);

#endif
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
