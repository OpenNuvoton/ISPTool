/**************************************************************************//**
 * @file     spi_flash.h
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 15/04/01 7:46p $
 * @brief   SPI Flash driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__


#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Component_Library Component Library
  @{
*/

/** @addtogroup SPIFLASH SPI Flash Library
  @{
*/

/** @addtogroup SPIFLASH_EXPORTED_CONSTANTS SPI Flash Library Exported Constants
  @{
*/


/* Function prototype declaration */
uint32_t SpiFlash_ReadMidDid(SPI_T *spi);
void SpiFlash_ChipErase(SPI_T *spi);
uint32_t SpiFlash_ReadStatusReg(SPI_T *spi);
void SpiFlash_WriteStatusReg(SPI_T *spi, uint32_t u32Value);
void SpiFlash_WaitReady(SPI_T *spi);
void SpiFlash_PageProgram(SPI_T *spi, uint32_t u32StartAddress, uint8_t *au8DataBuffer, uint32_t u32ByteCount);
void SpiFlash_ReadData(SPI_T *spi, uint32_t u32StartAddress, uint8_t *au8DataBuffer, uint32_t u32ByteCount);


#ifdef __cplusplus
}
#endif

#endif //__SPI_FLASH_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
