/**************************************************************************//**
 * @file     NuEdu-Basic01_SPI_Flash_w_PDMA.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 SPI Flash with PDMA driver header file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_SPI_FLASH_W_PDMA_H__
#define __NuEdu_Basic01_SPI_FLASH_W_PDMA_H__
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS SPI Flash with PDMA Functions
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern void Open_SPI_Flash(void);
extern void Init_PDMA_CH1_for_SPI0_TX(uint32_t u32SrcAddr);
extern void Init_PDMA_CH2_for_SPI0_RX(uint32_t u32DstAddr);
extern unsigned int SpiFlash_w_PDMA_ReadMidDid(void);
extern void SpiFlash_w_PDMA_ChipErase(void);
extern unsigned int SpiFlash_w_PDMA_ReadStatusReg1(void);
extern unsigned int SpiFlash_w_PDMA_ReadStatusReg2(void);
extern void SpiFlash_w_PDMA_WaitReady(void);
extern void SpiFlash_w_PDMA_PageProgram(unsigned int StartAddress, unsigned int ByteCount);
extern void SpiFlash_w_PDMA_ReadData(unsigned int StartAddress, unsigned int ByteCount);

#endif
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
