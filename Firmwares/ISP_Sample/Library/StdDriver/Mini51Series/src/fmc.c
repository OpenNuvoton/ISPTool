/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 15/10/06 10:18a $
 * @brief    MINI51 series FMC driver source file
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

//* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "Mini51Series.h"

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup MINI51_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/


/**
  * @brief    Disable all FMC functions
  */
void FMC_Close(void)
{
    FMC->ISPCON &= ~FMC_ISPCON_ISPEN_Msk;
}


/**
  * @brief    Erase a page. The page size is 512 bytes.
  * @param    u32PageAddr: Flash page address. Must be a 512-byte aligned address.
  * @retval   0: Success
  * @retval   -1: Erase failed
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    if (FMC->ISPCON & FMC_ISPCON_ISPFF_Msk) {
        FMC->ISPCON |= FMC_ISPCON_ISPFF_Msk;
        return -1;
    }
    return 0;
}


/**
  * @brief    get the current boot source
  * @retval   0: This chip is currently booting from APROM
  * @retval   1: This chip is currently booting from LDROM
  */
int32_t FMC_GetBootSource (void)
{
    if (FMC->ISPCON & FMC_ISPCON_BS_Msk)
        return 1;
    else
        return 0;
}


/**
  * @brief    Enable FMC ISP function
  */
void FMC_Open(void)
{
    FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
}


/**
  * @brief    Read a word from specified flash address.
  * @param    u32Addr: Flash word address. Must be a word aligned address.
  * @return   The word data stored in the flash address "u32Addr".
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}


/**
  * @brief    Read company ID.
  * @return   The company ID.
  */
uint32_t FMC_ReadCID(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
    return FMC->ISPDAT;
}


/**
  * @brief    Read product ID.
  * @return   The product ID.
  */
uint32_t FMC_ReadPID(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_PID;
    FMC->ISPADR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
    return FMC->ISPDAT;
}


/**
  * @brief    This function reads one of the four UCID.
  * @param    u32Index: index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return   The UCID.
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADR = (0x04 * u32Index) + 0x10;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}


/**
  * @brief    This function reads one of the three UID.
  * @param    u32Index: index of the UID to read. u32Index must be 0, 1, or 2.
  * @return   The UID.
  */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADR = 0x04 * u32Index;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}


/**
  * @brief    Get the base address of Data Flash if enabled.
  * @return   The base address of Data Flash
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBADR;
}


/**
  * @brief    This function will force re-map assigned flash page to CPU address 0x0.
  * @param    u32PageAddr:  address of the page to be mapped to CPU address 0x0.
  */
void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
}


/**
  * @brief    Obtain the current vector page address setting.
  * @return   The vector page address.
  */
uint32_t FMC_GetVectorPageAddr(void)
{
    return (FMC->ISPSTA & 0x0FFFFF00ul);
}


/**
  * @brief    Writes a word data to specified flash address.
  * @param    u32Addr: destination address
  * @param    u32Data: word data to be written
  */
void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
}


/**
  * @brief    Read the User Configuration words.
  * @param    u32Config: The word array to store data.
  * @param    u32Count: Maximum length of "u32Config".
  * @retval   0:  Success
  * @retval   -1: Failed
  */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
    u32Config[0] = FMC_Read(FMC_CONFIG_BASE);
    if (u32Count < 2)
        return 0;
    u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4);
    return 0;
}


/**
  * @brief    Write User Configuration
  * @param    u32Config: The word array to store data.
  * @param    u32Count: Maximum length of "u32Config".
  * @retval   0:  Success
  * @retval   -1: Failed
  */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{
    FMC_ENABLE_CFG_UPDATE();
    FMC_Erase(FMC_CONFIG_BASE);
    FMC_Write(FMC_CONFIG_BASE, u32Config[0]);
    FMC_Write(FMC_CONFIG_BASE+4, u32Config[1]);
    FMC_DISABLE_CFG_UPDATE();
    return 0;
}


/*@}*/ /* end of group MINI51_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_FMC_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/


