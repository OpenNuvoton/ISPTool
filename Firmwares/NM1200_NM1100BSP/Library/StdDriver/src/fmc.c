/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/05/25 3:56p $ 
 * @brief    NM1200_NM1100 FMC driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/  

//* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "NM1200_NM1100.h"

/** @addtogroup NM1200_NM1100_Device_Driver NM1200_NM1100 Device Driver
  @{
*/

/** @addtogroup NM1200_NM1100_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup NM1200_NM1100_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/


/**
  * @brief        Set boot source of next software reset
  * @param[in]    i32BootSrc     1: will boot from LDROM; 0: will boot from APROM
  */
void FMC_SetBootSource (int32_t i32BootSrc)
{
    if (i32BootSrc == 1)
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk;
    else
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;
}


/**
  * @brief    Disable all FMC functions
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief    Disable APROM update function
  */
void FMC_DisableAPUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk;
}


/**
  * @brief    Disable User Configuration update function
  */
void FMC_DisableConfigUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk;
}


/**
  * @brief    Disable LDROM update function
  */
void FMC_DisableLDUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;
}


/**
  * @brief    Enable APROM update function
  */
void FMC_EnableAPUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_APUEN_Msk;
}


/**
  * @brief    Enable User Configuration update function
  */
void FMC_EnableConfigUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk;
}


/**
  * @brief    Enable LDROM update function
  */
void FMC_EnableLDUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;
}


/**
  * @brief    Erase a page. The page size is 512 bytes. 
  * @param[in]    u32PageAddr: Flash page address. Must be a 512-byte aligned address.
  * @retval   0: Success
  * @retval   -1: Erase failed
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk; 
    
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
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
    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
        return 1;
    else
        return 0;
}


/**
  * @brief    Enable FMC ISP function
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief    Read a word from specified flash address. 
  * @param[in]    u32Addr: Flash word address. Must be a word aligned address.
  * @return   The word data stored in the flash address "u32Addr".
  */
uint32_t FMC_Read(uint32_t u32Addr)
{ 
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
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
    FMC->ISPADDR = 0x0;
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
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk; 
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
    return FMC->ISPDAT; 
}


/**
  * @brief    This function reads one of the four UCID. 
  * @param[in]    u32Index: index of the UCID to read. u32Index must be 0, 1, 2, or 3. 
  * @return   The UCID.
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk; 
    
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT; 
}


/**
  * @brief    This function reads one of the three UID. 
  * @param[in]    u32Index: index of the UID to read. u32Index must be 0, 1, or 2. 
  * @return   The UID.
  */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = 0x04 * u32Index;
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
    return FMC->DFBA; 
}


/**
  * @brief    This function will force re-map assigned flash page to CPU address 0x0.
  * @param[in]    u32PageAddr:  address of the page to be mapped to CPU address 0x0.
  */
void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk; 
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
}


/**
  * @brief    Writes a word data to specified flash address.
  * @param[in]    u32Addr: destination address
  * @param[in]   u32Data: word data to be written
  */
void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk; 
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
}


/**
  * @brief    Read the User Configuration words. 
  * @param[in]    u32Config: The word array to store data.
  * @param[in]    u32Count: Maximum length of "u32Config".
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
  * @param[in]    u32Config: The word array to store data.
  * @param[in]    u32Count: Maximum length of "u32Config".
  * @retval   0:  Success
  * @retval   -1: Failed
  */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{       
    FMC_EnableConfigUpdate();   
    FMC_Erase(FMC_CONFIG_BASE);
    FMC_Write(FMC_CONFIG_BASE, u32Config[0]);
    FMC_Write(FMC_CONFIG_BASE+4, u32Config[1]);
    FMC_DisableConfigUpdate();
    return 0;
}


/*@}*/ /* end of group NM1200_NM1100_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1200_NM1100_FMC_Driver */

/*@}*/ /* end of group NM1200_NM1100_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


