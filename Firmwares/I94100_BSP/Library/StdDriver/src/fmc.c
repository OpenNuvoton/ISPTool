/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/12/16 11:13a $
 * @brief    I94100 Series FMC driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "I94100.h"

#define ISBEN   0

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup I94100_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/


/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  * @param[in]  i32BootSrc
  *                         1: Boot from LDROM,
  *                         0: Boot from APROM
  * @return   None
  * @details  This function is used to switch APROM boot or LDROM boot. User need to call
  *           FMC_SetBootSource to select boot source first, then use CPU reset or
  *           System Reset Request to reset system.
  *
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    if(i32BootSrc)
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    else
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;/* Boot from APROM */
}

/**
  * @brief    Disable ISP Functions
  * @param    None
  * @return   None
  * @details  This function will clear ISPEN bit of ISPCTL to disable ISP function
  *
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief	 	Execute FMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 4096 bytes.
  * @param[in]	u32PageAddr Address of the flash page to be erased. 
  *           	It must be a 4096 bytes aligned address.
  * @return	  	ISP page erase success or not.
  * @retval  	0 Success
  * @retval   	-1 Erase failed
  * @note       FMC_ISPADDR[11:0] will be ignored.
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
	int32_t i32Err = 0;
	
	FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while (FMC->ISPTRG) ;

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        i32Err = -1;
    }
	
	FMC->ISPCMD = 0;
	
    return i32Err;
}

/**
  * @brief      Execute FMC_ISPCMD_BLOCK_ERASE command to erase a APROM block. The block size is 4 pages.
  * @param[in]  u32BlockAddr  Address of the flash block to be erased.
  *             It must be a 4 pages aligned address.
  * @return     ISP page erase success or not.
  * @retval     0  Success
  * @retval     -1  Erase failed
  * @note       FMC_ISPADDR[13:0] will be ignored.
  */
int32_t FMC_Erase_Block(uint32_t u32BlockAddr)
{
    int32_t i32Err = 0;
	
	FMC->ISPCMD = FMC_ISPCMD_BLOCK_ERASE;
    FMC->ISPADDR = u32BlockAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while (FMC->ISPTRG) ;

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        i32Err = -1;
    }
	
	FMC->ISPCMD = 0;
	
    return i32Err;
}

/**
  * @brief      Execute FMC_ISPCMD_BANK_ERASE command to erase a APROM bank. The bank size is 16 pages.
  * @param[in]  u32BankAddr Base address of the flash bank to be erased.
  * @return     ISP page erase success or not.
  * @retval     0  Success
  * @retval     -1  Erase failed
  * @note       FMC_ISPADDR[15:0] will be ignored.
  */
int32_t FMC_Erase_Bank(uint32_t u32BankAddr)
{
    int32_t i32Err = 0;
	
	FMC->ISPCMD = FMC_ISPCMD_BANK_ERASE;
    FMC->ISPADDR = u32BankAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
     while (FMC->ISPTRG) ;

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        i32Err = -1;
    }
	
	FMC->ISPCMD = 0;
	
    return i32Err;
}

/**
  * @brief    Get the current boot source
  * @param    None
  * @retval   0 This chip is currently booting from APROM
  * @retval   1 This chip is currently booting from LDROM
  * @note     This function only show the boot source.
  *           User need to read ISPSTA register to know if IAP mode supported or not in relative boot.
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
  * @param    None
  * @return   None
  * @details  ISPEN bit of ISPCTL must be set before we can use ISP commands.
  *           Therefore, To use all FMC function APIs, user needs to call FMC_Open() first to enable ISP functions.
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  *
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}


/**
 * @brief       Read 32-bit Data from specified address of flash
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @return      The data of specified address
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 */
uint32_t FMC_Read(uint32_t u32Addr)
{
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while(FMC->ISPTRG);

    return FMC->ISPDAT;

}

/**
 * @brief       Read 64-bit Data from specified address of flash
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[out]  u32data0  Save data0 of ISPADDR address
 * @param[out]  u32data1  Save data1 of ISPADDR+4 address
 * @return      None
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 */
void FMC_Read_64(uint32_t u32Addr, uint32_t * u32data0, uint32_t * u32data1)
{ 
	FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = u32Addr;
	FMC->ISPDAT	= 0x0;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
	while(FMC->ISPTRG);
	
	*u32data0 = FMC->MPDAT0;
	*u32data1 = FMC->MPDAT1;
}

/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit)
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *
  */
uint32_t FMC_ReadCID(void)
{

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADDR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}

/**
  * @brief    Read product ID
  * @param    None
  * @return   The product ID (32-bit)
  * @details  This function is used to read product ID.
  *
  */
uint32_t FMC_ReadPID(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_PID;
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}


/**
  * @brief      To read UCID
  * @param[in]  u32Index    Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return     The UCID of specified index
  * @details    This function is used to read unique customer ID (UCID).
  *
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif   
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}


/**
 * @brief       Read Unique ID
 * @param[in]   u32Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 * @return      The 32-bit unique ID data of specified UID index.
 * @details     To read out 96-bit Unique ID.
 *
 */
uint32_t FMC_ReadUID(uint32_t u32Index)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = 0x04 * u32Index;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    return FMC->ISPDAT;
}

/**
  * @brief    Get the base address of Data Flash if enabled.
  * @param    None
  * @return   The base address of Data Flash
  * @details  This function is used to return the base address of Data Flash.
  *
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}

/**
 * @brief       Set vector mapping address
 * @param[in]   u32PageAddr  The page address to remap to address 0x0. The address must be page alignment.
 * @return      To set VECMAP to remap specified page address to 0x0.
 * @details     This function is used to set VECMAP to map specified page to vector page (0x0).
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 *
 */
void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif  
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
}

/**
 * @brief      Program 32-bit data into specified address of flash
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data  32-bit Data to program
 * @return     None
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 */
void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while(FMC->ISPTRG);
}

/**
 * @brief      Program 64-bit data into specified address of flash
 * @param[in]  u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32data0 32-bit Data to program
 * @param[in]  u32data1 32-bit Data to program
 * @return     None
 * @details    To program two words data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 */
void FMC_Write_64(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1)
{
	FMC->ISPCMD = FMC_ISPCMD_PROGRAM_64;
    FMC->ISPADDR	= u32addr;
	FMC->MPDAT0 = u32data0;
	FMC->MPDAT1 = u32data1;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif	
	while(FMC->ISPTRG);
}

/**
* @brief       Read the User Configuration words.
* @param[out]  u32Config  The word buffer to store the User Configuration data.
* @param[in]   u32Count   The word count to be read.
* @retval       0 Success
* @retval      -1 Failed
* @details     This function is used to read the settings of user configuration.
*              if u32Count = 1, Only CONFIG0 will be returned to the buffer specified by u32Config.
*              if u32Count = 2, Both CONFIG0 and CONFIG1 will be returned.
 */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
	int32_t i;

    for(i = 0; i < u32Count; i++)
        u32Config[i] = FMC_Read(FMC_CONFIG_BASE + i * 4);

	return 0;
}

/**
 * @brief      Write User Configuration
 * @param[in]  u32Config The word buffer to store the User Configuration data.
 * @param[in]  u32Count The word count to program to User Configuration.
 * @retval     0 Success
 * @retval    -1 Failed
 * @details    User must enable User Configuration update before writing it.
 *             User must erase User Configuration before writing it.
 *             User Configuration is also be page erase. User needs to backup necessary data
 *             before erase User Configuration.
 */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{
	int32_t i;

    for(i = 0; i < u32Count; i++)
    {
        FMC_Write(FMC_CONFIG_BASE + i * 4, u32Config[i]);
        if(FMC_Read(FMC_CONFIG_BASE + i * 4) != u32Config[i])
            return -1;
    }

    return 0;
}


/**
 * @brief 		Run CRC32 checksum calculation and get result.
 * @param[in] 	u32addr   Starting flash address. It must be a page aligned address.
 * @param[in]	u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
 * @return		Success or not.
 * @retval 		>0          Success.
 * @retval 		==0         Address and size is over device limitation.
 * @retval   	0xFFFFFFFF  Invalid parameter.
 */
uint32_t  FMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
	//if ((u32addr % 512) || (u32count % 512))
	//	return 0xFFFFFFFF;
	if ((u32addr & 0x000001ff) || (u32count<512))
		return 0xFFFFFFFF;
	FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = u32addr;
	FMC->ISPDAT	 = u32count;
	FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
#if ISBEN
    __ISB();
#endif
    while(FMC->ISPTRG);
	
	FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPTRG);

	return FMC->ISPDAT;
}


/**
 * @brief	  Run flash all one verification and get result.
 * @param[in] u32addr   Starting flash address. It must be a page aligned address.
 * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
 * @retval    READ_ALLONE_YES      The contents of verified flash area are 0xFFFFFFFF.
 * @retval    READ_ALLONE_NOT  Some contents of verified flash area are not 0xFFFFFFFF.
 * @retval    READ_ALLONE_CMD_FAIL  Unexpected error occurred.
 */
uint32_t  FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
	FMC->ISPSTS = 0x80;   // clear check alll one bit
	
	FMC->ISPCMD = FMC_ISPCMD_RUN_ALL1;
    FMC->ISPADDR	= u32addr;
	FMC->ISPDAT	= u32count;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
	
 	while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) ;

	do {
		FMC->ISPCMD = FMC_ISPCMD_READ_ALL1;
    	FMC->ISPADDR	= u32addr;
		FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
		while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) ;
	}  while (FMC->ISPDAT == 0);

	if ((FMC->ISPDAT == FMC_READ_ALLONE_YES) || (FMC->ISPDAT == FMC_READ_ALLONE_NOT))
		return FMC->ISPDAT;
		
	return FMC_READ_ALLONE_CMD_FAIL;
}

/*@}*/ /* end of group I94100_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_FMC_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


