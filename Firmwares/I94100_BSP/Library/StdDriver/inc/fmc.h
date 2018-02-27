/**************************************************************************//**
 * @file     fmc.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/12/16 4:27p $
 * @brief    I94100 Series Flash Memory Controller Driver Header File
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup I94100_FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL   			/*!< APROM base address          */
#define FMC_APROM_END           0x0007FFFFUL   			/*!< APROM end address           */
#define FMC_LDROM_BASE          0x00100000UL    		/*!< LDROM base address          */
#define FMC_LDROM_END           0x00100FFFUL    		/*!< LDROM end address           */
#define FMC_CONFIG_BASE         0x00300000UL    		/*!< User Configuration address  */
#define FMC_USER_CONFIG_0       FMC_CONFIG_BASE 		/*!< User Config 0 address       */
#define FMC_USER_CONFIG_1       (FMC_CONFIG_BASE+4)    	/*!< User Config 1 address       */

#define FMC_FLASH_PAGE_SIZE     0x1000          /*!< Flash Page Size (4K bytes)  */
#define FMC_PAGE_ADDR_MASK      0xFFFFE000UL    /*!< Flash page address mask     */

#define FMC_APROM_SIZE          (FMC_APROM_END+1)   /*!< APROM Size                  */
#define FMC_LDROM_SIZE          0x1000       	    /*!< LDROM Size (4 Kbytes)       */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00            /*!< ISP Command: Read flash word         */
#define FMC_ISPCMD_READ_UID     0x04            /*!< ISP Command: Read Unique ID          */
#define FMC_ISPCMD_READ_ALL1    0x08            /*!< ISP Command: Read all-one result     */
#define FMC_ISPCMD_READ_CID     0x0B            /*!< ISP Command: Read Company ID         */
#define FMC_ISPCMD_READ_PID     0x0C            /*!< ISP Command: Read Product ID         */
#define FMC_ISPCMD_READ_CKS     0x0D            /*!< ISP Command: Read checksum           */
#define FMC_ISPCMD_PROGRAM      0x21            /*!< ISP Command: Write flash word        */
#define FMC_ISPCMD_PAGE_ERASE   0x22            /*!< ISP Command: Erase one page,4k bytes */
#define FMC_ISPCMD_BANK_ERASE   0x23            /*!< ISP Command: Erase 16 pages alignment of APROM*/
#define FMC_ISPCMD_BLOCK_ERASE  0x25            /*!< ISP Command: Erase 4 pages alignment of APROM*/
#define FMC_ISPCMD_PROGRAM_MUL  0x27            /*!< ISP Command: Multuple word program   */
#define FMC_ISPCMD_RUN_ALL1     0x28            /*!< ISP Command: Run all-one verification*/
#define FMC_ISPCMD_RUN_CKS      0x2D            /*!< ISP Command: Run checksum calculation*/
#define FMC_ISPCMD_VECMAP       0x2E            /*!< ISP Command: Vector Page Remap       */
#define FMC_ISPCMD_READ_64      0x40            /*!< ISP Command: Read double flash word  */
#define FMC_ISPCMD_PROGRAM_64   0x61            /*!< ISP Command: Write double flash word */


#define FMC_ISPCTL_BS_LDROM     0x1    			/*!< ISPCTL setting to select to boot from LDROM */
#define FMC_ISPCTL_BS_APROM     0x0    			/*!< ISPCTL setting to select to boot from APROM */

#define FMC_READ_ALLONE_YES         0xA11FFFFF      /*!< Check-all-one result is all one.     */         
#define FMC_READ_ALLONE_NOT         0xA1100000      /*!< Check-all-one result is not all one. */         
#define FMC_READ_ALLONE_CMD_FAIL    0xFFFFFFFF      /*!< Check-all-one command failed.        */


/*@}*/ /* end of group I94100_FMC_EXPORTED_CONSTANTS */


/** @addtogroup I94100_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief      Enable ISP Function
 * @param      None
 * @return     None
 * @details    This function will set ISPEN bit of ISPCTL control register to enable ISP function.
 *
 */
#define FMC_ENABLE_ISP()            (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)      /*!< Enable ISP function        */

/**
 * @brief      Disable ISP Function
 * @param      None
 * @return     None
 * @details    This function will clear ISPEN bit of ISPCTL control register to disable ISP function.
 *
 */
#define FMC_DISABLE_ISP()           (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)      /*!< Disable ISP function       */

/**
 * @brief      Enable LDROM Update Function
 * @param      None
 * @return     None
 * @details    This function will set LDUEN bit of ISPCTL control register to enable LDROM update function.
 *             User needs to set LDUEN bit before they can update LDROM.
 *
 */
#define FMC_ENABLE_LD_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)      /*!< Enable LDROM update        */

/**
 * @brief      Disable LDROM Update Function
 * @param      None
 * @return     None
 * @details    This function will set ISPEN bit of ISPCTL control register to disable LDROM update function.
 *
 */ 
#define FMC_DISABLE_LD_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)      /*!< Disable LDROM update       */

/**
 * @brief      Enable User Configuration Update Function
 * @param      None
 * @return     None
 * @details    This function will set CFGUEN bit of ISPCTL control register to enable User Configuration update function.
 *             User needs to set CFGUEN bit before they can update User Configuration area.
 *
 */
#define FMC_ENABLE_CFG_UPDATE()     (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk)     /*!< Enable User Config update  */

/**
 * @brief      Disable User Configuration Update Function
 * @param      None
 * @return     None
 * @details    This function will clear CFGUEN bit of ISPCTL control register to disable User Configuration update function.
 *
 */
#define FMC_DISABLE_CFG_UPDATE()    (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk)     /*!< Disable User Config update */

/**
 * @brief      Enable APROM Update Function
 * @param      None
 * @return     None
 * @details    This function will set APUEN bit of ISPCTL control register to enable APROM update function.
 *             User needs to set APUEN bit before they can update APROM in APROM boot mode.
 *
 */
#define FMC_ENABLE_AP_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)      /*!< Enable APROM update        */

/**
 * @brief      Disable APROM Update Function
 * @param      None
 * @return     None
 * @details    This function will clear APUEN bit of ISPCTL control register to disable APROM update function.
 *
 */
#define FMC_DISABLE_AP_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)      /*!< Disable APROM update       */

/**
 * @brief      Get ISP Fail Flag
 * @param      None
 * @return     None
 * @details    The fail flag is set by hardware when a triggered ISP meets any of the following conditions:
 *			   1. APROM writes to itself if APUEN is set to 0.
 *			   2. APROM writes to itself if APUEN is set to 0.
 *			   3. CONFIG is erased/programmed if CFGUEN is set to 0.
 *			   4. Destination address is illegal, such as over an available range.
 */
#define FMC_GET_FAIL_FLAG()         ((FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) ? 1 : 0)  /*!< Get ISP fail flag  */

/**
 * @brief      Clear ISP Fail Flag
 * @param      None
 * @return     None
 * @details    Write 1 to clear ISP fail flag
 *
 */
#define FMC_CLR_FAIL_FLAG()         (FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk)       /*!< Clear ISP fail flag        */

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

extern void FMC_SetBootSource(int32_t i32BootSrc);
extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_Erase_Block(uint32_t u32BlockAddr);
extern int32_t FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern void FMC_Read_64(uint32_t u32addr, uint32_t * u32data0, uint32_t * u32data1);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern void FMC_Write_64(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern uint32_t FMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);


/*@}*/ /* end of group I94100_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_FMC_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif   // __FMC_H__

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
