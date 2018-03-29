/**************************************************************************//**
 * @file     fmc.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/12/03 12:50p $
 * @brief    NANO103 Series Flash Memory Controller Driver Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_FMC_Driver FMC Driver
  @{
*/


/** @addtogroup NANO103_FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/


/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM Base Address          */
#define FMC_APROM_END           0x00010000UL    /*!< APROM End Address           */
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM Base Address          */
#define FMC_LDROM_END           0x00101200UL    /*!< LDROM End Address           */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< User Configuration Address  */
#define FMC_KPROM_BASE          0x00301000UL    /*!< Security ROM base address   */

#define FMC_FLASH_PAGE_SIZE     0x200           /*!< Flash Page Size (512 bytes) */
#define FMC_PAGE_ADDR_MASK      0xFFFFFE00UL    /*!< Flash page address mask     */

#define FMC_LDROM_SIZE          0x1200          /*!< LDROM Size (4.5 Kbytes)     */


/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00            /*!< ISP Command: Read flash word         */
#define FMC_ISPCMD_PROGRAM      0x21            /*!< ISP Command: Write flash word        */
#define FMC_ISPCMD_PAGE_ERASE   0x22            /*!< ISP Command: Page Erase Flash        */
#define FMC_ISPCMD_READ_CID     0x0B            /*!< ISP Command: Read Company ID         */
#define FMC_ISPCMD_READ_PID     0x0C            /*!< ISP Command: Read Product ID         */
#define FMC_ISPCMD_READ_UID     0x04            /*!< ISP Command: Read Unique ID          */
#define FMC_ISPCMD_RUN_CKS      0x2D            /*!< ISP Command: Run checksum calculation*/
#define FMC_ISPCMD_READ_CKS     0x0D            /*!< ISP Command: Read checksum           */
#define FMC_ISPCMD_RUN_ALL1     0x28            /*!< ISP Command: Run all-one verification*/
#define FMC_ISPCMD_READ_ALL1    0x08            /*!< ISP Command: Read all-one result     */
#define FMC_ISPCMD_VECMAP       0x2E            /*!< ISP Command: Vector Page Remap       */

#define IS_BOOT_FROM_APROM      0               /*!< Is booting from APROM                */
#define IS_BOOT_FROM_LDROM      1               /*!< Is booting from LDROM                */

#define READ_ALLONE_YES         0xA11FFFFF      /*!< Check-all-one result is all one.     */
#define READ_ALLONE_NOT         0xA1100000      /*!< Check-all-one result is not all one. */
#define READ_ALLONE_CMD_FAIL    0xFFFFFFFF      /*!< Check-all-one command failed.        */


/*@}*/ /* end of group NANO103_FMC_EXPORTED_CONSTANTS */


/** @addtogroup NANO103_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#define FMC_SET_APROM_BOOT()        (FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk)         /*!< Select booting from APROM  */
#define FMC_SET_LDROM_BOOT()        (FMC->ISPCTL |= FMC_ISPCTL_BS_Msk)          /*!< Select booting from LDROM  */
#define FMC_ENABLE_AP_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)      /*!< Enable APROM update        */
#define FMC_DISABLE_AP_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)      /*!< Disable APROM update       */
#define FMC_ENABLE_CFG_UPDATE()     (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk)     /*!< Enable User Config update  */
#define FMC_DISABLE_CFG_UPDATE()    (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk)     /*!< Disable User Config update */
#define FMC_ENABLE_LD_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)      /*!< Enable LDROM update        */
#define FMC_DISABLE_LD_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)      /*!< Disable LDROM update       */
#define FMC_DISABLE_ISP()           (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)      /*!< Disable ISP function       */
#define FMC_ENABLE_ISP()            (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)      /*!< Enable ISP function        */
#define FMC_GET_FAIL_FLAG()         ((FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) ? 1 : 0)  /*!< Get ISP fail flag  */
#define FMC_CLR_FAIL_FLAG()         (FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk)       /*!< Clear ISP fail flag        */

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern uint32_t FMC_GetVectorPageAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t  FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t  FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t  FMC_GetChkSum(uint32_t u32Addr, uint32_t u32Count, uint32_t *u32ChkSum);
extern uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
extern int32_t  FMC_SKey_Setup(uint32_t key[3], uint32_t kpmax, uint32_t kemax, int lock_CONFIG);
extern int32_t  FMC_SKey_Compare(uint32_t key[3]);


/*@}*/ /* end of group NANO103_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_FMC_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif   // __FMC_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
