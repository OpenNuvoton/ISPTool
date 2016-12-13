/**************************************************************************//**
 * @file     FMC.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/05/26 12:39p $
 * @brief    Mini58 series FMC driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif
    
/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_FMC_Driver FMC Driver
  @{
*/

/** @addtogroup Mini58_FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM  Base Address         */  
#define FMC_APROM_END           0x00008000UL    /*!< APROM  End Address          */  
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM  Base Address         */
#define FMC_LDROM_END           0x00100A00UL    /*!< LDROM  End Address          */
#define FMC_SPROM_BASE          0x00200000UL    /*!< SPROM  Base Address         */
#define FMC_SPROM_END           0x00200200UL    /*!< SPROM  End Address          */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< CONFIG Base Address         */

#define FMC_FLASH_PAGE_SIZE     0x200           /*!< Flash Page Size (512 Bytes) */
#define FMC_LDROM_SIZE          0xA00           /*!< LDROM Size (2 Kbytes)       */
#define FMC_SPROM_SIZE          0x200           /*!< SPROM Size (512 bytes)      */


/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00            /*!< ISP Command: Read Flash                     */
#define FMC_ISPCMD_READ_UID     0x04            /*!< ISP Command: Read Unique ID                 */
#define FMC_ISPCMD_READ_CID     0x0B            /*!< ISP Command: Read Company ID                */
#define FMC_ISPCMD_READ_PID     0x0C            /*!< ISP Command: Read Product ID                */
#define FMC_ISPCMD_READ_CRC32   0x0D            /*!< ISP Command: Read CRC32 checksum            */
#define FMC_ISPCMD_PROGRAM      0x21            /*!< ISP Command: Program Flash                  */
#define FMC_ISPCMD_PAGE_ERASE   0x22            /*!< ISP Command: Page Erase Flash               */
#define FMC_ISPCMD_CAL_CRC32    0x2D            /*!< ISP Command: Run CRC32 checksum calculation */
#define FMC_ISPCMD_VECMAP       0x2E            /*!< ISP Command: Vector Page Remap              */


/*@}*/ /* end of group Mini58_FMC_EXPORTED_CONSTANTS */

/** @addtogroup Mini58_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

#define FMC_SET_APROM_BOOT()        (FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk)         /*!< Select booting from APROM  */
#define FMC_SET_LDROM_BOOT()        (FMC->ISPCTL |= FMC_ISPCTL_BS_Msk)          /*!< Select booting from LDROM  */
#define FMC_DISABLE_AP_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)      /*!< Disable APROM update  */
#define FMC_DISABLE_SP_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_SPUEN_Msk)      /*!< Disable SPROM update  */
#define FMC_DISABLE_CFG_UPDATE()    (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk)     /*!< Disable User Config update  */
#define FMC_DISABLE_LD_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)      /*!< Disable LDROM update  */
#define FMC_DISABLE_ISP()           (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)      /*!< Disable ISP function  */
#define FMC_ENABLE_AP_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)      /*!< Enable APROM update  */
#define FMC_ENABLE_SP_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_SPUEN_Msk)      /*!< Enable SPROM update  */
#define FMC_ENABLE_LD_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)      /*!< Enable LDROM update  */
#define FMC_ENABLE_CFG_UPDATE()     (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk)     /*!< Enable User Config update  */
#define FMC_ENABLE_ISP()            (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)      /*!< Enable ISP function  */
#define FMC_GET_FAIL_FLAG()         ((FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) ? 1 : 0)  /*!< Get ISP fail flag  */


extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read (uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern uint32_t FMC_GetVectorPageAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_GetCRC32Sum(uint32_t addr, uint32_t count, uint32_t *chksum);


/*@}*/ /* end of group Mini58_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_FMC_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif


#endif

