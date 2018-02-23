/**************************************************************************//**
 * @file     fmc.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/01/13 2:52p $
 * @brief    I91200 Flash Memory Controller Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_FMC_Driver FMC Driver
  @{
*/

/** @addtogroup I91200_FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM Base Address           \hideinitializer */
#define FMC_APROM_END           0x0001ffffUL    /*!< APROM End Address            \hideinitializer */
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM Base Address           \hideinitializer */
#define FMC_LDROM_END           0x00100fffUL    /*!< LDROM End Address            \hideinitializer */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< User Configuration Address   \hideinitializer */

#define FMC_FLASH_PAGE_SIZE     0x200           /*!< Flash Page Size (512 bytes)   \hideinitializer */
#define FMC_LDROM_SIZE          0x1000          /*!< LDROM Size (4 Kbytes)        \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00            /*!< ISP Command: Read flash word          \hideinitializer */
#define FMC_ISPCMD_WRITE        0x21            /*!< ISP Command: Write flash word         \hideinitializer */
#define FMC_ISPCMD_PAGE_ERASE   0x22            /*!< ISP Command: Page Erase Flash         \hideinitializer */
#define FMC_ISPCMD_READ_CID     0x0B            /*!< ISP Command: Read Company ID          \hideinitializer */
#define FMC_ISPCMD_READ_DID     0x0C            /*!< ISP Command: Read Device ID           \hideinitializer */

#define IS_BOOT_FROM_APROM      0               /*!< Is booting from APROM                 \hideinitializer */
#define IS_BOOT_FROM_LDROM      1               /*!< Is booting from LDROM                 \hideinitializer */

/*@}*/ /* end of group I91200_FMC_EXPORTED_CONSTANTS */


/** @addtogroup I91200_FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#define FMC_SET_APROM_BOOT()        (FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk)         /*!< Select booting from APROM   \hideinitializer */
#define FMC_SET_LDROM_BOOT()        (FMC->ISPCTL |= FMC_ISPCTL_BS_Msk)          /*!< Select booting from LDROM   \hideinitializer */
#define FMC_ENABLE_CFG_UPDATE()     (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk)     /*!< Enable User Config update   \hideinitializer */
#define FMC_DISABLE_CFG_UPDATE()    (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk)     /*!< Disable User Config update  \hideinitializer */
#define FMC_ENABLE_LD_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)      /*!< Enable LDROM update         \hideinitializer */
#define FMC_DISABLE_LD_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)      /*!< Disable LDROM update        \hideinitializer */
#define FMC_DISABLE_ISP()           (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)      /*!< Disable ISP function        \hideinitializer */
#define FMC_ENABLE_ISP()            (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)      /*!< Enable ISP function         \hideinitializer */
#define FMC_DISABLE_APROM_WRITABLE()           (FMC->ISPCTL &= ~FMC_ISPCTL_APUWEN_Msk)      /*!< APROM can’t write itself        \hideinitializer */
#define FMC_ENABLE_APROM_WRITABLE()            (FMC->ISPCTL |=  FMC_ISPCTL_APUWEN_Msk)      /*!< APROM can write to itself         \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

extern void FMC_SetBootSource(int32_t i32BootSrc);
extern void FMC_Close(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadDID(void);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


/*@}*/ /* end of group I91200_FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_FMC_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
