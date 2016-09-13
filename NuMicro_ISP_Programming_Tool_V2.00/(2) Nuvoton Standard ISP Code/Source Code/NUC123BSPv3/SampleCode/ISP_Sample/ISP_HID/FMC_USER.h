#ifndef FMC_USER_H
#define FMC_USER_H

#include "targetdev.h"

// #define FMC_CONFIG0_ADDR        (FMC_CONFIG_BASE)       /*!< CONFIG 0 Address */
// #define FMC_CONFIG1_ADDR        (FMC_CONFIG_BASE + 4)   /*!< CONFIG 1 Address */

#define Config0         FMC_CONFIG_BASE
#define Config1         (FMC_CONFIG_BASE+4)

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


extern void EraseAP(unsigned int addr_start, unsigned int addr_end);
extern void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int* data);
extern void WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);
extern void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
int FMC_Write_User(unsigned int u32Addr, unsigned int u32Data);
int FMC_Read_User(unsigned int u32Addr, unsigned int * data);
int FMC_Erase_User(unsigned int u32Addr);

#endif

