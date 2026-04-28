#ifndef INC__FLASH_INFO_H__
#define INC__FLASH_INFO_H__
#pragma once

#include <stdio.h>
#include <sstream>
#include <iomanip>
#include "ChipDefs.h"
#include "IGetChipInformation.h"


/* Get Flash Info by PID */
typedef struct
{
    unsigned int uProgramMemorySize;
    unsigned int uDataFlashSize;
    unsigned int uRAMSize;
    unsigned int uDataFlashStartAddress;
    unsigned int uISPFlashSize;
    unsigned int uPID;
    unsigned int uFlashType;
} FLASH_PID_INFO_BASE_T;

void *GetInfo(unsigned int uPID,
              FLASH_PID_INFO_BASE_T *pInfo);


/* Get Flash Info by DID */
typedef struct
{
    unsigned int uProgramMemorySize;
    unsigned int uLDSize;
    unsigned int uRAMSize;
    unsigned int uDID;
    union
    {
        unsigned int uFlashType;
        struct
        {
            unsigned int uFlashMode : 2;
            unsigned int uFlashPageSize : 2;
            unsigned int bSupportPID : 1;
            unsigned int bSupportUID : 1;
            unsigned int bSupportUCID : 1;
            unsigned int bSupportHIRCOff : 1;
            unsigned int uIOVoltage : 4;
            unsigned int uReserved0 : 12;
            unsigned int bSupportSCode : 1;
            unsigned int bSupportCRC8 : 1;
            unsigned int uReserved1 : 5;
            unsigned int uCoreType : 1;
        } feature;
    };
} FLASH_INFO_BY_DID_T;

typedef struct : FLASH_INFO_BY_DID_T
{
    unsigned int    uAPROM_Addr;
    unsigned int    uAPROM_Size;
    unsigned int    uAPROM1_Addr;
    unsigned int    uAPROM1_Size;
    unsigned int    uAPROMNS_Addr;
    unsigned int    uAPROMNS_Size;
    unsigned int    uAPROMNS1_Addr;
    unsigned int    uAPROMNS1_Size;
    unsigned int    uLDROM_Addr;
    unsigned int    uLDROM_Size;
    unsigned int    uNVM_Addr;                    // FMC
    unsigned int    uNVM_Size;
    unsigned int    uDATAROM_Addr;                // DFMC
    unsigned int    uDATAROM_Size;
    unsigned int    uDATAROMNS_Addr;
    unsigned int    uDATAROMNS_Size;
    unsigned int    uEEPROM_Addr;
    unsigned int    uEEPROM_Size;
    unsigned int    uEEPROMNS_Addr;
    unsigned int    uEEPROMNS_Size;
    unsigned int    uCONFIG_Addr;
    unsigned int    uCONFIG_Size;
    unsigned int    uOTP_Addr;
    unsigned int    uOTP_Count;                    // 64-Bit Data & 32-bit Lock Bit
    unsigned int    uSOTP_Addr;
    unsigned int    uSOTP_Size;
    unsigned int    uSPIM_Addr;
    unsigned int    uSPIM_Size;
    unsigned int    uITCM_Size;
    unsigned int    uDTCM_Size;
    unsigned int    uFlash_PageSize;
    unsigned int    uDATAROM_PageSize;
    unsigned int    uSRAM_BlockSize_L;            // Bit 0-15
    unsigned int    uSRAM_BlockSize_H;            // Bit 16-31
    unsigned int    uISPSTS_VECMAP_Msk;
    unsigned int    uAPWPROT_RegionNum;
    unsigned int    uAPWPROT_RegionSize;
    unsigned int    uLDWPROT_RegionSize;
    unsigned int    uDFWPROT_RegionSize;
    unsigned int    uUID_Num;
    unsigned int    uXOM_Num;
    unsigned int    uXOM_RangeStart;
    unsigned int    uXOM_RangeEnd;
    unsigned int    uXOM_MaxCount;
    unsigned int    uKS_OTPKeyCount;
    unsigned int    uKS_OTPKeyAttribute;        // Bit 0-7: Key Owner, Bit 8: Secure, Bit 9: Privilege, Bit 10: Readable, Bit 11: Boot State, Bit 12: MTP
    unsigned int    uKS_ROTPKCount;
    unsigned int    uKS_ROTPKAttribute;            // P-256 ROTPK (Root of Trust Public Key)
    unsigned int    uKS_ROTPKAttribute_P521;    // P-521 ROTPK
    BOOL            bBankRemap;
    BOOL            bDualBank;
    BOOL            bMirrorBoundary;
    BOOL            bDPM;
    BOOL            bDIU;
    BOOL            bPLM;
    BOOL            bNPU;
    BOOL            bDFMC_NSCBA;
    BOOL            bDFMC_EEPROM;
    unsigned int    uMassEraseTime;                // Mass Erase Time in ms + ?% Tolerance
    unsigned int    uPageEraseTime;                // Page Erase Time in ms + ?% Tolerance
    unsigned int    uIOVoltageFlag;                // Bit 0: 1.8V, Bit 1: 2.5V, Bit 2: 3.3V, Bit 3: 5.0V
    unsigned int    uAlgoLoad_RAMAddr;
    unsigned int    uAlgoLoad_RAMSize;
    unsigned int    uAlgoLoad_ChipClock;
    unsigned int    uChipIndex;                    // ICP Tool
    const char        *pszSeriesName;
} CHIP_INFO_NUMICRO_TZ_T;

bool GetFlashSize_NuMicro(unsigned int uConfig0,
                          unsigned int uConfig1,
                          unsigned int uProgramMemorySize,
                          unsigned int uFlashType,
                          unsigned int uAPROM_Addr,
                          unsigned int *puNVM_Addr,
                          unsigned int *puAPROM_Size,
                          unsigned int *puNVM_Size);


/* 8051 1T Series */
void *GetInfo_8051_1T(unsigned int uDID, FLASH_INFO_BY_DID_T *pInfo);

void GetFlashSize_OT8051(unsigned int uConfig0,
                         unsigned int uProgramMemorySize,
                         unsigned int uFlashMode,
                         unsigned int *puLDROM_Addr,
                         unsigned int *puLDROM_Size,
                         unsigned int *puAPROM_Size,
                         unsigned int *puNVM_Size);

#endif

