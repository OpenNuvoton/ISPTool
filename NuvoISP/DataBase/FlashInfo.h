#ifndef INC__FLASH_INFO_H__
#define INC__FLASH_INFO_H__
#pragma once

#include <stdio.h>
#include <sstream>
#include <iomanip>
#include "ChipDefs.h"
#include "IGetChipInformation.h"

#define CHIP_INDEX_NUC200           1
#define CHIP_INDEX_M051             2
#define CHIP_INDEX_I9160            6
#define CHIP_INDEX_M451             8
#define CHIP_INDEX_NUC029           10
#define CHIP_INDEX_NUC505           13
#define CHIP_INDEX_N575             19
#define CHIP_INDEX_N570             20
#define CHIP_INDEX_OT8051           23
#define CHIP_INDEX_M0564            24
#define CHIP_INDEX_I91000           27
#define CHIP_INDEX_NUC126           29
#define CHIP_INDEX_I91200           30
#define CHIP_INDEX_N569             31
#define CHIP_INDEX_M2351            35
#define CHIP_INDEX_JNK561           37
#define CHIP_INDEX_M031             36
#define CHIP_INDEX_M261             42
#define CHIP_INDEX_NUC1311          45
#define CHIP_INDEX_M2354            47
#define CHIP_INDEX_M0A21            48
#define CHIP_INDEX_M030G            49
#define CHIP_INDEX_M471             51
#define CHIP_INDEX_M071             54
#define CHIP_INDEX_KM1M7AB          56
#define CHIP_INDEX_N574             57
#define CHIP_INDEX_I91500           58
#define CHIP_INDEX_N32F030          59
#define CHIP_INDEX_M460             60
#define CHIP_INDEX_NUC1263          61
#define CHIP_INDEX_NM1800           62
#define CHIP_INDEX_KM1M4B           63
#define CHIP_INDEX_KM1M7C           64
#define CHIP_INDEX_M091             65
#define CHIP_INDEX_M2L31            66
#define CHIP_INDEX_M2003            67
#define CHIP_INDEX_NSC74            68
#define CHIP_INDEX_M0A86            69
#define CHIP_INDEX_KM1M0D           70
#define CHIP_INDEX_M55M1            71
#define CHIP_INDEX_KM1M0G           72
#define CHIP_INDEX_M433             73
#define CHIP_INDEX_M2A23            74
#define CHIP_INDEX_CM1003           75
#define CHIP_INDEX_CM2003           76
#define CHIP_INDEX_N79E855          77
#define CHIP_INDEX_NSC128           78
#define CHIP_INDEX_M3331            79
#define CHIP_INDEX_M2U51            80
#define CHIP_INDEX_M5531			81
#define CHIP_INDEX_CM2U51			82
#define CHIP_INDEX_CM3031			83
#define CHIP_INDEX_UNKNOWN          9999


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
    unsigned int    uFlash_PageSize;
    unsigned int    uAPROM_Addr;
    unsigned int    uAPROM_Size;
    unsigned int    uAPROM1_Addr;
    unsigned int    uAPROM1_Size;
    unsigned int    uNVM_Addr;
    unsigned int    uNVM_Size;
    unsigned int    uLDROM_Addr;
    unsigned int    uLDROM_Size;
    unsigned int    uSPROM_Addr;
    unsigned int    uSPROM_Size;
    unsigned int    uCONFIG_Addr;
    unsigned int    uCONFIG_Size;
    unsigned int    uOTP_Addr;
    unsigned int    uOTP_Count;             // 64-Bit Data & 32-bit Lock Bit
    unsigned int    uDATAROM_Addr;          // DFMC
    unsigned int    uDATAROM_Size;
    unsigned int    uDATAROM_PageSize;
    unsigned int    uAPWPROT_RegionNum;
    unsigned int    uAPWPROT_RegionSize;
    unsigned int    uSPIM_Addr;
    unsigned int    uSPIM_Size;
    unsigned int    uConfigValue_s;         // Chip issue
    bool            bShareDataFlash;
    bool            bAdvanceLock;
    bool            bKeyStoreLock;
    bool            bBootSelect;            // ISPCTL BS
    bool            bBankRemap;
    bool            bDualBank;
    unsigned int    uXOM_Num;
    unsigned int    uXOM_RangeStart;
    unsigned int    uXOM_RangeEnd;
    unsigned int    uXOM_MaxCount;
    unsigned int    uMassEraseTime;         // Mass Erase Time in ms + ?% Tolerance
    unsigned int    uPageEraseTime;         // Page Erase Time in ms + ?% Tolerance
    //unsigned int  uXOMErase;
    unsigned int    uIOVoltageFlag;
    unsigned int    uChipClock;
    unsigned int    uChipIndex;             // ICP Tool
    unsigned int    uMemoryType;            // 0: Flash, 1: RRAM
    unsigned int    uRAMSizeForLoad;
    unsigned int    uKS_OTPKeyCount;
    unsigned int    uKS_OTPKeyAttribute;    // Bit 0-7: Key Owner, Bit 8: Secure, Bit 9: Privilege, Bit 10: Readable, Bit 11: Boot State, Bit 12: MTP
    unsigned int    uKS_ROTPKCount;
    unsigned int    uKS_ROTPKAttribute;     // ROTPK (Root of Trust Public Key)
    const char      *pszSeriesName;
} CHIP_INFO_NUMICRO_T;

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
    unsigned int    uNVM_Addr;
    unsigned int    uNVM_Size;
    unsigned int    uCONFIG_Addr;
    unsigned int    uCONFIG_Size;
    unsigned int    uOTP_Addr;
    unsigned int    uOTP_Count;             // 64-Bit Data & 32-bit Lock Bit
    unsigned int    uSOTP_Addr;
    unsigned int    uSOTP_Size;
    unsigned int    uSPIM_Addr;
    unsigned int    uSPIM_Size;
    unsigned int    uITCM_Size;
    unsigned int    uDTCM_Size;
    unsigned int    uFlash_PageSize;
    unsigned int    uSRAM_BlockSize_L;      // Bit 0-15
    unsigned int    uSRAM_BlockSize_H;      // Bit 16-31
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
    unsigned int    uKS_OTPKeyAttribute;    // Bit 0-7: Key Owner, Bit 8: Secure, Bit 9: Privilege, Bit 10: Readable, Bit 11: Boot State, Bit 12: MTP
    unsigned int    uKS_ROTPKCount;
	unsigned int	uKS_ROTPKAttribute;			// P-256 ROTPK (Root of Trust Public Key)
	unsigned int	uKS_ROTPKAttribute_P521;	// P-521 ROTPK
    BOOL            bBankRemap;
    BOOL            bDualBank;
    BOOL            bMirrorBoundary;
    BOOL            bDPM;
    BOOL            bDIU;
    BOOL            bPLM;
    BOOL            bNPU;
    unsigned int    uMassEraseTime;         // Mass Erase Time in ms + ?% Tolerance
    unsigned int    uPageEraseTime;         // Page Erase Time in ms + ?% Tolerance
    unsigned int    uIOVoltageFlag;         // Bit 0: 1.8V, Bit 1: 2.5V, Bit 2: 3.3V, Bit 3: 5.0V
    unsigned int    uAlgoLoad_RAMAddr;
    unsigned int    uAlgoLoad_RAMSize;
    unsigned int    uAlgoLoad_ChipClock;
    unsigned int    uChipIndex;             // ICP Tool
    const char      *pszSeriesName;
} CHIP_INFO_NUMICRO_TZ_T;


/* Get Chip Series Info by Part No. */
typedef struct
{
    unsigned int uMixedSeriesName;
    CString  sTitlesName;
    unsigned int uPartNoSelectIndex;
    unsigned int uRealSeriesIndex;
} SERIES_INFO_BY_PARTNO_T;

void GetFlashSize_NuMicro(unsigned int uConfig0,
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

/* 8051 4T Series */
void GetFlashSize_4T(unsigned int uConfig0,
                     unsigned int uProgramMemorySize,
                     unsigned int *puAPROM_Size,
                     unsigned int *puNVM_Size);


/* NuVoice */
bool GetInfo_AU9100(//unsigned int uDID,
    unsigned int uConfig0,
    unsigned int uConfig1,
    unsigned int uProgramMemorySize,
    unsigned int uFlashType,
    unsigned int *puNVM_Addr,
    unsigned int *puLDROM_Size,
    unsigned int *puAPROM_Size,
    unsigned int *puNVM_Size);

#endif

