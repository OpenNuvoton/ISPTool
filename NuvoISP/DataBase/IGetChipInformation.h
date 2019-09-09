#pragma once

#include <windows.h>

#define GETCHIPINFO_LIBRARY_NAME	_T("GetChipInformation.dll")
#define CREATE_CHIP_MANAGER_NAME	_T("CreateChipInfoManager")
#define CHIP_NAME_LEN				100
#define GROUP_NAME_LEN				100
#define CHIP_UCID_MAX_LEN			4
#define CHIP_CFG_MAX_LEN			4

enum eAllChipSeries : DWORD {
    ISD_94000_SERIES			= 0x100001,
    ISD_91200_SERIES			= 0x100002,		//I91200/N573
    ISD_9160_SERIES				= 0x100003,		//I9160/N575
    ISD_91300_SERIES			= 0x100004,
    ISD_91000_SERIES			= 0x100005,
    NPCx_SERIES					= 0x100006,
    ISD_96000_SERIES			= 0x100007,
    NUVOICE_N572F064_SERIES		= 0x200001,
    NUVOICE_N572F072_SERIES		= 0x200002,
    NUVOICE_N571_SERIES			= 0x300001,
    NUVOICE_N569_SERIES			= 0x400001,
    NUVOICE_N570_SERIES			= 0x500001,		//I91032/N570
    NUVOICE_N570H_SERIES		= 0x500002,
    NUVOICE_N575_SERIES			= 0x600001,
    NUVOICE_N576_SERIES			= 0x700001,
    NUVOICE_JNK561_SERIES		= 0x800001,
    N589A_SERIES				= 0x900001,
    N589B_SERIES				= 0x900002,
    N589C_SERIES				= 0x900003,
    N589D_SERIES				= 0x900004,
    N589D171_SERIES				= 0x900005,
    NSP_SERIES					= 0x900006,
    NSP171_SERIES				= 0x900007,
    UnKnow_SERIES				= 0xFFFFFF,
};

typedef struct sChipInfo {
    DWORD	dwChipID;
    DWORD	dwChipIDMask;
    DWORD	dwChipUCID[CHIP_UCID_MAX_LEN];
    char	sChipName[CHIP_NAME_LEN];
    DWORD	dwSeriesEnum;
    DWORD	dwAPROMSize;
    DWORD	dwLDROMSize;
    DWORD	dwEmbeedSPIFlashSize;
    DWORD	dwSRAMSize;
    DWORD	dwErasePageSize;
    DWORD	dwDataFlashAddress;
    DWORD	dwDataFlashSize;
} sChipInfo;

typedef struct sFindChipInfo {
    DWORD	dwChipID;
    DWORD	dwCfgNum;
    DWORD	dwConfig[CHIP_CFG_MAX_LEN];
    DWORD	dwChipUCID[CHIP_UCID_MAX_LEN];
    BOOL	bSkipUCID;
    sFindChipInfo()
    {
        memset(this, 0, sizeof(sFindChipInfo));

        for (int i = 0; i < CHIP_CFG_MAX_LEN; ++i) {
            dwConfig[i] = 0xFFFFFFFF;
        }

        for (int i = 0; i < CHIP_UCID_MAX_LEN; ++i) {
            dwChipUCID[i] = 0xFFFFFFFF;
        }
    }
} sFindChipInfo;

enum eChipInfoError {
    ECE_NO_ERROR,
    ECE_NO_CHIP_CFG_DATA,
    ECE_NO_MATCH_CHIP,
    ECE_NO_MATCH_SERIES,
    ECE_EXPORT_FAILED,
    ECE_CFG_NO_CFG,
    ECE_CFG_DATAFLASH_ADDRESS_ERROR,
    ECE_USE_DEFAULT_CHIP,
    ECE_XLSLIB_NOT_EXIST,
    ECE_FILE_NOT_EXIST,
};

class I_ChipInfoManager
{
public:
    virtual void ReleaseDLL() = 0;
    virtual eChipInfoError GetChipInfo(DWORD dwChipID, sChipInfo &GetChipInfo, DWORD *dwCfg = NULL) = 0;
    virtual eChipInfoError GetChipInfoByFindInfo(const sFindChipInfo &FindChipInfo, sChipInfo &GetChipInfo, DWORD dwDID = 0) = 0;
    // virtual eChipInfoError ExportChipInfo(CString csPath) = 0;
    // virtual eChipInfoError CheckChipFromXLS(CString csXLSPath) = 0;
};

typedef BOOL (FAR WINAPI *CREATE_CHIPINFO_MANAGER)(I_ChipInfoManager **pIGetChipInfoManager);
