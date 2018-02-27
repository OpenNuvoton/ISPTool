#pragma once

#define GETCHIPINFO_LIBRARY_NAME	_T("GetChipInformation.dll")
#define CREATE_CHIP_MANAGER_NAME	_T("CreateChipInfoManager")
#define CHIP_NAME_LEN				100
#define GROUP_NAME_LEN				100

enum eAllChipSeries : DWORD {
    ISD_94000_SERIES	= 0x100001,
    ISD_91200_SERIES	= 0x100002,
    ISD_9160_SERIES		= 0x100003,
    ISD_91300_SERIES	= 0x100004,
    ISD_91000_SERIES	= 0x100005,
    NUVOICE_N572_SERIES = 0x200001,
    NUVOICE_N571_SERIES = 0x200002,
    NUVOICE_N569_SERIES = 0x200003,
    NUVOICE_N570_SERIES = 0x200004,
};

typedef struct sChipInfo {
    DWORD	dwChipID;
    DWORD	dwChipIDMask;
    char	sChipName[CHIP_NAME_LEN];
    char	sGroupName[GROUP_NAME_LEN];
    DWORD	dwSeriesEnum;
    DWORD	dwAPROMSize;
    DWORD	dwLDROMSize;
    DWORD	dwEmbeedSPIFlashSize;
    DWORD	dwSRAMSize;
    DWORD	dwErasePageSize;
    DWORD	dwDataFlashAddress;
    DWORD	dwDataFlashSize;
} sChipInfo;

class I_ChipInfoManager
{
public:
    virtual void Release() = 0;
    virtual BOOL GetChipInfoByID(DWORD dwChipID, sChipInfo &chipInfo, DWORD *pConfig = NULL, DWORD dwConfigNum = 0) = 0;
    virtual BOOL ExportChipInfo(CString csPath) = 0;
};

typedef BOOL (FAR WINAPI *CREATE_CHIPINFO_MANAGER)(I_ChipInfoManager **pIGetChipInfoManager);
