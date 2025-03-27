#pragma once

#define GETCHIPINFO_LIBRARY_NAME    _T("GetChipInformation.dll")
#define CREATE_CHIP_MANAGER_NAME    _T("CreateChipInfoManager")
#define CHIP_NAME_LEN               100
#define GROUP_NAME_LEN              100
#define CHIP_UCID_MAX_LEN           4
#define CHIP_CFG_MAX_LEN            4

enum eAllChipSeries : DWORD
{
    //NuVoice Series Chip
    ISD_94000_SERIES = 0x100001,
    ISD_91200_SERIES = 0x100002,        //I91200/N573
    ISD_9160_SERIES = 0x100003,     //I9160/N575
    ISD_91300_SERIES = 0x100004,
    ISD_91000_SERIES = 0x100005,
    NPCx_SERIES = 0x100006,
    ISD_96000_SERIES = 0x100007,
    ISD_91500_SERIES = 0x100008,
    NUVOICE_N572F064_SERIES = 0x200001,
    NUVOICE_N572F072_SERIES = 0x200002,
    NUVOICE_N571_SERIES = 0x300001,
    NUVOICE_N569_SERIES = 0x400001,
    NUVOICE_N569J_SERIES = 0x400002,
    NUVOICE_N570_SERIES = 0x500001,     //I91032/N570
    NUVOICE_N570H_SERIES = 0x500002,
    NUVOICE_N570J_SERIES = 0x500003,
    NUVOICE_N575_SERIES = 0x600001,
    NUVOICE_N574F128_SERIES = 0x600002,
    NUVOICE_N574F1K5_SERIES = 0x600003,
    NUVOICE_NSC74128_SERIES = 0x600004,
    NUVOICE_NSC741K5_SERIES = 0x600005,
    NUVOICE_N574J_SERIES = 0x600006,
    NUVOICE_NSC128L42_SERIES = 0x600007,
    NUVOICE_N576_SERIES = 0x700001,
    NUVOICE_N577_SERIES = 0x700002,
    NUVOICE_JNK561_SERIES = 0x800001,
    NUVOICE_JNK561H_SERIES = 0x800002,
    NUVOICE_NSR_SERIES = 0x800005,
    //Speech Series Chip
    N589A_SERIES = 0x900001,
    N589B_SERIES = 0x900002,
    N589C_SERIES = 0x900003,
    N589D_SERIES = 0x900004,
    NSP_SERIES = 0x900005,
    NSP_OTP_SERIES = 0x900006,
    NSC_SERIES = 0x900007,
    NC90_SERIES = 0x900008,
    N588_SERIES = 0x900009,
    N566_SERIES = 0x90000A,
    N589E_SERIES = 0x90000B,
    N584_SERIES = 0x90000C,
    N589L_SERIES = 0x90000D,
    NSP2_SERIES = 0x90000E,
    SPEECH_NSR_SERIES = 0x90000F,
    N589S_SERIES = 0x900010,
    N589LS_SERIES = 0x900011,
    UNKNOWN_SERIES = 0xFFFFFF,
};

typedef struct sSeriesName
{
    eAllChipSeries Series;
    CString Name;
} sSeriesName;

const sSeriesName g_SeriesToName[] =
{
    {ISD_94000_SERIES, _T("I94000")},
    {ISD_91200_SERIES, _T("I91200")},
    {ISD_9160_SERIES, _T("I9160")},
    {ISD_91300_SERIES, _T("I91300")},
    {ISD_91000_SERIES, _T("I91000")},
    {ISD_91500_SERIES, _T("I91500")},
    {NPCx_SERIES, _T("NPCx")},
    {ISD_96000_SERIES, _T("I96000")},
    {NUVOICE_N572F064_SERIES, _T("N572F064")},
    {NUVOICE_N572F072_SERIES, _T("N572F072")},
    {NUVOICE_N571_SERIES, _T("N571")},
    {NUVOICE_N569_SERIES, _T("N569")},
    {NUVOICE_N569J_SERIES, _T("N569J")},
    {NUVOICE_N570_SERIES, _T("N570")},
    {NUVOICE_N570H_SERIES, _T("N570H")},
    {NUVOICE_N570J_SERIES, _T("N570J")},
    {NUVOICE_N574F128_SERIES, _T("N574F128")},
    {NUVOICE_N574F1K5_SERIES, _T("N574F1K5")},
    {NUVOICE_N575_SERIES, _T("N575")},
    {NUVOICE_N576_SERIES, _T("N576")},
    {NUVOICE_N577_SERIES, _T("N577")},
    {NUVOICE_JNK561_SERIES, _T("JNK561")},
    {NUVOICE_JNK561H_SERIES, _T("JNK561H")},
    {NUVOICE_NSR_SERIES, _T("NSR")},
    {NUVOICE_NSC74128_SERIES, _T("NSC74128")},
    {NUVOICE_NSC741K5_SERIES, _T("NSC741K5")},
    {NUVOICE_NSC128L42_SERIES, _T("NSC128L42")},
    {NUVOICE_N574J_SERIES, _T("N574J")},
    {N589A_SERIES, _T("N589A")},
    {N589B_SERIES, _T("N589B")},
    {N589C_SERIES, _T("N589C")},
    {N589D_SERIES, _T("N589D")},
    {NSP_SERIES, _T("NSP")},
    {NSP2_SERIES, _T("NSP2")},
    {NSP_OTP_SERIES, _T("NSP_OTP")},
    {NSC_SERIES, _T("NSC")},
    {NC90_SERIES, _T("NC90")},
    {N588_SERIES, _T("N588")},
    {N566_SERIES, _T("N566")},
    {N589E_SERIES, _T("N589E")},
    {N584_SERIES, _T("N584")},
    {N589L_SERIES, _T("N589L")},
    {N589S_SERIES, _T("N589S")},
    {N589LS_SERIES, _T("N589LS")},
    {SPEECH_NSR_SERIES, _T("NSR")},
};

typedef struct sChipInfo
{
    DWORD   dwChipID;
    DWORD   dwChipIDMask;
    DWORD   dwChipUCID[CHIP_UCID_MAX_LEN];
    char    sChipName[CHIP_NAME_LEN];
    DWORD   dwSeriesEnum;
    DWORD   dwAPROMSize;
    DWORD   dwLDROMSize;
    DWORD   dwEmbeedSPIFlashSize;
    DWORD   dwSRAMSize;
    DWORD   dwErasePageSize;
    DWORD   dwDataFlashAddress;
    DWORD   dwDataFlashSize;
} sChipInfo;

typedef struct sFindChipInfo
{
    DWORD   dwChipID;
    DWORD   dwCfgNum;
    DWORD   dwConfig[CHIP_CFG_MAX_LEN];
    DWORD   dwChipUCID[CHIP_UCID_MAX_LEN];
    BOOL    bSkipUCID;
    sFindChipInfo()
    {
        memset(this, 0, sizeof(sFindChipInfo));

        for (int i = 0; i < CHIP_CFG_MAX_LEN; ++i)
            dwConfig[i] = 0xFFFFFFFF;

        for (int i = 0; i < CHIP_UCID_MAX_LEN; ++i)
            dwChipUCID[i] = 0xFFFFFFFF;
    }
} sFindChipInfo;

enum eChipInfoError
{
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
    virtual eChipInfoError GetChipInfo(DWORD dwChipID, sChipInfo& GetChipInfo, DWORD* dwCfg = NULL) = 0;
    virtual eChipInfoError GetChipInfoByFindInfo(const sFindChipInfo& FindChipInfo, sChipInfo& GetChipInfo, DWORD dwDID = 0) = 0;
    virtual eChipInfoError ExportChipInfo(CString csPath) = 0;
    virtual eChipInfoError CheckChipFromXLS(CString csXLSPath) = 0;
};

typedef BOOL(FAR WINAPI* CREATE_CHIPINFO_MANAGER)(I_ChipInfoManager** pIGetChipInfoManager);