#include "stdafx.h"
#include "NuDataBase.h"

#include "FlashInfo.h"

CChipConfigInfo gsChipCfgInfo;
struct sChipInfo gNuVoiceChip;

std::vector<CPartNumID> g_NuMicroChipSeries;
std::vector<CPartNumID> g_AudioChipSeries;


bool GetInfo_NuVoice(DWORD dwChipID, DWORD *pConfig)
{
    bool ret = false;
    memset(&gNuVoiceChip, 0, sizeof(sChipInfo));
    HMODULE hDll = ::LoadLibrary(_T("GetChipInformation.dll"));

    if (hDll != NULL) {
        CREATE_CHIPINFO_MANAGER pCreateChipInfoManager = (CREATE_CHIPINFO_MANAGER)::GetProcAddress(hDll, "CreateChipInfoManager");

        if (pCreateChipInfoManager) {
            I_ChipInfoManager *pChipInfoManager = NULL;

            if (pCreateChipInfoManager(&pChipInfoManager) == TRUE) {
                eChipInfoError err = pChipInfoManager->GetChipInfo(dwChipID, gNuVoiceChip, pConfig);

                if (err == ECE_NO_ERROR) {
                    ret = true;
                }

                pChipInfoManager->ReleaseDLL();
            }
        }

        FreeLibrary(hDll);
    }

    return ret;
}

extern struct CPartNumID g_PartNumIDs[];
bool GetChipConfigInfo(unsigned int uID)
{
    if (gsChipCfgInfo.uID == uID) {
        return true;
    } else {
        char pName[] = "Unknown Chip";
        memset(&gsChipCfgInfo, 0, sizeof(gsChipCfgInfo));
        memcpy(gsChipCfgInfo.szPartNumber, pName, sizeof(pName));
    }

    int i = 0;

    while (g_PartNumIDs[i].uID != 0xFFFFFFFF) {
        if (g_PartNumIDs[i].uID == uID) {
            gsChipCfgInfo.uID = uID;
            gsChipCfgInfo.uSeriesCode = g_PartNumIDs[i].uProjectCode;
            memcpy(gsChipCfgInfo.szPartNumber, g_PartNumIDs[i].szPartNumber, 32);
            break;
        }

        i++;
    }

    // skip time-consuming dummy search flow for 8051 1T series
    if ((gsChipCfgInfo.uID == uID) && (gsChipCfgInfo.uSeriesCode == NUC_CHIP_TYPE_GENERAL_1T)) {
        return true;
    }

    FLASH_PID_INFO_BASE_T flashInfo;

    if (GetInfo(uID, &flashInfo) != NULL) {
        gsChipCfgInfo.uID = uID;
        gsChipCfgInfo.uProgramMemorySize = flashInfo.uProgramMemorySize;
        gsChipCfgInfo.uDataFlashSize =	flashInfo.uDataFlashSize;
        gsChipCfgInfo.uFlashType = flashInfo.uFlashType;
        return true;
    }

    if (GetInfo_NuVoice(uID)) {
        gsChipCfgInfo.uID = uID;
        gsChipCfgInfo.uSeriesCode = gNuVoiceChip.dwSeriesEnum;
        memcpy(gsChipCfgInfo.szPartNumber, gNuVoiceChip.sChipName, 100);

        if (gsChipCfgInfo.uSeriesCode == ISD_9160_SERIES) {
            gsChipCfgInfo.uProgramMemorySize = gNuVoiceChip.dwAPROMSize + gNuVoiceChip.dwDataFlashSize + gNuVoiceChip.dwLDROMSize - 4096;
        } else if (gsChipCfgInfo.uSeriesCode == ISD_91300_SERIES) {
            gsChipCfgInfo.uProgramMemorySize = gNuVoiceChip.dwAPROMSize + gNuVoiceChip.dwDataFlashSize + gNuVoiceChip.dwLDROMSize - 4096;
        } else {
            gsChipCfgInfo.uProgramMemorySize = gNuVoiceChip.dwAPROMSize + gNuVoiceChip.dwDataFlashSize;
        }

        return true;
    }

    return false;
}

// call by CNuvoISPDlg::ShowChipInfo(): Show Chip Info. after connection
// call by CISPProc::Thread_ProgramFlash(): Update Size Info. if CONFIG is changed
bool UpdateSizeInfo(unsigned int uID, unsigned int uConfig0, unsigned int uConfig1,
                    unsigned int *puNVM_Addr,
                    unsigned int *puAPROM_Size, unsigned int *puNVM_Size)
{
    // just make sure GetChipConfigInfo is called, so gsChipCfgInfo is valid
    GetChipConfigInfo(uID);

    // skip time-consuming dummy search flow for 8051 1T series
    if ((gsChipCfgInfo.uID == uID) && (gsChipCfgInfo.uSeriesCode == NUC_CHIP_TYPE_GENERAL_1T)) {
        // internal ref. to Flash_N76E1T.h
        FLASH_INFO_BY_DID_T fInfo, *pInfo = &fInfo;

        if (GetInfo_8051_1T(uID & 0x0000FFFF, pInfo) == NULL) {
            return false;
        }

        unsigned int uLDROM_Addr;
        unsigned int uLDROM_Size;
        GetInfo_8051_1T(//uDID,
            uConfig0,
            pInfo->uProgramMemorySize,
            pInfo->uFlashType,
            &uLDROM_Addr,
            &uLDROM_Size,
            puAPROM_Size,
            puNVM_Size);
        *puNVM_Addr	= *puAPROM_Size;
        return true;
    }

    if (GetInfo(uID, uConfig0, uConfig1, puNVM_Addr, puAPROM_Size, puNVM_Size)) {
        return true;
    } else {
        // NuVoice Chip Series (ISDXXX, I9XXX, N57XXX ...)
        DWORD pConfig[4];
        pConfig[0] = uConfig0;
        pConfig[1] = uConfig1;

        if (GetInfo_NuVoice(uID, pConfig)) {
            *puNVM_Addr = gNuVoiceChip.dwDataFlashAddress;
            *puNVM_Size = gNuVoiceChip.dwDataFlashSize;
            *puAPROM_Size = gNuVoiceChip.dwAPROMSize;
            return true;
        } else {
            return false;
        }
    }
}

// call by CNuvoISPDlg::ShowChipInfo()
std::string GetPartNumber(unsigned int uID)
{
    GetChipConfigInfo(uID);
    return gsChipCfgInfo.szPartNumber;
}

struct CPartNumID g_AudioPartNumIDs[] = {
    /* Audio Part Number */
    {"I94133A", 0x1D010588, ISD_94000_SERIES},
    {"I91230G", 0x1D0A0463, ISD_91200_SERIES},
    {"ISD9130", 0x1D060163, ISD_9160_SERIES},
    {"I91361", 0x1D010284, ISD_91300_SERIES},
    {"I91032F", 0x1D010362, ISD_91000_SERIES},
    {"I94124A", 0x1D0105BA, NPCx_SERIES},
    {"I96100", 0x1D010800, ISD_96000_SERIES},
    {"N572U130", 0x0BB2FF0F, NUVOICE_N572F064_SERIES},
    {"N572S08B", 0x0BB10004, NUVOICE_N572F072_SERIES},
    {"N571P032", 0x0B320000, NUVOICE_N571_SERIES},
    {"N569S250", 0x0BA00301, NUVOICE_N569_SERIES},
    {"N570SCA2", 0x0BB0037F, NUVOICE_N570_SERIES},
    {"N570H064", 0x0B010762, NUVOICE_N570H_SERIES},
    {"N575S08A", 0x0BB00104, NUVOICE_N575_SERIES},
    {"N576F145", 0x0B600000, NUVOICE_N576_SERIES},
    {"JNK561F064", 0x0B800300, NUVOICE_JNK561_SERIES},
    {"---------", 0xFFFFFFFF, 0},
};

int LoadChipSeries(void)
{
    unsigned int i = 0, uProjectCode = 0;

    while (g_PartNumIDs[i].uID != 0xFFFFFFFF) {
        if (g_PartNumIDs[i].uProjectCode != uProjectCode) {
            uProjectCode = g_PartNumIDs[i].uProjectCode;
            g_NuMicroChipSeries.push_back(g_PartNumIDs[i]);
        }

        i++;
    }

    i = 0;

    while (g_AudioPartNumIDs[i].uID != 0xFFFFFFFF) {
        if (g_AudioPartNumIDs[i].uProjectCode != uProjectCode) {
            uProjectCode = g_AudioPartNumIDs[i].uProjectCode;
            g_AudioChipSeries.push_back(g_AudioPartNumIDs[i]);
        }

        i++;
    }

    return (g_NuMicroChipSeries.size() + g_AudioChipSeries.size());
}
