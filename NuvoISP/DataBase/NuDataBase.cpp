#include "stdafx.h"
#include "NuDataBase.h"

#include "PartNumID.h"
#include "FlashInfo.h"

CChipConfigInfo gsChipCfgInfo;
struct sChipInfo gNuVoiceChip;

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

