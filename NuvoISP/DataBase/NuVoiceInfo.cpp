#include "stdafx.h"
#include "NuVoiceInfo.h"

struct sChipInfo gNuVoiceChip;


bool GetInfo_NuVoice(DWORD dwChipID, DWORD *pConfig, DWORD dwConfigNum)
{
    memset(&gNuVoiceChip, 0, sizeof(sChipInfo));
    HMODULE hDll = ::LoadLibrary(_T("GetChipInformation.dll"));

    if (hDll != NULL) {
        CREATE_CHIPINFO_MANAGER pCreateChipInfoManager = (CREATE_CHIPINFO_MANAGER)::GetProcAddress(hDll, "CreateChipInfoManager");

        if (pCreateChipInfoManager) {
            I_ChipInfoManager *pChipInfoManager = NULL;

            if (pCreateChipInfoManager(&pChipInfoManager) == TRUE) {
                if (pChipInfoManager->GetChipInfoByID(dwChipID, gNuVoiceChip, pConfig, dwConfigNum) == TRUE) {
                    return true;
                }

                pChipInfoManager->Release();
            }
        }

        FreeLibrary(hDll);
    }

    return false;
}
