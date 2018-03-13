#include "stdafx.h"
#include "NuVoiceInfo.h"

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
