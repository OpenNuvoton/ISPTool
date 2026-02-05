#include "stdafx.h"
#include "NuDataBase.h"
#include "..\resource.h"
#include "PartNumID.h"
#include "FlashInfo.h"
extern struct CPartNumID g_PartNumIDs[];

CChipConfigInfo gsChipCfgInfo;
struct sChipInfo gNuVoiceChip;

/**
  * @brief Check if any given dwChipID is available in GetChipInformation.dll. (for Audio Series only)
  * @param[in] dwChipID The PDID read from the target device.
  * @param[in] pConfig The pointer to CONFIG read from the target device.
  * @retval false The is an unknown chip to Audio Series.
  * @retval true  The chip is found in the Audio Chip database.
  * @details Search the PDID through the part number list of NuVoice family (Audio Series).
  */
bool GetInfo_NuVoice(DWORD dwChipID, DWORD *pConfig)
{
    bool ret = false;
    memset(&gNuVoiceChip, 0, sizeof(sChipInfo));
    HMODULE hDll = ::LoadLibrary(_T("GetChipInformation.dll"));

    if (hDll != NULL)
    {
        CREATE_CHIPINFO_MANAGER pCreateChipInfoManager = (CREATE_CHIPINFO_MANAGER)::GetProcAddress(hDll, "CreateChipInfoManager");

        if (pCreateChipInfoManager)
        {
            I_ChipInfoManager *pChipInfoManager = NULL;

            if (pCreateChipInfoManager(&pChipInfoManager) == TRUE)
            {
                eChipInfoError err = pChipInfoManager->GetChipInfo(dwChipID, gNuVoiceChip, pConfig);

                if (err == ECE_NO_ERROR)
                {
                    ret = true;
                }

                pChipInfoManager->ReleaseDLL();
            }
        }

        FreeLibrary(hDll);
    }

    return ret;
}

/**
  * @brief Check if any given uID is available in g_PartNumIDs (ref: PartNumID.cpp).
  * @param[in] uID The PDID read from the target device.
  * @retval false The given uID can not be found in the database, it may belong to Audio series or not suport yet.
  * @retval true  The given uID is found.
  * @details Search the PDID through the part number list of NuMicro family (M0, M23, M4, 80511T and Motor series).
  */
bool GetChipStaticInfo(unsigned int uID)
{
    if (gsChipCfgInfo.uID == uID)
    {
        return true;
    }

    if (0x00550505 == uID)
    {
        char pName[] = "NUC505";
        memset(&gsChipCfgInfo, 0, sizeof(gsChipCfgInfo));
        memcpy(gsChipCfgInfo.szPartNumber, pName, sizeof(pName));
        gsChipCfgInfo.uID = uID;
        gsChipCfgInfo.uSeriesCode = PROJ_NUC505;
        gsChipCfgInfo.uProductLine = 2;
        return true;
    }
    else
    {
        char pName[] = "Unknown Chip";
        memset(&gsChipCfgInfo, 0, sizeof(gsChipCfgInfo));
        memcpy(gsChipCfgInfo.szPartNumber, pName, sizeof(pName));
        gsChipCfgInfo.uProductLine = 0; // Unknown
    }

    int i = 0;

    // Step1: get part no. from PartNumID
    while (g_PartNumIDs[i].uID != 0xFFFFFFFF)
    {
        if (g_PartNumIDs[i].uID == uID)
        {
            gsChipCfgInfo.uID = uID;
            gsChipCfgInfo.uSeriesCode = g_PartNumIDs[i].uProjectCode;
            memcpy(gsChipCfgInfo.szPartNumber, g_PartNumIDs[i].szPartNumber, 32);
            break;
        }

        i++;
    }

    // Step2: get flash info. from FlashInfo
    if (gsChipCfgInfo.uID == uID)
    {
        // internal ref. to Flash_N76E1T.h

        if ((gsChipCfgInfo.uSeriesCode >= PROJ_N76E885) && (gsChipCfgInfo.uSeriesCode <= PROJ_MG51D))
        {
            FLASH_INFO_BY_DID_T flashInfo;

            if (GetInfo_8051_1T(uID & 0x0000FFFF, &flashInfo) == NULL)
            {
                return false;
            }

            gsChipCfgInfo.uProductLine = 1; // 8051-1T
            gsChipCfgInfo.uProgramMemorySize = flashInfo.uProgramMemorySize;
            gsChipCfgInfo.uFlashType = flashInfo.uFlashType;
            return true;
        }
        else
        {
            FLASH_PID_INFO_BASE_T flashInfo;

            if (GetInfo(uID, &flashInfo) == NULL)
            {
                return false;
            }

            gsChipCfgInfo.uProductLine = 2;
            gsChipCfgInfo.uProgramMemorySize = flashInfo.uProgramMemorySize;
            gsChipCfgInfo.uFlashType = flashInfo.uFlashType;
            gsChipCfgInfo.uLDROM_Size = flashInfo.uISPFlashSize;
            unsigned int uSeriesCode = gsChipCfgInfo.uSeriesCode;
            unsigned int uFlashType = 0;

            // DataFlash Type 2
            if ((uSeriesCode == PROJ_NUC123AN)
            || (uSeriesCode == PROJ_NUC123AE)
            || (uSeriesCode == PROJ_NUC1311)
            || (uSeriesCode == PROJ_M0518))
            {
                uFlashType = 2;
            }
            else
            {
                uFlashType = (flashInfo.uDataFlashSize != 0) ? 1 : 0;
            }

            // Step3.2: Page Size Type: 0x000 (512 Bytes, default), 0x200 (2K), 0x300 (4K)
            if ((uSeriesCode == PROJ_M3331IG)
    || (uSeriesCode == PROJ_M3331G)
	|| (uSeriesCode == PROJ_M3351))
            {
                uFlashType |= 0x104;
            }
            
            else if ((uSeriesCode == PROJ_NUC400AE)
            || (uSeriesCode == PROJ_M451HD)
            || (uSeriesCode == PROJ_M451LD)
            || (uSeriesCode == PROJ_M4521)
            || (uSeriesCode == PROJ_M471)
            || (uSeriesCode == PROJ_M0564)
            || (uSeriesCode == PROJ_NUC1261)
            || (uSeriesCode == PROJ_NUC1262)
            || (uSeriesCode == PROJ_NUC1263)
            || (uSeriesCode == PROJ_M031_512K)
            || (uSeriesCode == PROJ_M031_256K))
            {
                uFlashType |= 0x200;
            }
            else if ((uSeriesCode == PROJ_M480) //M480
            || (uSeriesCode == PROJ_M480LD)
            || (uSeriesCode == PROJ_M460HD)
            || (uSeriesCode == PROJ_M460LD)
            || (uSeriesCode == PROJ_M2L31_512K)
            || (uSeriesCode == PROJ_M2L31_256K))
            {
                uFlashType |= 0x300;
            }
            else if (uSeriesCode == PROJ_M55M1)
            {
                uFlashType |= 0x400;
            }

            gsChipCfgInfo.uFlashType = uFlashType;
            return true;
        }
    }
    else
    {
        return false;
    }
}

static bool HasNoDynamicInfo()
{
    if (gsChipCfgInfo.uSeriesCode == PROJ_NUC505)
    {
        return true;
    }

    if ((gsChipCfgInfo.uSeriesCode == PROJ_M2351)
    || (gsChipCfgInfo.uSeriesCode == PROJ_M2354)
    || (gsChipCfgInfo.uSeriesCode == PROJ_M2354ES))
    {
        return true;
    }

    return false;
}

bool GetChipDynamicInfo(unsigned int uID, unsigned int uConfig0, unsigned int uConfig1)
{
    if (gsChipCfgInfo.uID == uID)
    {
        if ((gsChipCfgInfo.uConfig0 == uConfig0) && (gsChipCfgInfo.uConfig1 == uConfig1))
        {
            return true;
        }

        if (HasNoDynamicInfo())
        {
            return false;
        }
    }

    unsigned int uProductLine = 0;
    unsigned int uProgramMemorySize = 0, uFlashType = 0;
    unsigned int uAPROM_Addr = NUMICRO_FLASH_APROM_ADDR;

    if (GetChipStaticInfo(uID))
    {
        uProductLine = gsChipCfgInfo.uProductLine;
        uProgramMemorySize = gsChipCfgInfo.uProgramMemorySize;
        uFlashType = gsChipCfgInfo.uFlashType;

        if (gsChipCfgInfo.uSeriesCode == PROJ_M55M1)
        {
            gsChipCfgInfo.uAPROM_Addr = NUMICRO_M55_APROM_ADDR;
        }

        if (HasNoDynamicInfo())
        {
            return false;
        }
    }

    unsigned int uAPROM_Size;
    unsigned int uNVM_Addr, uNVM_Size;
    unsigned int uLDROM_Addr, uLDROM_Size;

    if (uProductLine == 1)
    {
        unsigned int uFlashMode;
        uFlashMode = uFlashType & 0x3;
        GetFlashSize_OT8051(
            uConfig0,
            uProgramMemorySize,
            uFlashMode,
            &uLDROM_Addr,
            &uLDROM_Size,
            &uAPROM_Size,
            &uNVM_Size);
        gsChipCfgInfo.uConfig0 = uConfig0;
        gsChipCfgInfo.uConfig1 = uConfig1;
        gsChipCfgInfo.uLDROM_Addr = uLDROM_Addr;
        gsChipCfgInfo.uLDROM_Size = uLDROM_Size;
        gsChipCfgInfo.uAPROM_Size = uAPROM_Size;
        gsChipCfgInfo.uNVM_Addr = uAPROM_Size;
        gsChipCfgInfo.uNVM_Size = uNVM_Size;
        return true;
    }
    else if (uProductLine == 2)
    {
        GetFlashSize_NuMicro(uConfig0, uConfig1,
                             uProgramMemorySize, uFlashType, uAPROM_Addr,
                             &uNVM_Addr, &uAPROM_Size, &uNVM_Size);
        gsChipCfgInfo.uConfig0 = uConfig0;
        gsChipCfgInfo.uConfig1 = uConfig1;
        gsChipCfgInfo.uAPROM_Size = uAPROM_Size;
        gsChipCfgInfo.uNVM_Addr = uNVM_Addr;
        gsChipCfgInfo.uNVM_Size = uNVM_Size;
        gsChipCfgInfo.uLDROM_Addr = NUMICRO_FLASH_LDROM_ADDR + NUMICRO_SPECIAL_FLASH_OFFSET;
        return true;
    }
    else
    {
        DWORD pConfig[4];
        pConfig[0] = uConfig0;
        pConfig[1] = uConfig1;

        if (GetInfo_NuVoice(uID, pConfig))
        {
            uProductLine = 3; // Audio
            gsChipCfgInfo.uID = uID;
            gsChipCfgInfo.uConfig0 = uConfig0;
            gsChipCfgInfo.uConfig1 = uConfig1;
            gsChipCfgInfo.uSeriesCode = gNuVoiceChip.dwSeriesEnum;
            memcpy(gsChipCfgInfo.szPartNumber, gNuVoiceChip.sChipName, 100);
            gsChipCfgInfo.uNVM_Addr = gNuVoiceChip.dwDataFlashAddress;
            gsChipCfgInfo.uNVM_Size = gNuVoiceChip.dwDataFlashSize;
            gsChipCfgInfo.uAPROM_Size = gNuVoiceChip.dwAPROMSize;

            if (gsChipCfgInfo.uSeriesCode == ISD_9160_SERIES)
            {
                gsChipCfgInfo.uProgramMemorySize = gNuVoiceChip.dwAPROMSize + gNuVoiceChip.dwDataFlashSize + gNuVoiceChip.dwLDROMSize - 4096;
            }
            else if (gsChipCfgInfo.uSeriesCode == ISD_91300_SERIES)
            {
                gsChipCfgInfo.uProgramMemorySize = gNuVoiceChip.dwAPROMSize + gNuVoiceChip.dwDataFlashSize + gNuVoiceChip.dwLDROMSize - 4096;
            }
            else
            {
                gsChipCfgInfo.uProgramMemorySize = gNuVoiceChip.dwAPROMSize + gNuVoiceChip.dwDataFlashSize;
            }

            return true;
        }

        return false;
    }
}

unsigned int GetFlashType(unsigned int uID, unsigned int uConfig0, unsigned int uConfig1)
{
    GetChipDynamicInfo(uID, uConfig0, uConfig1);
    return gsChipCfgInfo.uFlashType;
}

#ifdef _DEBUG

// auto selection from g_PartNumIDs + manual selection from g_80511TPartNumIDs
std::vector<CPartNumID> g_NuMicroChipSeries;
// manual selection from g_AudioChipSeries since all part no. are stored in .dll
std::vector<CPartNumID> g_AudioChipSeries;

// This is a subset of "GetChipInformation.dll". (Audio Chip DataBase)
struct CPartNumID g_AudioPartNumIDs[] =
{
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
    {"N574F128", 0x0B010D83, NUVOICE_N574F128_SERIES},
    {"N574F512", 0x0B010AB0, NUVOICE_N574F128_SERIES},
    {"N574F1K5", 0x0B010BD3, NUVOICE_N574F1K5_SERIES},
    {"N576F145", 0x0B600000, NUVOICE_N576_SERIES},
    {"JNK561F064", 0x0B800300, NUVOICE_JNK561_SERIES},
    {"---------", 0xFFFFFFFF, 0},
};

// This is a subset of "g_PartNumIDs" for NUC_CHIP_TYPE_GENERAL_1T. (NuMicro Chip DataBase)
// All NUC_CHIP_TYPE_GENERAL_1T used CDialogConfiguration_OT8051.
// Need to specify part no. for difference cases in CDialogConfiguration_OT8051.
struct CPartNumID g_80511TPartNumIDs[] =
{
    /* 8051 1T N76 & ML51 & MS51 series */
    {"N76E885", 0x00002150, PROJ_N76E885},
    {"N76E616", 0x00002F50, PROJ_N76E616},
    {"N76E003", 0x00003650, PROJ_N76E003},

    // For ML51, Version A (PID0 = 0x00), Version B (PID0 = 0x10)
    {"ML51TC0AE", 0x0C104832, PROJ_ML51_32K},
    {"ML51UB9AE", 0x03104721, PROJ_ML51_16K},
    {"MS51FB9AE", 0x0B004B21, PROJ_MS51_16K},
    {"MS51DA9AE", 0x0A005211, PROJ_MS51_8K},
    {"MS51FC0AE", 0x0B005332, PROJ_MS51_32K},
    {"ML56SD1AE", 0x08125744, PROJ_ML56},

    // MUG51
    {"MUG51TB9AE", 0x0C106321, PROJ_MUG51},

    // MG51
    {"MG51XB9AE", 0x05006721, PROJ_MG51},
    {"MG51LD1AE", 0x00007244, PROJ_MG51D},

    {"---------", 0xFFFFFFFF, 0},
};

// call by CDialogMain::CDialogMain(...) in Debug Mode only (for test purpose)
// Used to generate a dynamic Menu to test Configuration Dialog
int LoadChipSeries(void)
{
    unsigned int i = 0, uProjectCode = 0;

    while (g_PartNumIDs[i].uID != 0xFFFFFFFF)
    {
        if ((g_PartNumIDs[i].uProjectCode >= PROJ_N76E885)
                && (g_PartNumIDs[i].uProjectCode <= PROJ_MG51D))
        {
            break;
        }

        if (g_PartNumIDs[i].uProjectCode != uProjectCode)
        {
            uProjectCode = g_PartNumIDs[i].uProjectCode;
            g_NuMicroChipSeries.push_back(g_PartNumIDs[i]);
        }

        i++;
    }

    i = 0;

    while (g_80511TPartNumIDs[i].uID != 0xFFFFFFFF)
    {
        struct CPartNumID tmp = g_80511TPartNumIDs[i];
        tmp.uProjectCode = g_80511TPartNumIDs[i].uProjectCode;
        tmp.uID = g_80511TPartNumIDs[i].uID;
        g_NuMicroChipSeries.push_back(tmp);
        i++;
    }

    i = 0;

    while (g_AudioPartNumIDs[i].uID != 0xFFFFFFFF)
    {
        if (g_AudioPartNumIDs[i].uProjectCode != uProjectCode)
        {
            uProjectCode = g_AudioPartNumIDs[i].uProjectCode;
            g_AudioChipSeries.push_back(g_AudioPartNumIDs[i]);
        }

        i++;
    }

    return (g_NuMicroChipSeries.size() + g_AudioChipSeries.size());
}

#endif // #ifdef _DEBUG
