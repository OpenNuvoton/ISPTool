#ifndef INC__NUVOICE_INFO_H__
#define INC__NUVOICE_INFO_H__
#pragma once

#include <stdio.h>
#include <string> // std::string
#include <vector> // std::vector

/* NuMicro Series */
#include "ChipDefs.h" // for enum NUC_CHIP_TYPE_E
#include "PartNumID.h"

/* NuVoice Series */
#include "IGetChipInformation.h" // for enum eAllChipSeries

// Note: There is no dynamic info. for NUC505 & M2351.
struct CChipConfigInfo {
    // Static Info.
    unsigned int uID;
    unsigned int uSeriesCode;
    unsigned int uProgramMemorySize;
    unsigned int uDataFlashSize;
    char szPartNumber[100];
    unsigned int uFlashType; // used by GetFlashSize_NuMicro(...)
    unsigned int uProductLine; // 0: Unknown, 1: 8051-1T, 2: NuMicro, 3: Audio
    // Dynamic info. according to user configuration (CONFIG0, CONFIG1)
    unsigned int uConfig0;
    unsigned int uConfig1;
    unsigned int uAPROM_Addr;
    unsigned int uAPROM_Size;
    unsigned int uNVM_Addr;
    unsigned int uNVM_Size;
    unsigned int uLDROM_Addr;
    unsigned int uLDROM_Size;
};

extern CChipConfigInfo gsChipCfgInfo;
bool GetChipDynamicInfo(unsigned int uID, unsigned int uConfig0, unsigned int uConfig1);

extern std::vector<CPartNumID> g_NuMicroChipSeries;
extern std::vector<CPartNumID> g_AudioChipSeries;
int LoadChipSeries(void);

#endif
