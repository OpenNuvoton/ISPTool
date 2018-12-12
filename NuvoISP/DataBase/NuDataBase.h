#ifndef INC__NUVOICE_INFO_H__
#define INC__NUVOICE_INFO_H__
#pragma once

#include <stdio.h>
#include <string> // std::string

/* NuVoice Series */
#include "IGetChipInformation.h"

struct CChipConfigInfo {
    unsigned int uID;
    unsigned int uSeriesCode;
    unsigned int uProgramMemorySize;
    unsigned int uDataFlashSize;
    char szPartNumber[100];
};

extern CChipConfigInfo gsChipCfgInfo;
bool GetChipConfigInfo(unsigned int uID);
std::string GetPartNumber(unsigned int uID);

bool UpdateSizeInfo(unsigned int uID, unsigned int uConfig0, unsigned int uConfig1,
                    unsigned int *puAPROM_Size,
                    unsigned int *puNVM_Addr, unsigned int *puNVM_Size);

bool GetInfo_NuVoice(DWORD dwChipID, DWORD *pConfig = NULL);
#endif
