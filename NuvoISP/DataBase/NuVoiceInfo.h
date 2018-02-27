#ifndef INC__NUVOICE_INFO_H__
#define INC__NUVOICE_INFO_H__
#pragma once

#include <stdio.h>
#include "ChipDefs.h"

/* NuVoice Series */
#include "IGetChipInformation.h"
extern struct sChipInfo gNuVoiceChip;
bool GetInfo_NuVoice(DWORD dwChipID, DWORD *pConfig = NULL, DWORD dwConfigNum = 2);

#endif
