#ifndef INC__FLASH_INFO_H__
#define INC__FLASH_INFO_H__
#pragma once

#include <stdio.h>
#include "ChipDefs.h"

/* Get Flash Info by PID */
typedef struct {
    unsigned int uProgramMemorySize;
    unsigned int uDataFlashSize;
    unsigned int uRAMSize;
    unsigned int uDataFlashStartAddress;
    unsigned int uISPFlashSize;
    unsigned int uPID;
} FLASH_PID_INFO_BASE_T;

void *GetInfo(unsigned int uPID,
              FLASH_PID_INFO_BASE_T *pInfo);

bool GetInfo2(unsigned int uPID,
              unsigned int uConfig0,
              unsigned int uConfig1,
              unsigned int *puNVM_Addr,
              unsigned int *puAPROM_Size,
              unsigned int *puNVM_Size);

/* Get Flash Info by DID */
typedef struct {
    unsigned int uProgramMemorySize;
    unsigned int uLDSize;
    unsigned int uRAMSize;
    unsigned int uDID;
    unsigned int uFlashType;
} FLASH_INFO_BY_DID_T;

/* 8051 1T Series */
void *GetInfo_N76E1T(unsigned int uDID,
                     FLASH_INFO_BY_DID_T *pInfo);

bool GetInfo_N76E1T(//unsigned int uDID,
    unsigned int uConfig0,
    unsigned int uProgramMemorySize,
    unsigned int uFlashType,
    unsigned int *puLDROM_Addr,
    unsigned int *puLDROM_Size,
    unsigned int *puAPROM_Size,
    unsigned int *puNVM_Size);

#endif

