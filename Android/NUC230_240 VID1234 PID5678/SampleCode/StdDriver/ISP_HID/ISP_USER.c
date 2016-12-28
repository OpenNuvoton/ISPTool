/******************************************************************************
 * @file     ISP_USER.c
 * @brief    ISP sample source file
 * @version  0x27
 * @date     31, December, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "string.h"
#include "ISP_USER.h"

__align(4) uint8_t response_buff[64];
__align(4) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];
uint32_t bUpdateApromCmd;
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;

#ifdef SUPPORT_WRITECKSUM
static uint32_t g_ckbase = (0x20000 - 8);

static void CheckCksumBase()
{
    //skip data flash
    unsigned int aprom_end = g_apromSize, data;
    int result;

    result = FMC_Read_User(0x1F000 - 8, &data);
    if(result == 0) { //128K flash
        FMC_Read_User(Config0, &data);
        if((data&0x01)==0) { //DFEN enable
            FMC_Read_User(Config1, &aprom_end);
        }
        g_ckbase = aprom_end - 8;
        return;
    } else { // less than 128K
        aprom_end = 0x10000;//64K
        do {
            result = FMC_Read_User(aprom_end - 8, &data);
            if(result == 0)
                break;
            aprom_end = aprom_end/2;
        } while(aprom_end > 4096);

        g_ckbase = aprom_end - 8;
    }
}
#endif

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for(c = 0 , i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
    int i;
    register uint16_t lcksum = 0;

    for(i = 0; i < len; i+=FMC_FLASH_PAGE_SIZE) {
        ReadData(start + i, start + i + FMC_FLASH_PAGE_SIZE, (uint32_t*)aprom_buf);
        if(len - i >= FMC_FLASH_PAGE_SIZE)
            lcksum += Checksum(aprom_buf, FMC_FLASH_PAGE_SIZE);
        else
            lcksum += Checksum(aprom_buf, len - i);
    }

    return lcksum;

}

//bAprom == TRUE erase all aprom besides data flash
void EraseAP(uint32_t bAprom, unsigned int addr_start, unsigned int addr_end)
{
    unsigned int eraseLoop;
    unsigned int erase_end;

    if(bAprom == TRUE) {
        //GetDataFlashInfo(&erase_end, &eraseLoop);
        erase_end = g_dataFlashAddr;
        eraseLoop = FMC_APROM_BASE;
    } else {
        eraseLoop = addr_start;
        erase_end = addr_end;
    }

    for(; eraseLoop < erase_end; eraseLoop += FMC_FLASH_PAGE_SIZE ) {
        FMC_Erase_User(eraseLoop);
    }
    return;
}

void UpdateConfig(unsigned int *data, unsigned int *res)
{

    FMC_ENABLE_CFG_UPDATE();

    FMC_Erase_User(Config0);

    FMC_Write_User(Config0, *data);
    FMC_Write_User(Config1, *(data+1));

    if(res) {
        FMC_Read_User(Config0, res);
        FMC_Read_User(Config1, res+1);
    }

    FMC_DISABLE_CFG_UPDATE();
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;

    *size = 0;

    if(g_apromSize >= 0x20000) {
        FMC_Read_User(Config0, &uData);
        if((uData&0x01) == 0) { //DFEN enable
            FMC_Read_User(Config1, &uData);
            if(uData > g_apromSize || uData < 0x200)//avoid config1 value from error
                uData = g_apromSize;

            *addr = uData;
            *size = g_apromSize - uData;
        } else {
            *addr = g_apromSize;
            *size = 0;
        }
    } else {
        *addr = 0x1F000;
        *size = 4096;//4K
    }
}

//the smallest of APROM size is 2K
uint32_t GetApromSize()
{
    uint32_t size = 0x800, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);
        if(result < 0) {
            return size;
        } else
            size *= 2;
    } while(1);
}

int ParseCmd(unsigned char *buffer, uint8_t len)
{
    static uint32_t StartAddress, StartAddress_bak, TotalLen, TotalLen_bak, LastDataLen, g_packno = 1;
    uint8_t *response;
    uint16_t lcksum;
    uint32_t lcmd, srclen, i, regcnf0, security;
    unsigned char *pSrc;
    static uint32_t	gcmd;

    response = response_buff;

    pSrc = buffer;
    srclen = len;
    lcmd = inpw(pSrc);

    outpw(response + 4, 0);

    pSrc += 8;
    srclen -= 8;

    ReadData(Config0, Config0 + 8, (uint32_t*)(response + 8));//read config
    regcnf0 = *(uint32_t*)(response + 8);
    security = regcnf0 & 0x2;


    if(lcmd == CMD_SYNC_PACKNO) {
        g_packno = inpw(pSrc);
    }

    if((lcmd) && (lcmd!=CMD_RESEND_PACKET))
        gcmd = lcmd;

    if(lcmd == CMD_GET_FWVER) {
        response[8] = FW_VERSION;//version 2.3
    } else if(lcmd == CMD_GET_DEVICEID) {
        outpw(response+8, SYS->PDID);
        goto out;
    } else if(lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET) {
        outpw(&SYS->RSTSRC, 3);//clear bit
        /* Set BS */
        if(lcmd == CMD_RUN_APROM) {
            i = (FMC->ISPCON & 0xFFFFFFFC);
        } else if(lcmd == CMD_RUN_LDROM) {
            i = (FMC->ISPCON & 0xFFFFFFFC);
            i |= 0x00000002;
        } else {
            i = (FMC->ISPCON & 0xFFFFFFFE);//ISP disable
        }

        outpw(&FMC->ISPCON, i);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

        /* Trap the CPU */
        while(1);
    } else if(lcmd == CMD_CONNECT) {
        g_packno = 1;
        goto out;
    } else if(lcmd == CMD_DISCONNECT) {
        return 0;
    }

    else if((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL)) {
        if(lcmd == CMD_ERASE_ALL) { //erase APROM + data flash
            EraseAP(FALSE, 0, (g_apromSize < 0x20000)?0x20000:g_apromSize);//erase all aprom including data flash
            *(uint32_t*)(response + 8) = regcnf0|0x02;
            UpdateConfig((uint32_t*)(response + 8), NULL);
        } else
            EraseAP(TRUE, 0, 0);//don't erase data flash

        bUpdateApromCmd = TRUE;
    }
#ifdef SUPPORT_WRITECKSUM
    else if(lcmd == CMD_WRITE_CHECKSUM) { //write cksum to aprom last
        uint32_t cktotallen = inpw(pSrc);
        lcksum = inpw(pSrc+4);
        CheckCksumBase();
        ReadData(g_ckbase & 0xFFE00, g_ckbase, (uint32_t*)aprom_buf);
        outpw(aprom_buf+FMC_FLASH_PAGE_SIZE - 8, cktotallen);
        outpw(aprom_buf+FMC_FLASH_PAGE_SIZE - 4, lcksum);
        FMC_Erase(g_ckbase & 0xFFE00);
        WriteData(g_ckbase & 0xFFE00, g_ckbase + 8, (uint32_t*)aprom_buf);
    }
#endif
    else if(lcmd == CMD_GET_FLASHMODE) {
        //return 1: APROM, 2: LDROM
        outpw(response+8, (FMC->ISPCON&0x2)? 2 : 1);
    }


    if((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH)) {
        if(lcmd == CMD_UPDATE_DATAFLASH) {
            StartAddress = g_dataFlashAddr;

            if(g_dataFlashSize) { //g_dataFlashAddr
                EraseAP(FALSE, g_dataFlashAddr, g_dataFlashAddr + g_dataFlashSize);
            } else
                goto out;
        } else {
            StartAddress = 0;
        }

        //StartAddress = inpw(pSrc);
        TotalLen = inpw(pSrc+4);
        pSrc += 8;
        srclen -= 8;
        StartAddress_bak = StartAddress;
        TotalLen_bak = TotalLen;
    } else if(lcmd == CMD_UPDATE_CONFIG) {
        if((security == 0) && (!bUpdateApromCmd))//security lock
            goto out;

        UpdateConfig((uint32_t*)(pSrc), (uint32_t*)(response+8));
        GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

        goto out;
    } else if(lcmd == CMD_RESEND_PACKET) { //for APROM&Data flash only
        StartAddress -= LastDataLen;
        TotalLen += LastDataLen;
        if((StartAddress & 0xFFE00) >= Config0)
            goto out;
        ReadData(StartAddress & 0xFFE00, StartAddress, (uint32_t*)aprom_buf);
        FMC_Erase_User(StartAddress & 0xFFE00);
        WriteData(StartAddress & 0xFFE00, StartAddress, (uint32_t*)aprom_buf);
        if((StartAddress%FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE-LastDataLen))
            FMC_Erase_User((StartAddress & 0xFFE00)+FMC_FLASH_PAGE_SIZE);
        goto out;

    }

    if((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH))
    {
        if(TotalLen < srclen) {
            srclen = TotalLen;//prevent last package from over writing
        }

        TotalLen -= srclen;

        WriteData(StartAddress, StartAddress + srclen, (uint32_t*)pSrc); //WriteData(StartAddress, StartAddress + srclen, (uint32_t*)pSrc);
        memset(pSrc, 0, srclen);

        ReadData(StartAddress, StartAddress + srclen, (uint32_t*)pSrc);
        StartAddress += srclen;
        LastDataLen =  srclen;

        if(TotalLen == 0) {
            lcksum = CalCheckSum(StartAddress_bak, TotalLen_bak);
            outps(response + 8, lcksum);
        }
    }
out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response+4, g_packno);
    g_packno++;


    return 0;
}

