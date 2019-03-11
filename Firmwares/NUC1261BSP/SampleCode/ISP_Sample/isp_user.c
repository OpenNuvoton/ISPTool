#include <stdio.h>
#include "isp_user.h"
#include "fmc_user.h"

#if 0
#define RSTSTS		RSTSRC
#define ISPCTL		ISPCON
#endif


volatile uint8_t bISPDataReady;

__align(4) uint8_t response_buff[64];
__align(4) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];
uint32_t bUpdateApromCmd;
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;

__STATIC_INLINE uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

int ParseCmd(unsigned char *buffer, uint8_t len)
{
    static uint32_t StartAddress, TotalLen, LastDataLen, g_packno = 1;
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
    ReadData(Config0, Config0 + 8, (uint32_t *)(response + 8)); //read config
    regcnf0 = *(uint32_t *)(response + 8);
    security = regcnf0 & 0x2;

    if (lcmd == CMD_SYNC_PACKNO) {
        g_packno = inpw(pSrc);
    }

    if ((lcmd) && (lcmd != CMD_RESEND_PACKET)) {
        gcmd = lcmd;
    }

    if (lcmd == CMD_GET_FWVER) {
        response[8] = FW_VERSION;
    } else if (lcmd == CMD_GET_DEVICEID) {
        outpw(response + 8, SYS->PDID);
        goto out;
    } else if (lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET) {
        SYS->RSTSTS = 3; //clear bit

        /* Set BS */
        if (lcmd == CMD_RUN_APROM) {
            i = (FMC->ISPCTL & 0xFFFFFFFC);
        } else if (lcmd == CMD_RUN_LDROM) {
            i = (FMC->ISPCTL & 0xFFFFFFFC);
            i |= 0x00000002;
        } else {
            i = (FMC->ISPCTL & 0xFFFFFFFE);//ISP disable
        }
        
        FMC->ISPCTL = i;
        SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

        /* Trap the CPU */
        while (1);
    } else if (lcmd == CMD_CONNECT) {
        g_packno = 1;
        goto out;
    } else if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL)) {
        EraseAP(FMC_APROM_BASE, (g_apromSize < g_dataFlashAddr) ? g_apromSize : g_dataFlashAddr); // erase APROM // g_dataFlashAddr, g_apromSize

        if (lcmd == CMD_ERASE_ALL) {
            EraseAP(g_dataFlashAddr, g_dataFlashSize);
            *(uint32_t *)(response + 8) = regcnf0 | 0x02;
            UpdateConfig((uint32_t *)(response + 8), NULL);
        }

        bUpdateApromCmd = TRUE;
    }

    if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH)) {
        if (lcmd == CMD_UPDATE_DATAFLASH) {
            StartAddress = g_dataFlashAddr;

            if (g_dataFlashSize) { //g_dataFlashAddr
                EraseAP(g_dataFlashAddr, g_dataFlashSize);
            } else {
                goto out;
            }
        } else {
            StartAddress = 0;
        }

        //StartAddress = inpw(pSrc);
        TotalLen = inpw(pSrc + 4);
        pSrc += 8;
        srclen -= 8;
    } else if (lcmd == CMD_UPDATE_CONFIG) {
        if ((security == 0) && (!bUpdateApromCmd)) { //security lock
            goto out;
        }

        UpdateConfig((uint32_t *)(pSrc), (uint32_t *)(response + 8));
        GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
        goto out;
    } else if (lcmd == CMD_RESEND_PACKET) { //for APROM&Data flash only
        uint32_t PageAddress;
        StartAddress -= LastDataLen;
        TotalLen += LastDataLen;
        PageAddress = StartAddress & (0x100000 - FMC_FLASH_PAGE_SIZE);

        if (PageAddress >= Config0) {
            goto out;
        }

        ReadData(PageAddress, StartAddress, (uint32_t *)aprom_buf);
        FMC_Erase_User(PageAddress);
        WriteData(PageAddress, StartAddress, (uint32_t *)aprom_buf);

        if ((StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - LastDataLen)) {
            FMC_Erase_User(PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH)) {
        if (TotalLen < srclen) {
            srclen = TotalLen;//prevent last package from over writing
        }

        TotalLen -= srclen;
        WriteData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        memset(pSrc, 0, srclen);
        ReadData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        StartAddress += srclen;
        LastDataLen =  srclen;
    }

out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response + 4, g_packno);
    g_packno++;
    return 0;
}

