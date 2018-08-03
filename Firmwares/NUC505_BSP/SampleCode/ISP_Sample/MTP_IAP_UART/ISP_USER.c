/******************************************************************************
 * @file     ISP_USER.c
 * @brief    NUC505 series ISP sample source file
 * @version  0x32
 * @date     01, March, 2017
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "string.h"
#include "ISP_USER.h"
#include "spiflash_drv.h"

__align(4) uint8_t g_u8RcvBuf[CMDBUFSIZE];
__align(4) uint8_t g_u8SendBuf[CMDBUFSIZE];

__align(4) static uint8_t aprom_buf[PAGE_SIZE];
BOOL bUpdateApromCmd;
uint32_t g_apromAddr, g_apromSize, g_dataFlashAddr, g_dataFlashSize;

#if(SUPPORT_WRITECKSUM)
static uint32_t g_ckbase = (0x20000 - 8);
static void CheckCksumBase(void);
#endif

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
    int i;
    register uint16_t lcksum = 0;

    for (i = 0; i < len; i += PAGE_SIZE) {
        SPIFlash_ReadData(start + i, PAGE_SIZE, aprom_buf);

        if (len - i >= PAGE_SIZE) {
            lcksum += Checksum(aprom_buf, PAGE_SIZE);
        } else {
            lcksum += Checksum(aprom_buf, len - i);
        }
    }

    return lcksum;
}

int ParseCmd(unsigned char *buffer, uint8_t len, uint8_t bUSB)
{
    static uint32_t StartAddress, StartAddress_bak, TotalLen, TotalLen_bak, LastDataLen, g_packno = 1;
    uint8_t *response;
    uint16_t lcksum;
    uint32_t lcmd, srclen;
    unsigned char *pSrc;
    static uint32_t	gcmd;
    response = g_u8SendBuf;
    pSrc = buffer;
    srclen = len;
    // Input Word0: Command
    lcmd = inpw(pSrc);
    outpw(response + 4, 0);
    pSrc += 8;
    srclen -= 8;

    if (lcmd == CMD_SYNC_PACKNO) {
        g_packno = inpw(pSrc);
    }

    if ((lcmd) && (lcmd != CMD_RESEND_PACKET)) {
        gcmd = lcmd;
    }

    switch (lcmd) {
        case CMD_GET_FWVER: {
            response[8] = FW_VERSION;
            break;
        }

        case CMD_GET_DEVICEID: {
            outpw(response + 8, SYS->PDID & SYS_PDID_PDID_Msk);
            goto out;
        }

        case CMD_RUN_APROM:
            //Reset_AfterREVMP();//YT modify for MTP
            SYS->LVMPADDR = 0x00004000;
            SYS->LVMPLEN = 0x10;
            SYS->RVMPLEN = 0x01;
            SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);
            __NOP();
            __NOP();

            while (1);

        case CMD_RUN_LDROM:
        case CMD_RESET: {
            outpw(0x40000050, 0x00000000);    /* Specify the load VECMAP address   (reg : SYS_LVMPADDR) */
            outpw(0x40000054, 0x00000000);    /* Any value allowed to map whole SPI Flash (reg : SYS_LVMPLEN)  */
            outpw(0x4000005C, 0x00000001);    /* Load VECMAP address and length    (reg : SYS_RVMPLEN)  */
            outpw(0x40007000, 0x03C00003);
            /* Specify divider for SPI bus clock to 2 (DIVIDER=1), and
               continue using IFSEL, which is assumed to match real
               hardware connection (reg : SPIM_CTL1) */
            outpw(0x40007004, 0x00010010 | (inpw(0x40007004) & 0x000000C0));
            outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

            /* Trap the CPU */
            while (1);
        }

        case CMD_CONNECT: {
            g_packno = 1;
            goto out;
        }

        case CMD_DISCONNECT: {
            return 0;
        }

        case CMD_UPDATE_APROM:
            bUpdateApromCmd = TRUE;
            break;

        case  CMD_GET_FLASHMODE:
            //return 1: APROM, 2: LDROM
            outpw(response + 8, 2);
            break;

        default:
            break;
            // Do Nothing
    }

// Erase APROM and DATAFLASH
    if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH)) {
        if (lcmd == CMD_UPDATE_APROM) {
            StartAddress = APROM_BASE;
        } else {
            StartAddress = inpw(pSrc);
        }

        TotalLen = inpw(pSrc + 4);
        pSrc += 8;
        srclen -= 8;
        StartAddress_bak = StartAddress;
        TotalLen_bak = TotalLen;
        SPIFlash_EraseAddrRange(StartAddress, TotalLen);
        //printf("StartAddress=%x,TotalPadLen=%d\n",StartAddress, TotalPadLen);
        //return 0;
    } else if (lcmd == CMD_RESEND_PACKET) { //for APROM&Data flash only
        StartAddress -= LastDataLen;
        TotalLen += LastDataLen;
        SPIFlash_ReadData(StartAddress & 0xFFFFF000, StartAddress & 0x0FFF, aprom_buf);
        SPIFlash_EraseAddrRange(StartAddress, LastDataLen);
        SPIFlash_WriteData(StartAddress & 0xFFFFF000, StartAddress & 0x0FFF, aprom_buf);
        goto out;
    }

// If data can¡¦t put into one packet, and then packet format is as following (named Format2)
// 0x00000000  Packet Number  Data¡K¡K
    if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH)) {
        if (TotalLen < srclen) {
            srclen = TotalLen;//prevent last package from over writing
        }

        if (0 == lcmd) {
            lcmd = CMD_UPDATE_APROM;
        }

        TotalLen -= srclen;
        SPIFlash_WriteData(StartAddress, srclen, pSrc);
        memset(pSrc, 0, srclen);
        SPIFlash_ReadData(StartAddress, srclen, pSrc);
        StartAddress += srclen;
        LastDataLen =  srclen;

        if (TotalLen == 0) {
            lcksum = CalCheckSum(StartAddress_bak, TotalLen_bak);
            outps(response + 8, lcksum);
        }
    }

out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response + 4, g_packno);
    g_packno++;
    return 0;
}
