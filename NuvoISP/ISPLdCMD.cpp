#include "stdafx.h"
#include "ISPLdCMD.h"
#include <stdio.h>

#define USBCMD_TIMEOUT		5000
#define USBCMD_TIMEOUT_LONG	25000

#define printf(...)

ISPLdCMD::ISPLdCMD()
    : m_bOpenPort(FALSE)
    , m_uCmdIndex(18)	//Do not use 0 to avoid firmware already has index 0 occasionally.
{
    //Test();
}

ISPLdCMD::~ISPLdCMD()
{
}

bool ISPLdCMD::Open_Port(BOOL bErrorMsg)
{
    if (m_bOpenPort) {
        return true;
    }

    m_uUSB_PID = 0;
    m_strDevPathName = _T("");
    ScopedMutex scopedLock(m_Mutex);

    switch (m_uInterface) {
        case 1:
            if (m_hidIO.OpenDevice(0x0416, 0x3F00, -1)) {	// ISP FW >= 0x30
                m_uUSB_PID = 0x3F00;
            } else if (m_hidIO.OpenDevice(0x0416, 0xA316, -1)) {	// ISP FW < 0x30
                m_uUSB_PID = 0xA316;
            } else {
                return false;
            }

            m_strDevPathName = m_hidIO.GetDevicePath();
            break;

        case 2:
            if (!m_comIO.OpenDevice(m_strComNum)) {
                //        if(bErrorMsg)
                //printf("No upload Device Found, Please Check the Link!\n");
                return false;
            }

            break;

        case 3:
        case 4:
        case 5:
        case 6:
            if (m_hidIO2.OpenDevice(0x0416, 0x5201, 5)) {	// Nu-Link2 with ISP-Bridge
                m_uUSB_PID = 0x5201;
            } else if (m_hidIO2.OpenDevice(0x0416, 0x3F10, -1)) {	// ISP-Bridge
                m_uUSB_PID = 0x3F10;
            } else {
                return false;
            }

            m_strDevPathName = m_hidIO2.GetDevicePath();
            break;

        default:
            return false;
    }

    m_bOpenPort = TRUE;
    return true;
}

void ISPLdCMD::Close_Port()
{
    m_strDevPathName = _T("");
    ScopedMutex scopedLock(m_Mutex);

    if (!m_bOpenPort) {
        return;
    }

    m_bOpenPort = FALSE;

    switch (m_uInterface) {
        case 1:
            m_hidIO.CloseDevice();
            break;

        case 2:
            m_comIO.CloseDevice();
            break;

        case 3:
        case 4:
        case 5:
        case 6:
            m_hidIO2.CloseDevice();
            break;

        default:
            break;
    }
}

void ISPLdCMD::ReOpen_Port(BOOL bForce)
{
    // Re-Open COM Port to clear previous status
    if (bForce || (m_uInterface == 2)) {
        Close_Port();
        Open_Port();
    }
}

bool ISPLdCMD::Check_USB_Link()
{
    ScopedMutex scopedLock(m_Mutex);
    return m_bOpenPort ? true : false;
}

BOOL ISPLdCMD::ReadFile(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex)
{
    bResendFlag = FALSE;

    while (1) {
        if (!m_bOpenPort) {
            throw _T("There is no Nu-Link connected to a USB port.");
        }

        DWORD dwLength;

        switch (m_uInterface) {
            case 1:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
                    return FALSE;
                }

                break;

            case 2:
                if (!m_comIO.ReadFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds)) {
                    printf("NG in m_comIO.ReadFile\n");
                    return FALSE;
                }

                break;

            case 3:
            case 4:
            case 5:
                if (!m_hidIO2.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
                    return FALSE;
                }

                break;

            default:
                return FALSE;
                break;
        }

        /* Check if correct package index was read */
        //m_acBuffer[0];	//For HID internal usage, ignored.
        USHORT usCheckSum = *((USHORT *)&m_acBuffer[1]);
        ULONG uCmdIndex = *((ULONG *)&m_acBuffer[5]);

        if (dwLength >= 8
                && (!bCheckIndex || uCmdIndex == m_uCmdIndex - 1)
                && usCheckSum == m_usCheckSum) {
            if (szMaxLen > dwLength - 8) {
                szMaxLen = dwLength - 8;
            }

            if (pcBuffer != NULL && szMaxLen > 0) {
                memcpy(pcBuffer, m_acBuffer + 9, szMaxLen);
            }

            return TRUE;
        } else if (m_uUSB_PID == 0xA316) {
            SleepEx(10, TRUE);
        } else {
            printf("dwLength = %d, uCmdIndex = %d, %d, usCheckSum = %d, %d\n", dwLength, uCmdIndex, m_uCmdIndex, usCheckSum, m_usCheckSum);
            bResendFlag = TRUE;
            break;
        }
    }

    return FALSE;
}

BOOL ISPLdCMD::WriteFile(unsigned long uCmd, const char *pcBuffer, DWORD dwLen, DWORD dwMilliseconds)
{
    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    }

    /* Set new package index value */
    DWORD dwCmdLength = dwLen;

    if (dwCmdLength > sizeof(m_acBuffer) - 9) {
        dwCmdLength = sizeof(m_acBuffer) - 9;
    }

    memset(m_acBuffer, 0, sizeof(m_acBuffer));
    //m_acBuffer[0] = 0x00;	//Always 0x00
    *((ULONG *)&m_acBuffer[1]) = uCmd;
    *((ULONG *)&m_acBuffer[5]) = m_uCmdIndex;

    if (pcBuffer != NULL && dwCmdLength > 0) {
        memcpy(m_acBuffer + 9, pcBuffer, dwCmdLength);
    }

    m_usCheckSum = Checksum((unsigned char *)&m_acBuffer[1], sizeof(m_acBuffer) - 1);
    DWORD dwLength;
    BOOL bRet = FALSE;

    switch (m_uInterface) {
        case 1:
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
            break;

        case 2:
            bRet = m_comIO.WriteFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds);
            break;

        case 3:
        case 4:
        case 5:
            m_acBuffer[2] = static_cast<CHAR>(m_uInterface);
            bRet = m_hidIO2.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
            break;

        default:
            break;
    }

    if (bRet != FALSE) {
        m_uCmdIndex += 2;
    } else {
        Close_Port();
    }

    printf("Write Cmd : %X\n", uCmd);
    return bRet;
}

void ISPLdCMD::SyncPackno()
{
    m_uCmdIndex = 1;
    WriteFile(
        CMD_SYNC_PACKNO,
        (const char *)&m_uCmdIndex,
        4,
        USBCMD_TIMEOUT);
    ReadFile(NULL, 0, USBCMD_TIMEOUT, FALSE);
}

unsigned char ISPLdCMD::GetVersion()
{
    WriteFile(
        CMD_GET_VERSION,
        NULL,
        0,
        USBCMD_TIMEOUT);
    unsigned char ucVersion;
    ReadFile((char *)&ucVersion, 1, USBCMD_TIMEOUT, TRUE);
    return ucVersion;
}

unsigned long ISPLdCMD::GetDeviceID()
{
    WriteFile(
        CMD_GET_DEVICEID,
        NULL,
        0,
        USBCMD_TIMEOUT);
    unsigned long uID;
    ReadFile((char *)&uID, 4, USBCMD_TIMEOUT, TRUE);
    return uID;
}

void ISPLdCMD::ReadConfig(unsigned int config[])
{
    WriteFile(
        CMD_READ_CONFIG,
        NULL,
        0,
        USBCMD_TIMEOUT);
    ReadFile((char *)config, 16, USBCMD_TIMEOUT, TRUE);
}

void ISPLdCMD::UpdateConfig(unsigned int config[], unsigned int response[])
{
    WriteFile(
        CMD_UPDATE_CONFIG,
        (const char *)config,
        16,
        USBCMD_TIMEOUT_LONG);
    ReadFile((char *)response, 16, USBCMD_TIMEOUT_LONG, TRUE);
}

void ISPLdCMD::UpdateAPROM(unsigned long start_addr,
                           unsigned long total_len,
                           unsigned long cur_addr,
                           const char *buffer,
                           unsigned long *update_len)
{
    unsigned long write_len = total_len - (cur_addr - start_addr);
    char acBuffer[
     HID_MAX_PACKET_SIZE_EP
     - 8 /* cmd, index */ ];

    if (start_addr == cur_addr) {
        if (write_len > sizeof(acBuffer) - 8/*start_addr, total_len*/) {
            write_len = sizeof(acBuffer) - 8/*start_addr, total_len*/;
        }

        memcpy(&acBuffer[0], &start_addr, 4);
        memcpy(&acBuffer[4], &total_len, 4);
        memcpy(&acBuffer[8], buffer, write_len);
        WriteFile(
            CMD_UPDATE_APROM,
            acBuffer,
            write_len + 8/*start_addr, total_len*/,
            USBCMD_TIMEOUT_LONG);
        /* First block need erase the chip, need long timeout */
        ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        if (write_len > sizeof(acBuffer)) {
            write_len = sizeof(acBuffer);
        }

        WriteFile(
            0,
            buffer,
            write_len,
            USBCMD_TIMEOUT_LONG);
        ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    if (update_len != NULL) {
        *update_len = write_len;
    }
}

void ISPLdCMD::UpdateNVM(unsigned long start_addr,
                         unsigned long total_len,
                         unsigned long cur_addr,
                         const char *buffer,
                         unsigned long *update_len)
{
    unsigned long write_len = total_len - (cur_addr - start_addr);
    char acBuffer[
     HID_MAX_PACKET_SIZE_EP
     - 8 /* cmd, index */ ];

    if (start_addr == cur_addr) {
        if (write_len > sizeof(acBuffer) - 8/*start_addr, total_len*/) {
            write_len = sizeof(acBuffer) - 8/*start_addr, total_len*/;
        }

        memcpy(&acBuffer[0], &start_addr, 4);
        memcpy(&acBuffer[4], &total_len, 4);
        memcpy(&acBuffer[8], buffer, write_len);
        WriteFile(
            CMD_UPDATE_DATAFLASH,
            acBuffer,
            write_len + 8/*start_addr, total_len*/,
            USBCMD_TIMEOUT_LONG);
        /* First block need erase the chip, need long timeout */
        ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        if (write_len > sizeof(acBuffer)) {
            write_len = sizeof(acBuffer);
        }

        WriteFile(
            0,
            buffer,
            write_len,
            USBCMD_TIMEOUT_LONG);
        ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    if (update_len != NULL) {
        *update_len = write_len;
    }
}

void ISPLdCMD::Test()
{
    SetInterface(1, _T(""));

    if (!Open_Port()) {
        printf("Open_Port failed\n");
        return;
    }

    printf("Open_Port successed\n");
    SyncPackno();
    unsigned char cVer = GetVersion();
    unsigned long sDID = GetDeviceID();
    unsigned int config[2];
    ReadConfig(config);
    printf("GetVersion: %X\n", cVer);
    printf("GetDeviceID: %X\n", sDID);
    printf("ReadConfig: %X, %X\n", config[0], config[1]);
    Close_Port();
}


BOOL ISPLdCMD::EraseAll()
{
    BOOL ret = FALSE;

    if (WriteFile(CMD_ERASE_ALL, NULL,	0, USBCMD_TIMEOUT_LONG)) {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    return ret;
}

BOOL ISPLdCMD::CMD_Connect(DWORD dwMilliseconds)
{
    if (m_uUSB_PID == 0xA316) {
        m_uCmdIndex = 1;
    }

    BOOL ret = FALSE;
    DWORD dwStart = GetTickCount();
    unsigned long uID;

    if (WriteFile(CMD_CONNECT, NULL, 0)) {
        ret = ReadFile((char *)&uID, 4, dwMilliseconds, FALSE);
    }

    if (ret) {
        m_uCmdIndex = 3;
        bSupport_SPI = (uID == 0x001540EF);
    } else if (m_uInterface == 3) {
        // For SPI interface, Nu-Link2 can get respones even if Target Device is not connected to Nu-Link2.
        // Without this delay, hidapi will crash due to heap corruption.
        DWORD dwDelay = (GetTickCount() - dwStart);

        if (dwDelay < dwMilliseconds) {
            Sleep(dwMilliseconds - dwDelay);
        }
    }

    return ret;
}

BOOL ISPLdCMD::CMD_Resend()
{
    BOOL ret = FALSE;

    if (WriteFile(CMD_RESEND_PACKET, NULL, 0, USBCMD_TIMEOUT_LONG)) {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, FALSE);
    }

    return ret;
}

BOOL ISPLdCMD::RunAPROM()
{
    return WriteFile(CMD_RUN_APROM, NULL, 0, USBCMD_TIMEOUT_LONG);
}

BOOL ISPLdCMD::RunLDROM()
{
    return WriteFile(CMD_RUN_LDROM, NULL, 0, USBCMD_TIMEOUT_LONG);
}


#if (SUPPORT_SPIFLASH)
BOOL ISPLdCMD::Cmd_ERASE_SPIFLASH(unsigned long offset, unsigned long total_len)
{
    unsigned long os = offset;

    while (os < total_len) {
        WriteFile(
            CMD_ERASE_SPIFLASH,
            (const char *)&os,
            4,
            USBCMD_TIMEOUT_LONG);

        if (ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE)) {
            os += 64 * 1024;
        } else {
            return FALSE;
        }
    }

    return TRUE;
}

BOOL ISPLdCMD::Cmd_UPDATE_SPIFLASH(unsigned long start_addr,
                                   unsigned long total_len,
                                   const char *buffer)
{
    unsigned long write_len = 0;
    char acBuffer[
     HID_MAX_PACKET_SIZE_EP
     - 8 /* cmd, index */ ];

    while (write_len < total_len) {
        unsigned long addr = start_addr + write_len;
        unsigned long len = (total_len - write_len);

        if (len > (sizeof(acBuffer) - 8)) {
            len = (sizeof(acBuffer) - 8);
        }

        memcpy(&acBuffer[0], &addr, 4);
        memcpy(&acBuffer[4], &len, 4);
        memcpy(&acBuffer[8], buffer + write_len, len);
        WriteFile(
            CMD_UPDATE_SPIFLASH,
            acBuffer,
            len + 8/*start_addr, total_len*/,
            USBCMD_TIMEOUT_LONG);

        if (ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE)) {
            write_len += len;
        } else {
            return FALSE;
        }
    }

    return TRUE;
}
#endif
