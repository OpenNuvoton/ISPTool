#include "stdafx.h"
#include "ISPLdCMD.h"
#include <stdio.h>

#define USBCMD_TIMEOUT		5000
#define USBCMD_TIMEOUT_LONG	25000

#define printf(...)

ISPLdCMD::ISPLdCMD()
    : m_bOpenPort(FALSE)
    , m_uCmdIndex(18)	// Do not use 0 to avoid firmware already has index 0 occasionally.
    , m_iIspType(TYPE_MKROM)
{
    memset(m_acPattern, 'a', sizeof(m_acPattern));
}

ISPLdCMD::~ISPLdCMD()
{
}

bool ISPLdCMD::Open_Port()
{
    if (m_bOpenPort) {
        return true;
    }

    m_uUSB_PID = 0;
    m_strDevPathName = _T("");
    ScopedMutex scopedLock(m_Mutex);

    switch (m_uInterface) {
        case INTF_HID:
            if (m_iIspType == TYPE_LDROM) {
                if (m_hidIO.OpenDevice(0x0416, 0x3F00, -1)) {	// ISP FW >= 0x30
                    m_uUSB_PID = 0x3F00;
                } else if (m_hidIO.OpenDevice(0x0416, 0xA316, -1)) {	// ISP FW < 0x30
                    m_uUSB_PID = 0xA316;
                } else {
                    return false;
                }
            } else  if (m_iIspType == TYPE_MKROM) {
                // Temp ID
                if (m_hidIO.OpenDevice(0x0416, 0x3F00, -1)) {	// ISP FW >= 0x30
                    m_uUSB_PID = 0x3F00;
                } else {
                    return false;
                }
            } else {
                throw _T("Invalid ISP Type.");
            }

            m_strDevPathName = m_hidIO.GetDevicePath();
            break;

        case INTF_UART:
            if (!m_comIO.OpenDevice(m_strComNum, m_dwClock)) {
                return false;
            }

            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
        case INTF_CAN:
            if (m_hidIO.OpenDevice(0x0416, 0x5201, 5)) {	// Nu-Link2 with ISP-Bridge
                m_uUSB_PID = 0x5201;
            } else if (m_hidIO.OpenDevice(0x0416, 0x5203, 5)) {	// Nu-Link2 with ISP-Bridge
                m_uUSB_PID = 0x5203;
            } else if (m_hidIO.OpenDevice(0x0416, 0x3F10, -1)) {	// ISP-Bridge
                m_uUSB_PID = 0x3F10;
            } else {
                return false;
            }

            m_strDevPathName = m_hidIO.GetDevicePath();
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
        case INTF_HID:
            m_hidIO.CloseDevice();
            break;

        case INTF_UART:
            m_comIO.CloseDevice();
            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
        case INTF_CAN:
            m_hidIO.CloseDevice();
            break;

        default:
            break;
    }
}

BOOL ISPLdCMD::WriteFileCAN(ULONG uCMD, ULONG uDAT, DWORD dwMilliseconds)
{
    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    } else if (m_uInterface != INTF_CAN) {
        throw _T("This API is used by CAN only.");
    }

    m_uCmdCAN = uCMD;
    m_uDatCAN = uDAT;
    memset(m_acBuffer, 0, sizeof(m_acBuffer));
    m_acBuffer[2] = static_cast<CHAR>(m_uInterface);
    *((ULONG *)&m_acBuffer[3]) = uCMD;
    *((ULONG *)&m_acBuffer[7]) = uDAT;
    DWORD dwLength;
    return m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
}

BOOL ISPLdCMD::ReadFileCAN(DWORD dwMilliseconds)
{
    bResendFlag = FALSE;

    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    } else if (m_uInterface != INTF_CAN) {
        throw _T("This API is used by CAN only.");
    }

    DWORD dwLength;

    if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
        return FALSE;
    }

    /* Check if correct package index was read */
    //m_acBuffer[0];	//For HID internal usage, ignored.
    ULONG uCMD = *((ULONG *)&m_acBuffer[1]);
    ULONG uDAT = *((ULONG *)&m_acBuffer[5]);

    if (uCMD != m_uCmdCAN) {
        return FALSE;
    } else {
        return TRUE;
    }
}

void ISPLdCMD::ReOpen_Port(BOOL bForce)
{
    // Re-Open COM Port to clear previous status
    if (bForce || (m_uInterface == INTF_UART)) {
        Close_Port();
        Open_Port();
    }
}

bool ISPLdCMD::Check_USB_Link()
{
    ScopedMutex scopedLock(m_Mutex);
    return m_bOpenPort ? true : false;
}

BOOL ISPLdCMD::ReadFileLDROM(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex)
{
    bResendFlag = FALSE;

    while (1) {
        if (!m_bOpenPort) {
            throw _T("There is no Nu-Link connected to a USB port.");
        } else if (m_uInterface == INTF_CAN) {
            throw _T("This API can not be used by CAN.");
        }

        DWORD dwLength;

        switch (m_uInterface) {
            case INTF_HID:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
                    return FALSE;
                }

                break;

            case INTF_UART:
                if (!m_comIO.ReadFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds)) {
                    printf("NG in m_comIO.ReadFile\n");
                    return FALSE;
                }

                break;

            case INTF_SPI:
            case INTF_I2C:
            case INTF_RS485:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
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

BOOL ISPLdCMD::WriteFileLDROM(unsigned long uCmd, const char *pcBuffer, DWORD dwLen, DWORD dwMilliseconds)
{
    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    } else if (m_uInterface == INTF_CAN) {
        throw _T("This API can not be used by CAN.");
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
        case INTF_HID:
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
            break;

        case INTF_UART:
            bRet = m_comIO.WriteFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds);
            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
            m_acBuffer[2] = static_cast<CHAR>(m_uInterface);
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
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
    if (m_uInterface == INTF_CAN) {
        return; // not support
    } else if (m_iIspType == TYPE_MKROM) {
        return; // not support
    }

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
    if (m_uInterface == INTF_CAN) {
        return 0xCA; // not support
    } else if (m_iIspType == TYPE_MKROM) {
        return mkChipInfo[1];
    }

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
    if (m_uInterface == INTF_CAN) {
        BOOL ret = FALSE;

        if (WriteFileCAN(CAN_CMD_GET_DEVICEID, 0)) {
            ret = ReadFileCAN();
        }

        if (ret) {
            return *((ULONG *)&m_acBuffer[5]);
        } else {
            return 0;
        }
    } else if (m_iIspType == TYPE_MKROM) {
        return mkChipInfo[0];
    }

    WriteFile(
        CMD_GET_DEVICEID,
        NULL,
        0,
        USBCMD_TIMEOUT);
    unsigned long uID;
    ReadFile((char *)&uID, 4, USBCMD_TIMEOUT, TRUE);
    return uID;
}

#define FMC_USER_CONFIG_0       0x00300000UL

void ISPLdCMD::ReadConfig(unsigned int config[])
{
    if (m_uInterface == INTF_CAN) {
        for (int i = 0; i < 4; i++) {
            if (WriteFileCAN(CAN_CMD_READ_CONFIG, FMC_USER_CONFIG_0 + 4 * i)) {
                if (ReadFileCAN()) {
                    config[i] = *((ULONG *)&m_acBuffer[5]);
                }
            }
        }

        return;
    } else if (m_iIspType == TYPE_MKROM) {
        if (Cmd_GET_CHIP_INFO()) {
            memcpy(config, &mkChipInfo[5], 16);
        }

        return;
    }

    WriteFile(
        CMD_READ_CONFIG,
        NULL,
        0,
        USBCMD_TIMEOUT);
    ReadFile((char *)config, 16, USBCMD_TIMEOUT, TRUE);
}

void ISPLdCMD::UpdateConfig(unsigned int config[], unsigned int response[])
{
    if (m_uInterface == INTF_CAN) {
        for (int i = 0; i < 4; i++) {
            if (WriteFileCAN(FMC_USER_CONFIG_0 + 4 * i, config[i])) {
                if (ReadFileCAN()) {
                    response[i] = *((ULONG *)&m_acBuffer[5]);
                }
            }
        }

        return;
    }  else if (m_iIspType == TYPE_MKROM) {
        if (Cmd_WRITE_DATA_EXT(FMC_CONFIG_BASE, 16, (DWORD *)config)) {
            memcpy(response, config, 16);
        }

        return;
    }

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
    if (m_uInterface == INTF_CAN) {
        bResendFlag = 1;
        unsigned long write_len = total_len - (cur_addr - start_addr);

        if (write_len > 4) {
            write_len = 4;
        }

        if (write_len) {
            unsigned long data = 0;
            memcpy(&data, buffer, write_len);

            if (WriteFileCAN(cur_addr, data)) {
                if (ReadFileCAN()) {
                    bResendFlag = 0;
                }
            }
        }

        if (update_len != NULL) {
            *update_len = write_len;
        }

        return;
    }

    unsigned long write_len = total_len - (cur_addr - start_addr);
    char acBuffer[
     HID_MAX_PACKET_SIZE_EP
     - 8 /* cmd, index */ ];

    if (start_addr == cur_addr) {
        if (write_len > sizeof(acBuffer) - 8/*start_addr, total_len*/) {
            write_len = sizeof(acBuffer) - 8/*start_addr, total_len*/;
        }

        unsigned long uCmd = (m_iIspType == TYPE_LDROM) ? CMD_UPDATE_APROM : CMD_WRITE_DATA_EXT;
        memcpy(&acBuffer[0], &start_addr, 4);
        memcpy(&acBuffer[4], &total_len, 4);
        memcpy(&acBuffer[8], buffer, write_len);
        WriteFile(
            uCmd,
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
    if (m_uInterface == INTF_CAN) {
        UpdateAPROM(start_addr, total_len, cur_addr, buffer, update_len);
        return;
    }

    unsigned long write_len = total_len - (cur_addr - start_addr);
    char acBuffer[
     HID_MAX_PACKET_SIZE_EP
     - 8 /* cmd, index */ ];

    if (start_addr == cur_addr) {
        if (write_len > sizeof(acBuffer) - 8/*start_addr, total_len*/) {
            write_len = sizeof(acBuffer) - 8/*start_addr, total_len*/;
        }

        unsigned long uCmd = (m_iIspType == TYPE_LDROM) ? CMD_UPDATE_DATAFLASH : CMD_WRITE_DATA_EXT;
        memcpy(&acBuffer[0], &start_addr, 4);
        memcpy(&acBuffer[4], &total_len, 4);
        memcpy(&acBuffer[8], buffer, write_len);
        WriteFile(
            uCmd,
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

BOOL ISPLdCMD::EraseAll()
{
    if (m_uInterface == INTF_CAN) {
        return FALSE; // not support
    }  else if (m_iIspType == TYPE_MKROM) {
        return Cmd_ERASE_ALL_FLASH();
    }

    BOOL ret = FALSE;

    if (WriteFile(CMD_ERASE_ALL, NULL,	0, USBCMD_TIMEOUT_LONG)) {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    return ret;
}

BOOL ISPLdCMD::MKROM_Connect(DWORD dwMilliseconds)
{
    if ((m_iIspType != TYPE_MKROM) || (m_uInterface != INTF_UART)) {
        return TRUE; // Always Pass
    }

    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    }

    // BOOL ISPLdCMD::WriteFile
    DWORD dwLength;

    if (m_comIO.WriteFile(m_acPattern, 64, &dwLength, dwMilliseconds)) {
        if (!m_comIO.ReadFile(m_acBuffer, 1, &dwLength, dwMilliseconds)) {
            printf("NG in m_comIO.ReadFile\n");
            return FALSE;
        }

        if (m_acBuffer[0] == 'A') {
            return TRUE;
        } else {
            return FALSE;
        }
    } else {
        Close_Port();
        return FALSE;
    }
}


BOOL ISPLdCMD::CMD_Connect(DWORD dwMilliseconds)
{
    if (m_uInterface == INTF_CAN) {
        BOOL ret = FALSE;

        if (WriteFileCAN(CAN_CMD_GET_DEVICEID, 0)) {
            ret = ReadFileCAN();
        }

        return ret;
    } else if (m_iIspType == TYPE_MKROM) {
        return Cmd_GET_CHIP_INFO();
    }

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
    if (m_uInterface == INTF_CAN) {
        return FALSE; // not support
    }

    BOOL ret = FALSE;

    if (WriteFile(CMD_RESEND_PACKET, NULL, 0, USBCMD_TIMEOUT_LONG)) {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, FALSE);
    }

    return ret;
}

BOOL ISPLdCMD::RunAPROM()
{
    if (m_uInterface == INTF_CAN) {
        return (WriteFileCAN(CAN_CMD_RUN_APROM, 0));
    } else {
        return WriteFile(CMD_RUN_APROM, NULL, 0, USBCMD_TIMEOUT_LONG);
    }
}

BOOL ISPLdCMD::RunLDROM()
{
    if (m_uInterface == INTF_CAN) {
        return FALSE; // not support
    } else {
        return WriteFile(CMD_RUN_LDROM, NULL, 0, USBCMD_TIMEOUT_LONG);
    }
}

BOOL ISPLdCMD::Cmd_ERASE_SPIFLASH(unsigned long offset, unsigned long total_len)
{
    if (m_uInterface == INTF_CAN) {
        return FALSE; // not support
    }

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
    if (m_uInterface == INTF_CAN) {
        return FALSE; // not support
    }

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


BOOL ISPLdCMD::ReadFileMKROM(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex)
{
    size_t szPacket = (szMaxLen == 0) ? 12 : 64;
    bResendFlag = FALSE;

    while (1) {
        if (!m_bOpenPort) {
            throw _T("There is no Nu-Link connected to a USB port.");
        } else if (m_uInterface == INTF_CAN) {
            throw _T("This API can not be used by CAN.");
        }

        DWORD dwLength;

        switch (m_uInterface) {
            case INTF_HID:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
                    return FALSE;
                }

                break;

            case INTF_UART:
                if (!m_comIO.ReadFile(m_acBuffer + 1, szPacket, &dwLength, dwMilliseconds)) {
                    printf("NG in m_comIO.ReadFile\n");
                    return FALSE;
                }

                break;

            case INTF_SPI:
            case INTF_I2C:
            case INTF_RS485:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
                    return FALSE;
                }

                break;

            default:
                return FALSE;
                break;
        }

        /* Check if correct package index was read */
        //m_acBuffer[0];	//For HID internal usage, ignored.
        USHORT usCheckSum = *((USHORT *)&m_acBuffer[3]);
        ULONG uCmdIndex = *((ULONG *)&m_acBuffer[5]);
        ULONG uCmdResult = *((ULONG *)&m_acBuffer[9]);

        if (dwLength >= 12
                && (!bCheckIndex || uCmdIndex == m_uCmdIndex - 1)) {
            if (szMaxLen > dwLength - 12) {
                szMaxLen = dwLength - 12;
            }

            if (pcBuffer != NULL && szMaxLen > 0) {
                memcpy(pcBuffer, m_acBuffer + 13, szMaxLen);
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

BOOL ISPLdCMD::WriteFileMKROM(unsigned long uCmd, const char *pcBuffer, DWORD dwLen, DWORD dwMilliseconds)
{
    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    } else if (m_uInterface == INTF_CAN) {
        throw _T("This API can not be used by CAN.");
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

    m_usCheckSum = Checksum((unsigned char *)&m_acBuffer[5], sizeof(m_acBuffer) - 5);
    // 2021.05.19 new check sum should include the command id.
    m_usCheckSum += uCmd;
    *((unsigned short *)&m_acBuffer[3]) = m_usCheckSum;
    DWORD dwLength;
    BOOL bRet = FALSE;

    switch (m_uInterface) {
        case INTF_HID:
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
            break;

        case INTF_UART:
            bRet = m_comIO.WriteFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds);
            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
            m_acBuffer[2] = static_cast<CHAR>(m_uInterface);
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
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

BOOL ISPLdCMD::ReadFile(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex)
{
    if (m_iIspType == TYPE_LDROM) {
        return ReadFileLDROM(pcBuffer, szMaxLen, dwMilliseconds, bCheckIndex);
    } else {
        return ReadFileMKROM(pcBuffer, szMaxLen, dwMilliseconds, bCheckIndex);
    }
}

BOOL ISPLdCMD::WriteFile(unsigned long uCmd, const char *pcBuffer, DWORD dwLen, DWORD dwMilliseconds)
{
    if (m_iIspType == TYPE_LDROM) {
        return WriteFileLDROM(uCmd, pcBuffer, dwLen, dwMilliseconds);
    } else {
        return WriteFileMKROM(uCmd, pcBuffer, dwLen, dwMilliseconds);
    }
}

/* short ack command - 32-bits x 3:
    [ checksum + cmd_id ] + [ packet number ] + [ result ]
      [31:16]    [15:0]          [31:0]           [31:0]
*/

BOOL ISPLdCMD::Cmd_SET_UART_SPEED(DWORD dwClock)
{
    if (WriteFileMKROM(CMD_SET_UART_SPEED, (const char *)(&dwClock), 4)) {
        return ReadFileMKROM(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD::Cmd_SET_CAN_SPEED(DWORD dwClock)
{
    if (WriteFileMKROM(CMD_SET_CAN_SPEED, (const char *)(&dwClock), 4)) {
        return ReadFileMKROM(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD::Cmd_REBOOT_SOURCE(DWORD rebootSrc, DWORD address)
{
    DWORD Input[] = { rebootSrc, address };
    return WriteFileMKROM(CMD_SET_CAN_SPEED, (const char *)(Input), 8);
}

BOOL ISPLdCMD::Cmd_GOTO_USBDISP(void)
{
    if (WriteFileMKROM(CMD_GOTO_USBDISP)) {
        return ReadFileMKROM(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD::Cmd_ERASE_ALL_FLASH(void)
{
    if (WriteFileMKROM(CMD_ERASE_ALL_FLASH)) {
        return ReadFileMKROM(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD::Cmd_ERASE_PAGE(DWORD address, DWORD page_cnt)
{
    DWORD Input[] = { address, page_cnt };

    if (WriteFileMKROM(CMD_ERASE_PAGE, (const char *)(Input), 8)) {
        return ReadFileMKROM(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD::Cmd_WRITE_DATA_EXT(DWORD address, DWORD byte_length, DWORD *data)
{
    size_t len = (byte_length < 48) ? byte_length : 48;
    DWORD Input[16] = { address, byte_length };
    memcpy(&Input[2], data, len);

    if (WriteFileMKROM(CMD_WRITE_DATA_EXT, (const char *)(Input), len + 16)) {
        return ReadFileMKROM(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    return FALSE;
}

/* long ack command - 32-bits x 16:
   [ checksum + cmd_id ] + [ packet number ] + [ result ] + [ word-0 ~ word-12 ]
     [31:16]    [15:0]          [31:0]           [31:0]
*/

BOOL ISPLdCMD::Cmd_GET_CHIP_INFO(void)
{
    bSupport_SPI = 0;

    if (WriteFileMKROM(CMD_GET_CHIP_INFO)) {
        // don't check index
        if (ReadFileMKROM((char *)(mkChipInfo), 36, USBCMD_TIMEOUT_LONG, FALSE)) {
            m_uCmdIndex = 3;
            return TRUE;
        }
    }

    return FALSE;
}


