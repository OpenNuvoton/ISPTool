#include "stdafx.h"
#include "ISPMkCMD.h"
#include <stdio.h>

#define USBCMD_TIMEOUT		5000
#define USBCMD_TIMEOUT_LONG	25000

#define printf(...)

ISPMkCMD::ISPMkCMD()
    : m_bOpenPort(FALSE)
    , m_uCmdIndex(18)	//Do not use 0 to avoid firmware already has index 0 occasionally.
{
}

ISPMkCMD::~ISPMkCMD()
{
}

bool ISPMkCMD::Open_Port()
{
    if (m_bOpenPort) {
        return true;
    }

    m_uUSB_PID = 0;
    m_strDevPathName = _T("");
    ScopedMutex scopedLock(m_Mutex);

    switch (m_uInterface) {
        case INTF_HID:
            if (m_hidIO.OpenDevice(0x0416, 0x3F00, -1)) {	// ISP FW >= 0x30
                m_uUSB_PID = 0x3F00;
            } else if (m_hidIO.OpenDevice(0x0416, 0xA316, -1)) {	// ISP FW < 0x30
                m_uUSB_PID = 0xA316;
            } else {
                return false;
            }

            m_strDevPathName = m_hidIO.GetDevicePath();
            break;

        case INTF_UART:
            if (!m_comIO.OpenDevice(m_strComNum)) {
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

void ISPMkCMD::Close_Port()
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

void ISPMkCMD::ReOpen_Port(BOOL bForce)
{
    // Re-Open COM Port to clear previous status
    if (bForce || (m_uInterface == INTF_UART)) {
        Close_Port();
        Open_Port();
    }
}

bool ISPMkCMD::Check_USB_Link()
{
    ScopedMutex scopedLock(m_Mutex);
    return m_bOpenPort ? true : false;
}

BOOL ISPMkCMD::ReadFile(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex)
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
                && (!bCheckIndex || uCmdIndex == m_uCmdIndex - 1)
                && usCheckSum == m_usCheckSum) {
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

BOOL ISPMkCMD::WriteFile(unsigned long uCmd, const char *pcBuffer, DWORD dwLen, DWORD dwMilliseconds)
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

    *((unsigned short *)&m_acBuffer[3]) = 0;
    m_usCheckSum = Checksum((unsigned char *)&m_acBuffer[1], sizeof(m_acBuffer) - 1);
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

void ISPMkCMD::SyncPackno()
{
    if (m_uInterface == INTF_CAN) {
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

unsigned char ISPMkCMD::GetVersion()
{
    if (m_uInterface == INTF_CAN) {
        return 0xCA; // not support
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

unsigned long ISPMkCMD::GetDeviceID()
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

#define FMC_USER_CONFIG_0       0x00300000UL

void ISPMkCMD::ReadConfig(unsigned int config[])
{
    WriteFile(
        CMD_READ_CONFIG,
        NULL,
        0,
        USBCMD_TIMEOUT);
    ReadFile((char *)config, 16, USBCMD_TIMEOUT, TRUE);
}

void ISPMkCMD::UpdateConfig(unsigned int config[], unsigned int response[])
{
    WriteFile(
        CMD_UPDATE_CONFIG,
        (const char *)config,
        16,
        USBCMD_TIMEOUT_LONG);
    ReadFile((char *)response, 16, USBCMD_TIMEOUT_LONG, TRUE);
}

void ISPMkCMD::UpdateAPROM(unsigned long start_addr,
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

void ISPMkCMD::UpdateNVM(unsigned long start_addr,
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

BOOL ISPMkCMD::EraseAll()
{
    if (m_uInterface == INTF_CAN) {
        return FALSE; // not support
    }

    BOOL ret = FALSE;

    if (WriteFile(CMD_ERASE_ALL, NULL,	0, USBCMD_TIMEOUT_LONG)) {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    return ret;
}

BOOL ISPMkCMD::CMD_Connect(DWORD dwMilliseconds)
{
    bSupport_SPI = 0;

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

BOOL ISPMkCMD::CMD_Resend()
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

BOOL ISPMkCMD::RunAPROM()
{
    return WriteFile(CMD_RUN_APROM, NULL, 0, USBCMD_TIMEOUT_LONG);
}

BOOL ISPMkCMD::RunLDROM()
{
    return WriteFile(CMD_RUN_LDROM, NULL, 0, USBCMD_TIMEOUT_LONG);
}

