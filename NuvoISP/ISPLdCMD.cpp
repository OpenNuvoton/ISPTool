#include "stdafx.h"
#include "ISPLdCMD.h"
#include <stdio.h>

#define USBCMD_TIMEOUT       5000
#define USBCMD_TIMEOUT_LONG  25000

#define BLE_SERV_F01_UUID    0xABF0
#define BLE_CHAR_W01_UUID    0xABF1
#define BLE_CHAR_N01_UUID    0xABF2    

#define printf(...)

#define CONFIG_64

ISPLdCMD::ISPLdCMD()
    : m_bOpenPort(FALSE)
    , m_uCmdIndex(18)   //Do not use 0 to avoid firmware already has index 0 occasionally.
    , m_uInterface(0)
{
}

ISPLdCMD::~ISPLdCMD()
{
}

bool ISPLdCMD::Open_Port()
{
    if (m_bOpenPort)
    {
        return true;
    }

    m_uUSB_PID = 0;
    m_strDevPathName = _T("");
    m_strBDName = _T("");
    ScopedMutex scopedLock(m_Mutex);

    switch (m_uInterface)
    {
        case INTF_HID:
            if (m_hidIO.OpenDevice(0x0416, 0x3F00, -1))     // ISP FW >= 0x30
            {
                m_uUSB_PID = 0x3F00;
            }
            else if (m_hidIO.OpenDevice(0x0416, 0xA316, -1))        // ISP FW < 0x30
            {
                m_uUSB_PID = 0xA316;
            }
            else
            {
                return false;
            }

            m_strDevPathName = m_hidIO.GetDevicePath();
            break;

        case INTF_UART:
            if (!m_comIO.OpenDevice(m_strComNum))
            {
                return false;
            }

            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
        case INTF_CAN:
        case INTF_LIN:
            if (m_hidIO.OpenDevice(0x0416, 0x5201, 5))      // Nu-Link2 with ISP-Bridge
            {
                m_uUSB_PID = 0x5201;
            }
            else if (m_hidIO.OpenDevice(0x0416, 0x5203, 5))     // Nu-Link2 with ISP-Bridge
            {
                m_uUSB_PID = 0x5203;
            }
            else if (m_hidIO.OpenDevice(0x0416, 0x2006, 4))
            {
                m_uUSB_PID = 0x2006;
            }
            else if (m_hidIO.OpenDevice(0x0416, 0x2009, 4))
            {
                m_uUSB_PID = 0x2009;
            }
            else if (m_hidIO.OpenDevice(0x0416, 0x3F10, -1))        // ISP-Bridge
            {
                m_uUSB_PID = 0x3F10;
            }
            else
            {
                return false;
            }

            m_strDevPathName = m_hidIO.GetDevicePath();
            break;

        case INTF_WIFI:
            if (!m_trsp.OpenDevice(CTRSP::INTF_E_WIFI, m_strIPAddress, m_strIPPort))
            {
                return false;
            }

            break;

        case INTF_BLE:
            if (!m_trsp.OpenDevice(CTRSP::INTF_E_BLE, BLE_SERV_F01_UUID, BLE_CHAR_W01_UUID, BLE_CHAR_N01_UUID))
            {
                return false;
            }

            m_strBDName = m_trsp.GetActiveDeviceName().c_str();
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
    m_strBDName = _T("");
    ScopedMutex scopedLock(m_Mutex);

    if (!m_bOpenPort)
    {
        return;
    }

    m_bOpenPort = FALSE;

    switch (m_uInterface)
    {
        case INTF_HID:
            m_hidIO.CloseDevice();
            break;

        case INTF_UART:
            m_comIO.CloseDevice();
            break;

        case INTF_WIFI:
        case INTF_BLE:
            m_trsp.CloseDevice();
            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
        case INTF_CAN:
        case INTF_LIN:
            m_hidIO.CloseDevice();
            break;

        default:
            break;
    }
}

BOOL ISPLdCMD::WriteFileCAN(ULONG uCMD, ULONG uDAT, DWORD dwMilliseconds)
{
    if (!m_bOpenPort)
    {
        throw _T("There is no Nu-Link connected to a USB port.");
    }
    else if (m_uInterface != INTF_CAN)
    {
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

    if (!m_bOpenPort)
    {
        throw _T("There is no Nu-Link connected to a USB port.");
    }
    else if (m_uInterface != INTF_CAN)
    {
        throw _T("This API is used by CAN only.");
    }

    DWORD dwLength;

    if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds))
    {
        return FALSE;
    }

    /* Check if correct package index was read */
    //m_acBuffer[0];    //For HID internal usage, ignored.
    ULONG uCMD = *((ULONG *)&m_acBuffer[1]);
    ULONG uDAT = *((ULONG *)&m_acBuffer[5]);

    if (uCMD != m_uCmdCAN)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

void ISPLdCMD::ReOpen_Port(BOOL bForce)
{
    // Re-Open COM Port to clear previous status
    if (bForce || (m_uInterface == INTF_UART))
    {
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

    while (1)
    {
        if (!m_bOpenPort)
        {
            throw _T("There is no Nu-Link connected to a USB port.");
        }
        else if (m_uInterface == INTF_CAN)
        {
            throw _T("This API can not be used by CAN.");
        }

        DWORD dwLength;

        switch (m_uInterface)
        {
            case INTF_HID:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds))
                {
                    return FALSE;
                }

                break;

            case INTF_UART:
                if (!m_comIO.ReadFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds))
                {
                    printf("NG in m_comIO.ReadFile\n");
                    return FALSE;
                }

                break;

            case INTF_WIFI:
            case INTF_BLE:
                if (!m_trsp.Read(m_acBuffer + 1, 64, &dwLength, dwMilliseconds))
                {
                    return FALSE;
                }

                break;

            case INTF_SPI:
            case INTF_I2C:
            case INTF_RS485:
            case INTF_LIN:
                if (!m_hidIO.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds))
                {
                    return FALSE;
                }

                break;

            default:
                return FALSE;
                break;
        }

        /* Check if correct package index was read */
        //m_acBuffer[0];    //For HID internal usage, ignored.
        USHORT usCheckSum = *((USHORT *)&m_acBuffer[1]);
        ULONG uCmdIndex = *((ULONG *)&m_acBuffer[5]);

        if (dwLength >= 8
        && (!bCheckIndex || uCmdIndex == m_uCmdIndex - 1)
        && usCheckSum == m_usCheckSum)
        {
            if (szMaxLen > dwLength - 8)
            {
                szMaxLen = dwLength - 8;
            }

            if (pcBuffer != NULL && szMaxLen > 0)
            {
                memcpy(pcBuffer, m_acBuffer + 9, szMaxLen);
            }

            return TRUE;
        }
        else if (m_uUSB_PID == 0xA316)
        {
            SleepEx(10, TRUE);
        }
        else
        {
            printf("dwLength = %d, uCmdIndex = %d, %d, usCheckSum = %d, %d\n", dwLength, uCmdIndex, m_uCmdIndex, usCheckSum, m_usCheckSum);
            bResendFlag = TRUE;
            break;
        }
    }

    return FALSE;
}

BOOL ISPLdCMD::WriteFile(unsigned long uCmd, const char *pcBuffer, DWORD dwLen, DWORD dwMilliseconds)
{
    if (!m_bOpenPort)
    {
        throw _T("There is no Nu-Link connected to a USB port.");
    }
    else if (m_uInterface == INTF_CAN)
    {
        throw _T("This API can not be used by CAN.");
    }

    /* Set new package index value */
    DWORD dwCmdLength = dwLen;

    if (dwCmdLength > sizeof(m_acBuffer) - 9)
    {
        dwCmdLength = sizeof(m_acBuffer) - 9;
    }

    memset(m_acBuffer, 0, sizeof(m_acBuffer));
    //m_acBuffer[0] = 0x00; //Always 0x00
    *((ULONG *)&m_acBuffer[1]) = uCmd;
    *((ULONG *)&m_acBuffer[5]) = m_uCmdIndex;

    if (pcBuffer != NULL && dwCmdLength > 0)
    {
        memcpy(m_acBuffer + 9, pcBuffer, dwCmdLength);
    }

    m_usCheckSum = Checksum((unsigned char *)&m_acBuffer[1], sizeof(m_acBuffer) - 1);
    DWORD dwLength;
    BOOL bRet = FALSE;

    switch (m_uInterface)
    {
        case INTF_HID:
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
            break;

        case INTF_UART:
            bRet = m_comIO.WriteFile(m_acBuffer + 1, 64, &dwLength, dwMilliseconds);
            break;

        case INTF_WIFI:
        case INTF_BLE:
            m_trsp.ClearReadBuf();
            bRet = m_trsp.Write(m_acBuffer + 1, 64, &dwLength);
            break;

        case INTF_SPI:
        case INTF_I2C:
        case INTF_RS485:
        case INTF_LIN:
            m_acBuffer[2] = static_cast<CHAR>(m_uInterface);
            bRet = m_hidIO.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
            break;

        default:
            break;
    }

    if (bRet != FALSE)
    {
        m_uCmdIndex += 2;
    }
    else
    {
        Close_Port();
    }

    printf("Write Cmd : %X\n", uCmd);
    return bRet;
}

void ISPLdCMD::SyncPackno()
{
    if (m_uInterface == INTF_CAN)
    {
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
    if (m_uInterface == INTF_CAN)
    {
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

unsigned long ISPLdCMD::GetDeviceID()
{
    if (m_uInterface == INTF_CAN)
    {
        BOOL ret = FALSE;

        if (WriteFileCAN(CAN_CMD_GET_DEVICEID, 0))
        {
            ret = ReadFileCAN();
        }

        if (ret)
        {
            return *((ULONG *)&m_acBuffer[5]);
        }
        else
        {
            return 0;
        }
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
#define SPEC_FMC_USER_CONFIG_0  0x0F300000UL

void ISPLdCMD::ReadConfig(unsigned int config[])
{
    if (m_uInterface == INTF_CAN)
    {
        unsigned int addr = (!bSpec_addr) ? FMC_USER_CONFIG_0 : SPEC_FMC_USER_CONFIG_0;

        for (int i = 0; i < 14; i++)
        {
            if (WriteFileCAN(CAN_CMD_READ_CONFIG, addr + 4 * i))
            {
                if (ReadFileCAN())
                {
                    config[i] = *((ULONG *)&m_acBuffer[5]);
                }
            }
        }

        return;
    }
    WriteFile(
        CMD_READ_CONFIG,
        NULL,
        0,
        USBCMD_TIMEOUT);
    ReadFile((char*)&config[0], 56, USBCMD_TIMEOUT, TRUE);
}

void ISPLdCMD::ReadConfig_Ext(unsigned int config[], unsigned int i)
{
    if (m_uInterface == INTF_CAN)
    {
        unsigned int addr = (!bSpec_addr) ? FMC_USER_CONFIG_0 : SPEC_FMC_USER_CONFIG_0;
        
        unsigned int index = i;
        if (i >= 16 && i <= 18) {
            index += 16;  // CONFIG_16 at 0x0F300080 not 0x0F300040
        }

        if (WriteFileCAN(CAN_CMD_READ_CONFIG, addr + 4 * index))
        {
            if (ReadFileCAN())
            {
                config[i] = *((ULONG*)&m_acBuffer[5]);
            }
        }

        return;
    }

    unsigned int index = i;
    if (i >= 16 && i <= 18) {
        index += 16;  // CONFIG_16 at 0x0F300080 not 0x0F300040
    }
    WriteFile(
        CMD_READ_CONFIG_EXT,
        (const char*)&index,
        4,
        USBCMD_TIMEOUT);
    ReadFile((char*)&config[i], 4, USBCMD_TIMEOUT, TRUE);
}

void ISPLdCMD::UpdateConfig(unsigned int config[], unsigned int response[])
{
    if (m_uInterface == INTF_CAN)
    {
        unsigned int addr = (!bSpec_addr) ? FMC_USER_CONFIG_0 : SPEC_FMC_USER_CONFIG_0;

        for (int i = 0; i < 14; i++)
        {
            if (WriteFileCAN(addr + 4 * i, config[i]))
            {
                if (ReadFileCAN())
                {
                    response[i] = *((ULONG *)&m_acBuffer[5]);
                }
            }
        }

        return;
    }
    
    WriteFile(
        CMD_UPDATE_CONFIG,
        (const char*)&config[0],
        56,
        USBCMD_TIMEOUT_LONG);
    ReadFile((char*)&response[0], 56, USBCMD_TIMEOUT_LONG, TRUE);

}

void ISPLdCMD::UpdateConfig_Ext(unsigned int config[], unsigned int response[], unsigned int i)
{
    if (m_uInterface == INTF_CAN)
    {
        unsigned int addr = (!bSpec_addr) ? FMC_USER_CONFIG_0 : SPEC_FMC_USER_CONFIG_0;

        if (WriteFileCAN(addr + 4 * i, config[i]))
        {
            if (ReadFileCAN())
            {
                response[i] = *((ULONG*)&m_acBuffer[5]);
            }
        }
        return;
    }
    char ext_buffer[8];
    unsigned int index = i;
    if (i >= 16 && i <= 18) {
        index += 16;  // CONFIG_16 at 0x0F300080 not 0x0F300040
    }

#ifdef CONFIG_64
    unsigned int j = i - i % 2;
    unsigned int index_j = index - index % 2;
    memcpy(&ext_buffer[0], &index_j, 4);
    memcpy(&ext_buffer[4], &config[j], 4);
    if (j + 1 <= 18) {
        memcpy(&ext_buffer[8], &config[j + 1], 4);
    }
    else {
		unsigned int empty = 0xFFFFFFFF;
        memcpy(&ext_buffer[8], &empty, 4);
    }
    WriteFile(
        CMD_UPDATE_CONFIG_EXT,
        ext_buffer,
        12,
        USBCMD_TIMEOUT_LONG);
    ReadFile((char*)&response[j], 8, USBCMD_TIMEOUT_LONG, TRUE);
#else
    memcpy(&ext_buffer[0], &index, 4);
    memcpy(&ext_buffer[4], &config[i], 4);

    WriteFile(
        CMD_UPDATE_CONFIG_EXT,
        ext_buffer,
        8,
        USBCMD_TIMEOUT_LONG);
    ReadFile((char*)&response[i], 4, USBCMD_TIMEOUT_LONG, TRUE);
#endif
}

void ISPLdCMD::UpdateAPROM(unsigned long start_addr,
                           unsigned long total_len,
                           unsigned long cur_addr,
                           const char *buffer,
                           unsigned long *update_len,
                           unsigned long program_64bit)
{
    if (m_uInterface == INTF_CAN)
    {
        bResendFlag = 1;
        unsigned long write_len = total_len - (cur_addr - start_addr);
        int m_write_len = (program_64bit) ? 8 : 4;
        if (write_len > m_write_len)
        {
            write_len = m_write_len;
        }

        if (write_len)
        {
            if (program_64bit) {
				BOOL ret_1 = 1, ret_2 = 1, ret_3 = 1;
                unsigned long data_1 = 0;
                unsigned long data_2 = 0;
                memcpy(&data_1, buffer, 4);
                memcpy(&data_2, buffer + 4, 4);
                if (WriteFileCAN(cur_addr, data_1))
                {
                    if (ReadFileCAN())
                    {
                        ret_1 = 0;
                    }
                }
                if (WriteFileCAN(cur_addr + 4, data_2))
                {
                    if (ReadFileCAN())
                    {
                        ret_2 = 0;
                    }
                }
                if (WriteFileCAN(CAN_CMD_SECOND_READ, cur_addr + 4)) {
                    if (ReadFileCAN())
                    {
                        ret_3 = 0;
                    }
                }

                bResendFlag = ret_1 | ret_2 | ret_3;

            }
            else {
                unsigned long data = 0;
                memcpy(&data, buffer, write_len);

                if (WriteFileCAN(cur_addr, data))
                {
                    if (ReadFileCAN())
                    {
                        bResendFlag = 0;
                    }
                }
            }
        }

        if (update_len != NULL)
        {
            *update_len = write_len;
        }

        return;
    }

    unsigned long write_len = total_len - (cur_addr - start_addr);
    char acBuffer[
        HID_MAX_PACKET_SIZE_EP
        - 8 /* cmd, index */ ];

    if (start_addr == cur_addr)
    {
        if (write_len > sizeof(acBuffer) - 8/*start_addr, total_len*/)
        {
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
    }
    else
    {
        if (write_len > sizeof(acBuffer))
        {
            write_len = sizeof(acBuffer);
        }

        WriteFile(
            0,
            buffer,
            write_len,
            USBCMD_TIMEOUT_LONG);
        ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    if (update_len != NULL)
    {
        *update_len = write_len;
    }
}

void ISPLdCMD::UpdateNVM(unsigned long start_addr,
                         unsigned long total_len,
                         unsigned long cur_addr,
                         const char *buffer,
                         unsigned long *update_len,
                         unsigned long program_64bit)
{
    if (m_uInterface == INTF_CAN)
    {
        UpdateAPROM(start_addr, total_len, cur_addr, buffer, update_len, program_64bit);
        return;
    }

    unsigned long write_len = total_len - (cur_addr - start_addr);
    char acBuffer[
        HID_MAX_PACKET_SIZE_EP
        - 8 /* cmd, index */ ];

    if (start_addr == cur_addr)
    {
        if (write_len > sizeof(acBuffer) - 8/*start_addr, total_len*/)
        {
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
    }
    else
    {
        if (write_len > sizeof(acBuffer))
        {
            write_len = sizeof(acBuffer);
        }

        WriteFile(
            0,
            buffer,
            write_len,
            USBCMD_TIMEOUT_LONG);
        ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    if (update_len != NULL)
    {
        *update_len = write_len;
    }
}

BOOL ISPLdCMD::EraseAll()
{
    if (m_uInterface == INTF_CAN)
    {
        return FALSE; // not support
    }

    BOOL ret = FALSE;

    if (WriteFile(CMD_ERASE_ALL, NULL, 0, USBCMD_TIMEOUT_LONG))
    {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE);
    }

    return ret;
}

BOOL ISPLdCMD::CMD_Connect(DWORD dwMilliseconds)
{
    bSupport_SPI = 0;
    bSpec_addr = 0;

    if (m_uInterface == INTF_CAN)
    {
        BOOL ret = FALSE;

        if (WriteFileCAN(CAN_CMD_GET_DEVICEID, 0))
        {
            ret = ReadFileCAN();
        }

        if (ret)
        {
            ULONG CAN_uID = *((ULONG*)&m_acBuffer[5]);
            // M460, M2L31, M55M1
            bSpec_addr = ((CAN_uID == 0x00551000) || ((CAN_uID & 0xFFFFFF00) == 0x01F31000)
            || ((CAN_uID & 0xFFFFF000) == 0x01B46000) || ((CAN_uID & 0xFFFFF000) == 0x01C46000));
        }

        return ret;
    }

    if (m_uUSB_PID == 0xA316)
    {
        m_uCmdIndex = 1;
    }

    BOOL ret = FALSE;
    DWORD dwStart = GetTickCount();
    unsigned long uID;

    if (WriteFile(CMD_CONNECT, NULL, 0))
    {
        ret = ReadFile((char *)&uID, 4, dwMilliseconds, FALSE);
    }

    if (ret)
    {
        m_uCmdIndex = 3;
        bSupport_SPI = (uID == 0x001540EF);
    }
    else if (m_uInterface == 3)
    {
        // For SPI interface, Nu-Link2 can get respones even if Target Device is not connected to Nu-Link2.
        // Without this delay, hidapi will crash due to heap corruption.
        DWORD dwDelay = (GetTickCount() - dwStart);

        if (dwDelay < dwMilliseconds)
        {
            Sleep(dwMilliseconds - dwDelay);
        }
    }

    return ret;
}

BOOL ISPLdCMD::CMD_Resend()
{
    if (m_uInterface == INTF_CAN)
    {
        return FALSE; // not support
    }

    BOOL ret = FALSE;

    if (WriteFile(CMD_RESEND_PACKET, NULL, 0, USBCMD_TIMEOUT_LONG))
    {
        ret = ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, FALSE);
    }

    return ret;
}

BOOL ISPLdCMD::RunAPROM()
{
    if (m_uInterface == INTF_CAN)
    {
        return (WriteFileCAN(CAN_CMD_RUN_APROM, 0));
    }
    else
    {
        return WriteFile(CMD_RUN_APROM, NULL, 0, USBCMD_TIMEOUT_LONG);
    }
}

BOOL ISPLdCMD::RunLDROM()
{
    if (m_uInterface == INTF_CAN)
    {
        return FALSE; // not support
    }
    else
    {
        return WriteFile(CMD_RUN_LDROM, NULL, 0, USBCMD_TIMEOUT_LONG);
    }
}

BOOL ISPLdCMD::Cmd_ERASE_SPIFLASH(unsigned long offset, unsigned long total_len)
{
    if (m_uInterface == INTF_CAN)
    {
        return FALSE; // not support
    }

    unsigned long os = offset;

    while (os < total_len)
    {
        WriteFile(
            CMD_ERASE_SPIFLASH,
            (const char *)&os,
            4,
            USBCMD_TIMEOUT_LONG);

        if (ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE))
        {
            os += 64 * 1024;
        }
        else
        {
            return FALSE;
        }
    }

    return TRUE;
}

BOOL ISPLdCMD::Cmd_UPDATE_SPIFLASH(unsigned long start_addr,
                                   unsigned long total_len,
                                   const char *buffer)
{
    if (m_uInterface == INTF_CAN)
    {
        return FALSE; // not support
    }

    unsigned long write_len = 0;
    char acBuffer[
        HID_MAX_PACKET_SIZE_EP
        - 8 /* cmd, index */ ];

    while (write_len < total_len)
    {
        unsigned long addr = start_addr + write_len;
        unsigned long len = (total_len - write_len);

        if (len > (sizeof(acBuffer) - 8))
        {
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

        if (ReadFile(NULL, 0, USBCMD_TIMEOUT_LONG, TRUE))
        {
            write_len += len;
        }
        else
        {
            return FALSE;
        }
    }

    return TRUE;
}
