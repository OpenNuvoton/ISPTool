#include "stdafx.h"
#include "ISPLdCMD.h"
#include <stdio.h>

#define USBCMD_TIMEOUT		5000
#define USBCMD_TIMEOUT_LONG	25000

#define NEW_CMD_READ_CONFIG                   0xA2000000
#define NEW_CMD_RUN_APROM                     0xAB000000
#define NEW_CMD_GET_DEVICEID                  0xB1000000
#define FMC_USER_CONFIG_0       0x00300000UL

ISPLdCMD2::ISPLdCMD2()
    : ISPLdCMD()
{
}

ISPLdCMD2::~ISPLdCMD2()
{
}

BOOL ISPLdCMD2::WriteFile(ULONG uCMD, ULONG uDAT, DWORD dwMilliseconds)
{
    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    }

    m_uCMD = uCMD;
    m_uDAT = uDAT;
    memset(m_acBuffer, 0, sizeof(m_acBuffer));
    m_acBuffer[2] = static_cast<CHAR>(m_uInterface);
    *((ULONG *)&m_acBuffer[3]) = uCMD;
    *((ULONG *)&m_acBuffer[7]) = uDAT;
    DWORD dwLength;
    return m_hidIO2.WriteFile(m_acBuffer, 65, &dwLength, dwMilliseconds);
}

BOOL ISPLdCMD2::ReadFile(DWORD dwMilliseconds)
{
    bResendFlag = FALSE;

    if (!m_bOpenPort) {
        throw _T("There is no Nu-Link connected to a USB port.");
    }

    DWORD dwLength;

    if (!m_hidIO2.ReadFile(m_acBuffer, 65, &dwLength, dwMilliseconds)) {
        return FALSE;
    }

    /* Check if correct package index was read */
    //m_acBuffer[0];	//For HID internal usage, ignored.
    ULONG uCMD = *((ULONG *)&m_acBuffer[1]);
    ULONG uDAT = *((ULONG *)&m_acBuffer[5]);

    if (uCMD != m_uCMD) {
        return FALSE;
    } else {
        return TRUE;
    }
}

BOOL ISPLdCMD2::CMD_Connect(DWORD dwMilliseconds)
{
    bSupport_SPI = FALSE;

    if (m_uInterface != 6) {
        return ISPLdCMD::CMD_Connect(dwMilliseconds);
    } else {
        BOOL ret = FALSE;
        DWORD dwStart = GetTickCount();

        if (WriteFile(NEW_CMD_GET_DEVICEID, 0, dwMilliseconds)) {
            ret = ReadFile(dwMilliseconds);
        }

        return ret;
    }
}

void ISPLdCMD2::SyncPackno()
{
    if (m_uInterface != 6) {
        ISPLdCMD::SyncPackno();
    }
}

unsigned char ISPLdCMD2::GetVersion()
{
    if (m_uInterface != 6) {
        return ISPLdCMD::GetVersion();
    } else {
        return 0x40;
    }
}

unsigned long ISPLdCMD2::GetDeviceID()
{
    if (m_uInterface != 6) {
        return ISPLdCMD::GetDeviceID();
    } else {
        BOOL ret = FALSE;

        if (WriteFile(NEW_CMD_GET_DEVICEID, 0)) {
            ret = ReadFile();
        }

        if (ret) {
            return *((ULONG *)&m_acBuffer[5]);
        } else {
            return 0;
        }
    }
}

void ISPLdCMD2::ReadConfig(unsigned int config[])
{
    if (m_uInterface != 6) {
        ISPLdCMD::ReadConfig(config);
    } else {
        for (int i = 0; i < 4; i++) {
            if (WriteFile(NEW_CMD_READ_CONFIG, FMC_USER_CONFIG_0 + 4 * i)) {
                if (ReadFile()) {
                    config[i] = *((ULONG *)&m_acBuffer[5]);
                }
            }
        }
    }
}

void ISPLdCMD2::UpdateConfig(unsigned int config[], unsigned int response[])
{
    if (m_uInterface != 6) {
        ISPLdCMD::UpdateConfig(config, response);
    } else {
        for (int i = 0; i < 4; i++) {
            if (WriteFile(FMC_USER_CONFIG_0 + 4 * i, config[i])) {
                if (ReadFile()) {
                    response[i] = *((ULONG *)&m_acBuffer[5]);
                }
            }
        }
    }
}

void ISPLdCMD2::UpdateAPROM(unsigned long start_addr,
                            unsigned long total_len,
                            unsigned long cur_addr,
                            const char *buffer,
                            unsigned long *update_len)
{
    if (m_uInterface != 6) {
        ISPLdCMD::UpdateAPROM(start_addr, total_len, cur_addr, buffer, update_len);
    } else {
        bResendFlag = 1;
        unsigned long write_len = total_len - (cur_addr - start_addr);

        if (write_len > 4) {
            write_len = 4;
        }

        if (write_len) {
            unsigned long data =  0;
            memcpy(&data, buffer, write_len);

            if (WriteFile(cur_addr, data)) {
                if (ReadFile()) {
                    bResendFlag = 0;
                }
            }
        }

        if (update_len != NULL) {
            *update_len = write_len;
        }

        return;
    }
}

void ISPLdCMD2::UpdateNVM(unsigned long start_addr,
                          unsigned long total_len,
                          unsigned long cur_addr,
                          const char *buffer,
                          unsigned long *update_len)
{
    if (m_uInterface != 6) {
        ISPLdCMD::UpdateNVM(start_addr, total_len, cur_addr, buffer, update_len);
    } else {
        ISPLdCMD2::UpdateAPROM(start_addr, total_len, cur_addr, buffer, update_len);
    }
}

BOOL ISPLdCMD2::EraseAll()
{
    if (m_uInterface != 6) {
        return ISPLdCMD::EraseAll();
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD2::CMD_Resend()
{
    if (m_uInterface != 6) {
        return ISPLdCMD::CMD_Resend();
    } else {
        return FALSE;
    }
}

BOOL ISPLdCMD2::RunAPROM()
{
    if (m_uInterface != 6) {
        return ISPLdCMD::RunAPROM();
    } else {
        return (WriteFile(NEW_CMD_RUN_APROM, 0));
    }
}

BOOL ISPLdCMD2::RunLDROM()
{
    if (m_uInterface != 6) {
        return ISPLdCMD::RunLDROM();
    } else {
        return FALSE;
    }
}
