#ifndef INC__ISP_LD_H__
#define INC__ISP_LD_H__
#pragma once

#define SUPPORT_SPIFLASH (1)

#include "CScopedMutex.hpp"
#include "Interface\CHidIO2.h"
#include "Interface\CUartIO.h"
#define HID_MAX_PACKET_SIZE_EP 64
class CUartIO;

class ISPLdCMD
{
protected:
    CHAR	m_acBuffer[HID_MAX_PACKET_SIZE_EP + 1];
    unsigned long m_uCmdIndex;
    USHORT	m_usCheckSum;

    // Interface
    ULONG			m_uInterface;
    ULONG			m_uUSB_PID;		// for compatibility
    CString			m_strComNum;
    CHidIO2			m_hidIO;
    CHidIO2			m_hidIO2;
    CUartIO			m_comIO;
    BOOL			m_bOpenPort;
    CMutex2			m_Mutex;


    BOOL ReadFile(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex = TRUE);
    BOOL WriteFile(unsigned long uCmd, const char *pcBuffer = NULL, DWORD dwLen = 0, DWORD dwMilliseconds = 20/*USBCMD_TIMEOUT*/);


public:
    BOOL bSupport_SPI;;


    BOOL bResendFlag;	// This flag is set by ReadFile
    ISPLdCMD();
    virtual ~ISPLdCMD();

    bool Check_USB_Link();
    bool Open_Port(BOOL bErrorMsg = FALSE);
    void Close_Port();
    void ReOpen_Port(BOOL bForce = FALSE);

    unsigned short Checksum(const unsigned char *buf, int len)
    {
        int i;
        unsigned short c;

        for (c = 0, i = 0; i < len; i++) {
            c += buf[i];
        }

        return (c);
    }

    enum {
        CMD_GET_VERSION     = 0x000000A6,
        CMD_UPDATE_APROM	= 0x000000A0,
        CMD_SYNC_PACKNO		= 0x000000A4,
        CMD_UPDATE_CONFIG   = 0x000000A1,
        CMD_ERASE_ALL 	    = 0x000000A3,
        CMD_READ_CONFIG     = 0x000000A2,
        CMD_GET_DEVICEID    = 0x000000B1,
        CMD_RUN_APROM		= 0x000000AB,
        CMD_RUN_LDROM		= 0x000000AC,
        CMD_RESET			= 0x000000AD,
        CMD_CONNECT			= 0x000000AE,
        CMD_UPDATE_DATAFLASH = 0x000000C3,
        CMD_RESEND_PACKET   = 0x000000FF,
#if (SUPPORT_SPIFLASH)
        CMD_ERASE_SPIFLASH = 0x000000D0,
        CMD_UPDATE_SPIFLASH = 0x000000D1,
#endif
    };

    BOOL CMD_Connect(DWORD dwMilliseconds = 30);
    BOOL CMD_Resend();


    void SyncPackno();
    unsigned char GetVersion();
    unsigned long GetDeviceID();
    void ReadConfig(unsigned int config[]);
    void UpdateConfig(unsigned int config[], unsigned int response[]);
    void UpdateAPROM(unsigned long start_addr,
                     unsigned long total_len,
                     unsigned long cur_addr,
                     const char *buffer,
                     unsigned long *update_len);
    void UpdateNVM(unsigned long start_addr,
                   unsigned long total_len,
                   unsigned long cur_addr,
                   const char *buffer,
                   unsigned long *update_len);

    BOOL EraseAll();

    BOOL RunAPROM();
    BOOL RunLDROM();

    void Test();
    // it = 1 for HID, str is ignored.
    // it = 2 for UART, str as "COM5".
    void SetInterface(unsigned int it, CString str)
    {
        m_uInterface = it;
        m_strComNum = str;
    };
    CString m_strDevPathName;

#if (SUPPORT_SPIFLASH)
    BOOL Cmd_ERASE_SPIFLASH(unsigned long offset, unsigned long total_len);
    BOOL Cmd_UPDATE_SPIFLASH(unsigned long offset, unsigned long total_len, const char *buffer);
#endif
};

class ISPLdCMD2 : public ISPLdCMD
{
protected:
    ULONG			m_uCMD;
    ULONG			m_uDAT;
public:
    BOOL ReadFile(DWORD dwMilliseconds = 5000);
    BOOL WriteFile(ULONG uCMD, ULONG uDAT, DWORD dwMilliseconds = 20/*USBCMD_TIMEOUT*/);

    ISPLdCMD2();
    virtual ~ISPLdCMD2();

    BOOL CMD_Connect(DWORD dwMilliseconds = 30);
    BOOL CMD_Resend();


    void SyncPackno();
    unsigned char GetVersion();
    unsigned long GetDeviceID();
    void ReadConfig(unsigned int config[]);
    void UpdateConfig(unsigned int config[], unsigned int response[]);
    void UpdateAPROM(unsigned long start_addr,
                     unsigned long total_len,
                     unsigned long cur_addr,
                     const char *buffer,
                     unsigned long *update_len);
    void UpdateNVM(unsigned long start_addr,
                   unsigned long total_len,
                   unsigned long cur_addr,
                   const char *buffer,
                   unsigned long *update_len);

    BOOL EraseAll();

    BOOL RunAPROM();
    BOOL RunLDROM();


};

#ifdef _DEBUG
// Offline Test
class ISPLdCMDTest : public ISPLdCMD
{

public:
    std::vector<unsigned int> m_test_chips;

    BOOL bResendFlag;	// This flag is set by ReadFile

    ISPLdCMDTest()
    {
        bResendFlag = 0; // to make programming pass.
        m_test_chips.push_back(0x00235100); // M2351KIAAE
        m_test_chips.push_back(0x00261000); // M261ZIAAE
        m_test_chips.push_back(0x00235500); // M2354ES
    };

    virtual ~ISPLdCMDTest() {};

    bool Check_USB_Link()
    {
        return true;
    }

    bool Open_Port(BOOL bErrorMsg = FALSE)
    {
        return true;
    }

    void Close_Port() {};

    BOOL CMD_Connect(DWORD dwMilliseconds = 30)
    {
        bSupport_SPI = FALSE;
        return TRUE;
    }

    BOOL CMD_Resend()
    {
        return TRUE;
    }

    void SyncPackno() {};

    unsigned char GetVersion()
    {
        return 0x60;
    }

    unsigned long GetDeviceID()
    {
        static int i = 0;
        unsigned int id = m_test_chips[i];
        i++;

        if (i == m_test_chips.size()) {
            i = 0;
        }

        return id;
    }

    void ReadConfig(unsigned int config[])
    {
        config[0] = 0x11111111;
        config[1] = 0x22222222;
        config[2] = 0x33333333;
        config[3] = 0x44444444;
    }

    void UpdateConfig(unsigned int config[], unsigned int response[])
    {
        response[0] = config[0];
        response[1] = config[1];
        response[2] = config[2];
        response[3] = config[3];
    }

    void UpdateAPROM(unsigned long start_addr,
                     unsigned long total_len,
                     unsigned long cur_addr,
                     const char *buffer,
                     unsigned long *update_len) {};

    void UpdateNVM(unsigned long start_addr,
                   unsigned long total_len,
                   unsigned long cur_addr,
                   const char *buffer,
                   unsigned long *update_len) {};

    BOOL EraseAll()
    {
        return TRUE;
    }

    BOOL RunAPROM()
    {
        return TRUE;
    }

    BOOL RunLDROM()
    {
        return TRUE;
    }

    void SetInterface(unsigned int it, CString str) {};

    CString m_strDevPathName;
};
#endif // #ifdef _DEBUG

#endif
