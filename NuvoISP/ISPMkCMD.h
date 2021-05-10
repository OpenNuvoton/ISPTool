#ifndef INC__ISP_LD_H__
#define INC__ISP_LD_H__
#pragma once

// interface
#define INTF_HID     (1)
#define INTF_UART    (2)
// OpenNuvoton/NuLink2_ISP_Bridge
// https://github.com/OpenNuvoton/NuLink2_ISP_Bridge
#define INTF_SPI     (3)
#define INTF_I2C     (4)
#define INTF_RS485   (5)
#define INTF_CAN     (6)

#include "CScopedMutex.hpp"
#include "Interface\CHidIO2.h"
#include "Interface\CUartIO.h"
#define HID_MAX_PACKET_SIZE_EP 64
class CUartIO;

class ISPMkCMD
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
    CUartIO			m_comIO;
    BOOL			m_bOpenPort;
    CMutex2			m_Mutex;


    BOOL ReadFile(char *pcBuffer, size_t szMaxLen, DWORD dwMilliseconds, BOOL bCheckIndex = TRUE);
    BOOL WriteFile(unsigned long uCmd, const char *pcBuffer = NULL, DWORD dwLen = 0, DWORD dwMilliseconds = 20/*USBCMD_TIMEOUT*/);

public:
    BOOL bSupport_SPI;;


    BOOL bResendFlag;	// This flag is set by ReadFile
    ISPMkCMD();
    virtual ~ISPMkCMD();

    bool Check_USB_Link();
    bool Open_Port();
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
        CMD_ERASE_SPIFLASH  = 0x000000D0,
        CMD_UPDATE_SPIFLASH = 0x000000D1,
    };

    // For Code size consideration, CAN only implements some basic isp commands.
    // Need to return any constant in tool side to pass the flow.
    enum {
        CAN_CMD_READ_CONFIG = 0xA2000000,
        CAN_CMD_RUN_APROM = 0xAB000000,
        CAN_CMD_GET_DEVICEID = 0xB1000000,
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

    // it = 1 for HID, str is ignored.
    // it = 2 for UART, str as "COM5".
    void SetInterface(unsigned int it, CString str)
    {
        m_uInterface = it;
        m_strComNum = str;
    };
    CString m_strDevPathName;


};


#endif
