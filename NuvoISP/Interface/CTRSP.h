#ifndef INC__CTRSP_H__
#define INC__CTRSP_H__

#include <string>
#include <mutex>
#include "BleIO.h"

class CTRSP
{
public:
    enum INTF_E {
        INTF_E_WIFI = 0,
        INTF_E_BLE
    };

public:
    CTRSP(void);
    ~CTRSP(void);

    BOOL OpenDevice(INTF_E eInterface = INTF_E_WIFI, ...);
    void CloseDevice(void);

    size_t GetDeviceLength(void) const;
    BOOL SetActiveDevice(size_t szIndex);
    size_t GetActiveDevice(void) const;
    std::string GetActiveDeviceName(void) const;

    BOOL Write(const CHAR *pcBuffer, size_t szLen, DWORD *pdwLength);
    BOOL Read(CHAR *pcBuffer, size_t szLen, DWORD *pdwLength, DWORD dwTime = 5000/*ms*/);
    void ClearReadBuf(void);

private:
    INTF_E m_Intf;
    SOCKET m_conn_socket;
    CBleIO m_BleIO;
    USHORT m_ServF01;
    USHORT m_CharW01;
    USHORT m_CharN01;
    CHAR m_aucRxBuf[5][64];
    ULONG m_auRxCount[5];
    std::mutex m_RxMutex;

public:
    friend void BLE_NotifyCallback(UCHAR *pData, ULONG uDataSize, void *pContext);
};

#endif
