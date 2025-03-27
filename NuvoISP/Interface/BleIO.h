#ifndef INC__BLEIO_H__
#define INC__BLEIO_H__

#include <string>

#ifdef _DEBUG
    #pragma comment(lib, "BleIOd.lib")
#else
    #pragma comment(lib, "BleIO.lib")
#endif

class CBleIO
{
public:
    CBleIO(void);
    ~CBleIO(void);

    BOOL OpenDevice(GUID LongUuid);
    BOOL OpenDevice(USHORT ShortUuid);
    void CloseDevice(void);

    size_t GetDeviceLength(void) const;
    BOOL SetActiveDevice(size_t szIndex);
    size_t GetActiveDevice(void) const;
    std::string GetActiveDeviceAddr() const;

    BOOL SetCharacteristic(GUID LongUuid);
    BOOL SetCharacteristic(USHORT ShortUuid);

    BOOL Write(const CHAR *pcBuffer, size_t szLen, DWORD *pdwLength);
    BOOL Read(CHAR *pcBuffer, size_t szLen, DWORD *pdwLength);
    BOOL SubScribe(BOOL bNotification, BOOL bIndication);
    BOOL RegisterNotify(void (*pCallback)(UCHAR *pData, ULONG uDataSize, void *pContext), void *pCallbackContext = NULL);

private:
    CBleIO(const CBleIO &);
    const CBleIO &operator =(const CBleIO &);

    class CImpl;
    CImpl *m_Impl;
};

#endif
