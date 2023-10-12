#ifndef INC__CTRSP_H__
#define INC__CTRSP_H__

class CTRSP
{
public:
    enum INTF_E
    {
        INTF_E_WIFI = 0
    };

public:
    CTRSP(void);
    ~CTRSP(void);

    BOOL OpenDevice(INTF_E eInterface = INTF_E_WIFI, ...);
    void CloseDevice(void);

    size_t GetDeviceLength(void) const;
    BOOL SetActiveDevice(size_t szIndex);
    size_t GetActiveDevice(void) const;

    BOOL Write(const CHAR *pcBuffer, size_t szLen, DWORD *pdwLength);
    BOOL Read(CHAR *pcBuffer, size_t szLen, DWORD *pdwLength, DWORD dwTime = 5000/*ms*/);
    void ClearReadBuf(void);

private:
    INTF_E m_Intf;
    SOCKET m_conn_socket;
};

#endif
