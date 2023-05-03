#ifndef INC__UART_HPP__
#define INC__UART_HPP__

#define BAUD_RATE_115200     (115200)
#define BAUD_RATE_57600      (57600)
#define BAUD_RATE_38400      (38400)
#define BAUD_RATE_19200      (19200)
#define BAUD_RATE_9600       (9600)
#define BAUD_RATE_4800       (4800)

class CUartIO
{
protected:
    HANDLE m_hCOMHandle;

public:
    CUartIO();
    virtual ~CUartIO();
    void CloseDevice();
    BOOL OpenDevice(CString strComNum);
    BOOL ReadFile(char *pcBuffer, DWORD szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds);
    BOOL WriteFile(const char *pcBuffer, DWORD szLen, DWORD *pdwLength, DWORD dwMilliseconds);
    //OVERLAPPED m_overlapped;
};

#endif
