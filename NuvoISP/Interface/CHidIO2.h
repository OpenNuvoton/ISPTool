#ifndef INC__HID_IO2_H__
#define INC__HID_IO2_H__

#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include <vector>

#include "hidapi.h"

class CHidIO2
{
protected:
    hid_device *handle;
public:
    CHidIO2();
    virtual ~CHidIO2();
    BOOL OpenDevice(unsigned short vendor_id, unsigned short product_id, int interface_number);
    CString GetDevicePath();
    void CloseDevice();
    BOOL ReadFile(char *pcBuffer, size_t szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds);
    BOOL WriteFile(const char *pcBuffer, size_t szLen, DWORD *pdwLength, DWORD dwMilliseconds);
    hid_device *hid_open(unsigned short vendor_id, unsigned short product_id, int interface_number);
    CString sDevPath;
};

#endif
