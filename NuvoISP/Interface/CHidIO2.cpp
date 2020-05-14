#include "stdafx.h"
#include "CHidIO2.h"
#include <tchar.h>
#include <vector>

#pragma comment(lib, "Setupapi.lib")

// Copy from hid.c
hid_device *CHidIO2::hid_open(unsigned short vendor_id, unsigned short product_id, int interface_number)
{
    /* TODO: Merge this functions with the Linux version. This function should be platform independent. */
    struct hid_device_info *devs, *cur_dev;
    const char *path_to_open = NULL;
    hid_device *handle = NULL;
    devs = hid_enumerate(vendor_id, product_id);
    cur_dev = devs;

    while (cur_dev) {
        if (cur_dev->vendor_id == vendor_id &&
                cur_dev->product_id == product_id &&
                ((-1 == interface_number) || (cur_dev->interface_number == interface_number))) {
            path_to_open = cur_dev->path;
            break;
        }

        cur_dev = cur_dev->next;
    }

    if (path_to_open) {
        /* Open the device */
        handle = hid_open_path(path_to_open);
        sDevPath = CString(path_to_open);
    } else {
        sDevPath = _T("");
    }

    hid_free_enumeration(devs);
    return handle;
}


CHidIO2::CHidIO2()
    : handle(0)
{
    if (hid_init()) {
        throw (_T("hid_init failed"));
    }
}

CHidIO2::~CHidIO2()
{
    CloseDevice();
    /* Free static HIDAPI objects. */
    hid_exit();
}

void CHidIO2::CloseDevice()
{
    if (handle) {
        hid_close(handle);
        handle = 0;
    }
}

BOOL CHidIO2::OpenDevice(unsigned short vendor_id, unsigned short product_id, int interface_number)
{
    if (!handle) {
        handle = CHidIO2::hid_open(vendor_id, product_id, interface_number);
    }

    BOOL ret = (handle != 0);

    // Set the hid_read() function to be blocking.
    if (ret) {
        hid_set_nonblocking(handle, 0);
    }

    return (handle != 0);
}

BOOL CHidIO2::ReadFile(char *pcBuffer, size_t szMaxLen, DWORD *pdwLength, DWORD dwMilliseconds)
{
    int res;
    unsigned char buf[256];
    res = hid_read_timeout(handle, buf, szMaxLen, dwMilliseconds);
    BOOL ret = (res == (szMaxLen - 1));

    if (ret) {
        memcpy(pcBuffer + 1, buf, res);
    }

    if (pdwLength != NULL) {
        *pdwLength = res;
    }

    return ret;
}

BOOL CHidIO2::WriteFile(const char *pcBuffer, size_t szLen, DWORD *pdwLength, DWORD dwMilliseconds)
{
    int res;
    unsigned char buf[256];
    memcpy(buf, pcBuffer, szLen);
    res = hid_write(handle, buf, szLen);
    BOOL ret = (res == szLen);
    return ret;
}

CString CHidIO2::GetDevicePath()
{
    return sDevPath;
}
