
#include "targetdev.h"
#include "isp_user.h"

__align(4) uint8_t usb_rcvbuf[64];
uint8_t bUsbDataReady;

uint32_t GetApromSize()
{
    return 0x20000; // 128K
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    FMC_Read_User(Config0, &uData);

    if ((uData & 0x01) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);

        if (uData > g_apromSize || (uData & (FMC_FLASH_PAGE_SIZE - 1))) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}

