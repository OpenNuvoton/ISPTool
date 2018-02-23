
#include "targetdev.h"
#include "ISP_USER.h"

#define CONFIG0_DFEN                0x01

// Supports 64K/128K (APROM)
uint32_t GetApromSize()
{
    uint32_t size = 0x10000, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);

        if (result < 0) {
            return size;
        } else {
            size *= 2;
        }
    } while (1);
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    // LDROM Enable: LDROM = 4K, APROM = 145K - 4KB = 141 K
    g_apromSize = GetApromSize();
    *size = 0;

    if ((uData & CONFIG0_DFEN) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);

        if (uData > g_apromSize || (uData & 0x1FF)) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}
