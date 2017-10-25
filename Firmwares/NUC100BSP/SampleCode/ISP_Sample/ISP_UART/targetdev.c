
#include "targetdev.h"
#include "ISP_USER.h"

// Supports 32K/64K (APROM)

//the smallest of APROM size is 32K (32K, 64K, 128K)
uint32_t GetApromSize()
{
    uint32_t size = 0x8000, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);
        if(result < 0) {
            return size;
        } else
            size *= 2;
    } while(1);
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;

    *size = 0;

    if(g_apromSize >= 0x20000) {
        FMC_Read_User(Config0, &uData);
        if((uData&0x01) == 0) { //DFEN enable
            FMC_Read_User(Config1, &uData);
            if(uData > g_apromSize || (uData & 0x1FF))//avoid config1 value from error
                uData = g_apromSize;

            *addr = uData;
            *size = g_apromSize - uData;
        } else {
            *addr = g_apromSize;
            *size = 0;
        }
    } else {
        *addr = 0x1F000;
        *size = 4096;//4K
    }
}
