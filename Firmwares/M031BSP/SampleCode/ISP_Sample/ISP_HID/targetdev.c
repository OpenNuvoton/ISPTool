
#include "NuMicro.h"
#include "ISP_USER.h"

uint32_t GetApromSize()
{
    uint32_t size = 0x4000, data;
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

    FMC_Read_User(Config0, &uData);
    if((uData&0x01) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);
        uData &= 0x000FFFFF;

        if(uData > g_apromSize || (uData & 0x1FF))//avoid config1 value from error
            uData = g_apromSize;

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}

