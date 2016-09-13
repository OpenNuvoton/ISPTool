
#include "targetdev.h"
#include "ISP_USER.h"

//the APROM size is 17.5K
uint32_t GetApromSize()
{
    return 0x4600;
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    FMC_Read_User(Config0, &uData);
    if((uData&0x01) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);
        if(uData > g_apromSize || uData < 0x200)//avoid config1 value from error
            uData = g_apromSize;

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}

