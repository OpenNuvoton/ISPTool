
#include "targetdev.h"
#include "isp_user.h"

//the APROM size is 48K/64K
uint32_t GetApromSize()
{
    uint32_t size = 0xC000, data;
    int result;
    result = FMC_Read_User(size, &data);

    if (result < 0) {
        return 48 * 1024;
    } else {
        return 64 * 1024;
    }
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    FMC_Read_User(Config0, &uData);

    if ((uData & 0x01) == 0) { //DFEN enable
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
