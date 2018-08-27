
#include "targetdev.h"
#include "isp_user.h"

#define CONFIG0_DFEN                0x01
#define CONFIG0_DFVSEN              0x04

// Supports 32K/64K (APROM)
uint32_t GetApromSize()
{
    uint32_t size = 0xA000, data;
    int result;
    result = FMC_Read_User(size, &data);

    if (result < 0) {
        return 32 * 1024;
    } else {
        return 64 * 1024;
    }
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    g_apromSize = GetApromSize();
    *size = 0;
    /* Note: DFVSEN = 1, DATA Flash Size is 4K bytes
             DFVSEN = 0, DATA Flash Size is based on CONFIG1 */
    FMC_Read_User(Config0, &uData);

    if (uData & CONFIG0_DFVSEN) {
        *addr = 0x1F000;
        *size = 4096;//4K
    } else if (uData & CONFIG0_DFEN) {
        g_apromSize += 4096;
        *addr = g_apromSize;
        *size = 0;
    } else {
        g_apromSize += 4096;
        FMC_Read_User(Config1, &uData);

        if (uData > g_apromSize || (uData & 0x1FF)) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
        g_apromSize -= *size;
    }
}
