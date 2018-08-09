#include "targetdev.h"

// Supports 40K/72K/128K/256K bytes application ROM (APROM)
uint32_t GetApromSize()
{
    uint32_t data;
    int result, i = 0;
    unsigned int size[4] = {40 * 1024, 72 * 1024, 128 * 1024, 256 * 1024};

    while (i != 4) {
        result = FMC_Read_User(size[i], &data);

        if (result < 0) {
            return size[i];
        } else {
            i++;
        }
    }

    while (1);	// Error, trap MCU

    return 0;
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    FMC_Read_User(Config0, &uData);

    if ((uData & 0x01) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);

        if ((uData > g_apromSize) || (uData & FMC_FLASH_PAGE_SIZE - 1)) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}
