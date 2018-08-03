
#include "targetdev.h"
#include "ISP_USER.h"

#define CONFIG0_DFEN                0x01

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    // LDROM Enable: LDROM = 4K, APROM = 145K - 4KB = 141 K
    g_apromSize = 141 * 1024;
    *size = 0;

    if ((uData & CONFIG0_DFEN) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);

        if (uData > g_apromSize || (uData & 0x3FF)) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}
