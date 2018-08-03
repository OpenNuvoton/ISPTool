
#include "NUC029xAN.h"
#include "ISP_USER.h"

//the smallest of APROM size is 16K
uint32_t GetApromSize()
{
    uint32_t size = 0x4000, data;
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
    *addr = 0x1F000;
    *size = 4096;//4K
}

