
#include "targetdev.h"
#include "ISP_USER.h"

// Nano100B is equipped with 32K/64K/123K bytes APROM
#if defined(TARGET_NANO100B)
uint32_t GetApromSize()
{
    uint32_t size = 0x8000, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);
        if(result < 0) {
            if(size == 0x20000)
                size = 123 * 1024;
            return size;
        } else
            size *= 2;
    } while(1);
}

#elif defined(TARGET_M451)
// Supports 40K/72K/128K/256K bytes application ROM (APROM)
uint32_t GetApromSize()
{
    uint32_t data;
    int result, i = 0;
    unsigned int size[4] = {40*1024, 72*1024, 128*1024, 256*1024};

    while(i != 4) {
        result = FMC_Read_User(size[i], &data);
        if(result < 0)
            return size[i];
        else
            i++;
    }
    return 0;
}

#else
// Mini51DE is equipped with 4K/8K/16K bytes APROM
// Mini58DE is equipped with 32K bytes APROM
// Nano100A is equipped with 32K/64K bytes APROM
// Nano103 is equipped with 32K/64K bytes APROM
// Nano103 is equipped with 256K/512K bytes APROM
uint32_t GetApromSize()
{
    //the smallest of APROM size is 2K
    uint32_t size = 0x800, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);
        if(result < 0) {
            return size;
        } else
            size *= 2;
    } while(1);
}
#endif

#if defined(TARGET_NUC472_442)
#define PAGE_SIZE			0x800
#else
#define PAGE_SIZE			0x200
#endif

// Data Flash is shared with APROM.
// The size and start address are defined in CONFIG1.
void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;

    *size = 0;

    FMC_Read_User(Config0, &uData);
    if((uData&0x01) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);
        if(uData > g_apromSize || uData & (PAGE_SIZE - 1))//avoid config1 value from error
            uData = g_apromSize;

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}

//void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
//{
//    uint32_t uData;

//    *size = 0;

//    // Data Flash is shared with APROM.
//    // The size and start address are defined in CONFIG1.
//    #if defined(TARGET_MINI51) || defined(TARGET_MINI58)
//    FMC_Read_User(Config0, &uData);
//    if((uData&0x01) == 0) { //DFEN enable
//        FMC_Read_User(Config1, &uData);
//        if(uData > g_apromSize || uData < 0x200)//avoid config1 value from error
//            uData = g_apromSize;

//        *addr = uData;
//        *size = g_apromSize - uData;
//    } else {
//        *addr = g_apromSize;
//        *size = 0;
//    }
//    #else
//    if(g_apromSize >= 0x20000) {
//        FMC_Read_User(Config0, &uData);
//        if((uData&0x01) == 0) { //DFEN enable
//            FMC_Read_User(Config1, &uData);
//            if(uData > g_apromSize || uData < 0x200)//avoid config1 value from error
//                uData = g_apromSize;

//            *addr = uData;
//            *size = g_apromSize - uData;
//        } else {
//            *addr = g_apromSize;
//            *size = 0;
//        }
//    } else {
//        *addr = 0x1F000;
//        *size = 4096;//4K
//    }
//    #endif
//}
