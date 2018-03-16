#include <stdio.h>
#include "FMC_USER.h"

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32data  32-bit Data to program
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of TRM.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int FMC_Write_User(unsigned int u32Addr, unsigned int u32Data)
{
    // call api from fmc.c
    FMC_Write(u32Addr, u32Data);
    return 0;
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function. User can check the status of
 *              Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int FMC_Read_User(unsigned int u32Addr, unsigned int * data)
{
    // call api from fmc.c
    *data = FMC_Read(u32Addr);
    return 0;
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int32_t FMC_Erase_User(uint32_t u32PageAddr)
{
    // call api from fmc.c
    return FMC_Erase(u32PageAddr);
}

void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int* data)    // Read data from flash
{
    unsigned int rLoop;

    for ( rLoop = addr_start; rLoop < addr_end; rLoop += 4 ) {
        FMC_Read_User(rLoop, data);
        data++;
    }

    return;
}

void WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data)  // Write data into flash
{
    unsigned int wLoop;

    for ( wLoop = addr_start; wLoop < addr_end; wLoop+=4 ) {
        FMC_Write_User(wLoop, *data);
        data++;
    }

}
