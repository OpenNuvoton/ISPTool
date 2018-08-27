#include <stdio.h>
#include "fmc_user.h"

int FMC_Proc(unsigned int u32Cmd, unsigned int addr_start, unsigned int addr_end, unsigned int *data)
{
    unsigned int u32Addr, Reg;

    for (u32Addr = addr_start; u32Addr < addr_end; data++) {
        FMC->ISPCMD = u32Cmd;
        FMC->ISPADR = u32Addr;

        if (u32Cmd == FMC_ISPCMD_PROGRAM) {
            FMC->ISPDAT = *data;
        }

        FMC->ISPTRG = 0x1;
        __ISB();

        while (FMC->ISPTRG);                /* Waiting for ISP Done */

        Reg = FMC->ISPCON;

        if (Reg & FMC_ISPCON_ISPFF_Msk) {
            FMC->ISPCON = Reg;
            return -1;
        }

        if (u32Cmd == FMC_ISPCMD_READ) {
            *data = FMC->ISPDAT;
        }

        if (u32Cmd == FMC_ISPCMD_PAGE_ERASE) {
            u32Addr += FMC_FLASH_PAGE_SIZE;
        } else {
            u32Addr += 4;
        }
    }

    return 0;
}

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
    return FMC_Proc(FMC_ISPCMD_PROGRAM, u32Addr, u32Addr + 4, &u32Data);
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
int FMC_Read_User(unsigned int u32Addr, unsigned int *data)
{
    return FMC_Proc(FMC_ISPCMD_READ, u32Addr, u32Addr + 4, data);
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
int FMC_Erase_User(unsigned int u32Addr)
{
    return FMC_Proc(FMC_ISPCMD_PAGE_ERASE, u32Addr, u32Addr + 4, 0);
}

void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int *data)    // Read data from flash
{
    FMC_Proc(FMC_ISPCMD_READ, addr_start, addr_end, data);
    return;
}

void WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data)  // Write data into flash
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, addr_start, addr_end, data);
    return;
}

void EraseAP(unsigned int addr_start, unsigned int addr_end)
{
    FMC_Proc(FMC_ISPCMD_PAGE_ERASE, addr_start, addr_end, 0);
    return;
}

void UpdateConfig(unsigned int *data, unsigned int *res)
{
    FMC_ENABLE_CFG_UPDATE();
    FMC_Proc(FMC_ISPCMD_PAGE_ERASE, Config0, Config0 + 8, 0);
    FMC_Proc(FMC_ISPCMD_PROGRAM, Config0, Config0 + 8, data);

    if (res) {
        FMC_Proc(FMC_ISPCMD_READ, Config0, Config0 + 8, res);
    }

    FMC_DISABLE_CFG_UPDATE();
}
