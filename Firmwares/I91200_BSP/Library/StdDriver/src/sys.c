/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/12/09 11:40a $
 * @brief    I91200 Series SYS driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/
	
/** @addtogroup I91200_SYS_Driver SYS Driver
  @{
*/
	
/** @addtogroup I91200_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief  This function clear the selected system reset source
  * @param  u32Src is system reset source
  * @return None
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSTS |= u32Src;
}

/**
  * @brief  This function get the system reset source register value
  * @return Reset source
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief  This function check register write-protection bit setting
  * @return 0: Write-protection function is disabled.
  *         1: Write-protection function is enabled.
  */
uint32_t SYS_IsRegLocked(void)
{
  return ( (SYS->REGLCTL & SYS_REGLCTL_REGLCTL_Msk)? 0:1 );
}

/**
  * @brief  This function enable register write-protection function
  * @return None
  * @details To lock the protected register to forbid write access
  */
void SYS_LockReg(void)
{
    SYS->REGLCTL = 0;
}

/**
  * @brief  This function disable register write-protection function
  * @return None
  * @details To unlock the protected register to allow write access
  */
void SYS_UnlockReg(void)
{
    while(SYS->REGLCTL != SYS_REGLCTL_REGLCTL_Msk) 
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }
}

/**
  * @brief  This function enable register write-protection function or not according to input lock optin.
  * @param  u8Lock is lock option. 1 represents to lock protected register. 0 represents not to lock protected register.
  * @return None
  * @details To lock the protected register to forbid write access
  */
void SYS_Lock(uint8_t u8Lock)
{
	if ( u8Lock )
		SYS->REGLCTL = 0;
}

/**
  * @brief  This function disable register write-protection function and return previous lock state before calling this function.
  * @return 1: represent the previous state is locked, 0: represent the previous state is unlocked.
  * @details To unlock the protected register to allow write access
  */
uint8_t SYS_Unlock(void)
{
	if ( ((SYS->REGLCTL)&SYS_REGLCTL_REGLCTL_Msk) == 1 )
		return 0;
	
    while(SYS->REGLCTL != SYS_REGLCTL_REGLCTL_Msk) 
	{
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

	return 1;
}

/**
  * @brief  This function get product ID.
  * @return Product ID
  */
uint32_t  SYS_ReadPDID(void)
{
    return SYS->PDID;
}

/**
  * @brief  This function reset chip.
  * @return None
  */
void SYS_ResetChip(void)
{
	uint8_t u8Lock = SYS_Unlock();
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function reset CPU.
  * @return None
  */
void SYS_ResetCPU(void)
{
	uint8_t u8Lock = SYS_Unlock();
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function reset selected modules.
  * @param  u32ModuleIndex is module index. Including :
  * - \ref CHIP_RST
  * - \ref CPU_RST
  * - \ref PDMA_RST
  * - \ref TMR0_RST
  * - \ref TMR1_RST
  * - \ref I2C0_RST
  * - \ref SPI0_RST
  * - \ref DPWM_RST
  * - \ref UART0_RST
  * - \ref BIQ_RST
  * - \ref PWM0_RST
  * - \ref I2S0_RST
  * - \ref ANA_RST
  * - \ref SARADC_RST
  * - \ref RTC_RST
  * - \ref UART1_RST
  * - \ref SPI1_RST
  * @return None
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    *(volatile uint32_t *)((uint32_t)&(SYS->IPRST0) + (u32ModuleIndex>>24)) |= 1<<(u32ModuleIndex & 0x00ffffff);
    *(volatile uint32_t *)((uint32_t)&(SYS->IPRST0) + (u32ModuleIndex>>24)) &= ~(1<<(u32ModuleIndex & 0x00ffffff));
}

/*@}*/ /* end of group I91200_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SYS_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
