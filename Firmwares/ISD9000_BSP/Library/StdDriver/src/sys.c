/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/10/16 16:49p $
 * @brief    ISD9000 Series SYS driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/
	
/** @addtogroup ISD9000_SYS_Driver SYS Driver
  @{
*/
	
/** @addtogroup ISD9000_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
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
    return ((SYS->REGLCTL == SYS_REGLCTL_REGLCTL_Msk)?0:1);
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
    (u8Lock)?(SYS->REGLCTL=0):0;
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
  * @brief  This function get device ID.
  * @return Device ID
  */
uint32_t  SYS_ReadDeviceID(void)
{
    return SYS->DEVICEID;
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
  * - \ref GPIO_RST
  * - \ref TMR0_RST
  * - \ref TMR1_RST
  * - \ref TMR2_RST
  * - \ref TMRF_RST
  * - \ref PDMA_RST
  * - \ref SPI0_RST
  * - \ref SPIM_RST
  * - \ref PWM0_RST
  * - \ref PWM1_RST
  * - \ref ADC_RST
  * - \ref DPWM_RST
  * @return None
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    uint8_t u8Lock = SYS_Unlock();
	*(volatile uint32_t *)((uint32_t)&(SYS->IPRST0) + (u32ModuleIndex>>24)) |= 1<<(u32ModuleIndex & 0x00ffffff);
    *(volatile uint32_t *)((uint32_t)&(SYS->IPRST0) + (u32ModuleIndex>>24)) &= ~(1<<(u32ModuleIndex & 0x00ffffff));
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function disables POR(Power On Reset) function. This can avoid power noise to cause the POR active again!
  * @return None
  */
void SYS_DisablePOR(void)
{
	uint8_t u8Lock = SYS_Unlock();
	//SYS->PORCTL = 0x5AA5;
	SYS_Lock(u8Lock);
}
/**
  * @brief  This function enables POR(Power On Reset) function.
  * @return None
  */
void SYS_EnablePOR(void)
{
	uint8_t u8Lock = SYS_Unlock();
	//SYS->PORCTL = 0x0000;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function turns the DC offset voltage between SPK+/SPK-..
  * @return None
  */
void SYS_SetPADCOffset(uint8_t u8AdjValue)
{
	uint8_t u8Lock = SYS_Unlock();
	//SYS->PA_ADJ = (u8AdjValue&0x1F)|0x20;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function set ICE_TCK and ICE_TDA to be ICE CLCOK/ ICE DIO,for debug purpose.
  * @return None
  */
void SYS_EnableICEPin(void)
{
	uint8_t u8Lock = SYS_Unlock();
	SYS->ICE_MFP |= SYS_ICE_MFP_ICE_EN_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function set ICE_TCK and ICE_TDA to be ICE CLCOK/ ICE DIO,for debug purpose.
  * @return None
  */
void SYS_DisableICEPin(void)
{
	uint8_t u8Lock = SYS_Unlock();
	SYS->ICE_MFP &= (~SYS_ICE_MFP_ICE_EN_Msk);
	SYS_Lock(u8Lock);
}
/*@}*/ /* end of group ISD9000_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_SYS_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
