/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 15/09/25 9:24a $
 * @brief    MINI51 series SYS driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini51Series.h"
/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_SYS_Driver SYS Driver
  @{
*/


/** @addtogroup MINI51_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief  This function clear the selected system reset source
  * @param  u32Src is system reset source
  * @return None
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSRC |= u32Src;
}

/**
  * @brief  This function get Brown-out detector output status
  * @return 0: System voltage is higher than BOD_VL setting or BOD_EN is 0.
  *         1: System voltage is lower than BOD_VL setting.
  *         Note : If the BOD_EN is 0, this function always return 0.
  */
uint32_t SYS_GetBODStatus(void)
{
    return (SYS->BODCTL & SYS_BODCR_BOD_OUT_Msk)?1:0;
}

/**
  * @brief  This function get the system reset source register value
  * @return Reset source
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSRC);
}

/**
  * @brief  This function check register write-protection bit setting
  * @return 0: Write-protection function is disabled.
  *         1: Write-protection function is enabled.
  */
uint32_t SYS_IsRegLocked(void)
{
    return (SYS->RegLockAddr & SYS_RegLockAddr_RegUnLock_Msk);
}

/**
  * @brief  This function enable register write-protection function
  * @return None
  * @details To lock the protected register to forbid write access
  */
void SYS_LockReg(void)
{
    SYS->RegLockAddr = 0;
}

/**
  * @brief  This function disable register write-protection function
  * @return None
  * @details To unlock the protected register to allow write access
  */
void SYS_UnlockReg(void)
{
    while(SYS->RegLockAddr != SYS_RegLockAddr_RegUnLock_Msk) {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }
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
    SYS->IPRSTC1 |= SYS_IPRSTC1_CHIP_RST_Msk;
}

/**
  * @brief  This function reset CPU.
  * @return None
  */
void SYS_ResetCPU(void)
{
    SYS->IPRSTC1 |= SYS_IPRSTC1_CPU_RST_Msk;
}

/**
  * @brief  This function reset selected modules.
  * @param  u32ModuleIndex is module index. Including :
  *           - \ref ADC_RST
  *           - \ref ACMP_RST
  *           - \ref PWM_RST
  *           - \ref UART_RST
  *           - \ref SPI_RST
  *           - \ref I2C_RST
  *           - \ref TMR1_RST
  *           - \ref TMR0_RST
  *           - \ref GPIO_RST
  * @return None
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    *(volatile uint32_t *)(&(SYS->IPRSTC1) + (u32ModuleIndex>>24)) |= 1<<(u32ModuleIndex & 0x00ffffff);
    *(volatile uint32_t *)(&(SYS->IPRSTC1) + (u32ModuleIndex>>24)) &= ~(1<<(u32ModuleIndex & 0x00ffffff));
}

/**
  * @brief  This function configure BOD function.
  *         Configure BOD reset or interrupt mode and set Brown-out voltage level.
  *         Enable Brown-out function
  * @param  i32Mode is reset or interrupt mode. Including :
  *         - \ref SYS_BODCR_BOD_RST_EN
  *         - \ref SYS_BODCR_BOD_INTERRUPT_EN
  * @param  u32BODLevel is Brown-out voltage level. Including :
  *         - \ref SYS_BODCR_BOD_VL_4_4V
  *         - \ref SYS_BODCR_BOD_VL_3_7V
  *         - \ref SYS_BODCR_BOD_VL_2_7V
  *         - \ref SYS_BODCR_BOD_VL_2_2V
  * @return None
  */
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    SYS->BODCTL |= SYS_BODCR_BOD_VL_EXT_Msk;
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCR_BOD_RSTEN_Msk) | i32Mode;
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCR_BOD_VL_Msk) | u32BODLevel;
}

/**
  * @brief  This function disable BOD function.
  * @return None
  */
void SYS_DisableBOD(void)
{
    SYS->BODCTL &= ~SYS_BODCR_BOD_VL_EXT_Msk;
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCR_BOD_VL_Msk) | SYS_BODCR_BOD_DISABLE ;
}



/*@}*/ /* end of group MINI51_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_SYS_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
