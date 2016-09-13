/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/04/07 2:47p $ 
 * @brief    NM1200_NM1100 SYS driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/ 
#include "NM1200_NM1100.h"
/** @addtogroup NM1200_NM1100_Device_Driver NM1200_NM1100 Device Driver
  @{
*/

/** @addtogroup NM1200_NM1100_SYS_Driver SYS Driver
  @{
*/


/** @addtogroup NM1200_NM1100_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief  This function clear the selected system reset source
  * @param[in]  u32Src is system reset source
  * @return None
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{ 
  SYS->RSTSTS |= u32Src;
}

/**
  * @brief  This function get Brown-out detector output status  
  * @return 0: System voltage is higher than BOD_VL setting or BOD_EN is 0.
  *         1: System voltage is lower than BOD_VL setting.
  *         Note : If the BOD_EN is 0, this function always return 0.
  */
uint32_t SYS_GetBODStatus(void)
{
  return (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)?1:0;
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
  return (SYS->REGLCTL & SYS_REGLCTL_REGPROTDIS_Msk);
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
  while(SYS->REGLCTL != SYS_REGLCTL_REGPROTDIS_Msk) {
    SYS->REGLCTL = 0x59;
    SYS->REGLCTL = 0x16;
    SYS->REGLCTL = 0x88;  
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
  SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
}

/**
  * @brief  This function reset CPU.
  * @return None
  */
void SYS_ResetCPU(void)
{
  SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
}

/**
  * @brief  This function reset selected modules.
  * @param[in]  u32ModuleIndex is module index. Including :
  *                  - \ref CHIP_RST 
  *                  - \ref CPU_RST
  *                  - \ref ADC_RST 
  *                  - \ref ACMP_RST 
  *                  - \ref PWM_RST  
  *                  - \ref UART0_RST
  *                  - \ref UART1_RST
  *                  - \ref SPI_RST  
  *                  - \ref I2C_RST  
  *                  - \ref TMR1_RST 
  *                  - \ref TMR0_RST 
  *                  - \ref GPIO_RST
  * @return None
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
  *(volatile uint32_t *)(&(SYS->IPRST0) + (u32ModuleIndex>>24)) |= 1<<(u32ModuleIndex & 0x00ffffff);
  *(volatile uint32_t *)(&(SYS->IPRST0) + (u32ModuleIndex>>24)) &= ~(1<<(u32ModuleIndex & 0x00ffffff));
}

/**
  * @brief  This function configure BOD function. 
  *         Configure BOD reset or interrupt mode and set Brown-out voltage level. 
  *         Enable Brown-out function  
  * @param[in]  i32Mode is reset or interrupt mode. Including :   
  *                  - \ref SYS_BODCTL_BOD_RST_EN     
  *                  - \ref SYS_BODCTL_BOD_INTERRUPT_EN  
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *                  - \ref SYS_BODCTL_BOD_VL_4_4V     
  *                  - \ref SYS_BODCTL_BOD_VL_3_7V
  *                  - \ref SYS_BODCTL_BOD_VL_2_7V
  *                  - \ref SYS_BODCTL_BOD_VL_2_2V   
  * @return None
  */
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{   
    SYS->BODCTL |= SYS_BODCTL_BODEN_Msk;
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODRSTEN_Msk) | i32Mode; 
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL1_0_Msk) | u32BODLevel;  
}

/**
  * @brief  This function disable BOD function.  
  * @return None
  */
void SYS_DisableBOD(void)
{ 
  SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;
  SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL1_0_Msk) | SYS_BODCTL_BOD_DISABLE; 
}



/*@}*/ /* end of group NM1200_NM1100_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1200_NM1100_SYS_Driver */

/*@}*/ /* end of group NM1200_NM1100_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
