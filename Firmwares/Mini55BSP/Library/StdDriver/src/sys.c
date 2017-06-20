/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/07/06 3:01p $
 * @brief    MINI55 series SYS driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini55Series.h"
/** @addtogroup MINI55_Device_Driver MINI55 Device Driver
  @{
*/

/** @addtogroup MINI55_SYS_Driver SYS Driver
  @{
*/


/** @addtogroup MINI55_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
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
    return !(SYS->REGLCTL & SYS_REGLCTL_REGPROTDIS_Msk);
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
  *                  - \ref SYS_BODCTL_BOD_VL_3_0V
  *                  - \ref SYS_BODCTL_BOD_VL_2_4V
  *                  - \ref SYS_BODCTL_BOD_VL_2_0V
  *                  - \ref SYS_BODCTL_BOD_VL_1_7V
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
    SYS->BODCTL = (SYS->BODCTL & ~(SYS_BODCTL_BODVL1_0_Msk|SYS_BODCTL_BODVL2_Msk)) | u32BODLevel;
}

/**
  * @brief  This function disable BOD function.
  * @return None
  */
void SYS_DisableBOD(void)
{
    SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL1_0_Msk) | SYS_BODCTL_BODVL1_0_Msk;
}

/**
  * @brief  This function enable HIRC trim function.
  * @param[in]  u32TrimSel is trim frequency selection. Including :
  *         - \ref SYS_IRCTCTL_TRIM_22_1184M
  * @param[in]  u32TrimEnInt is HIRC trim interrupt selection. Including :
  *         - \ref SYS_IRCTIEN_TFAILIEN_Msk
  *         - \ref SYS_IRCTIEN_CLKEIEN_Msk
  *         - \ref SYS_IRCTIEN_DISABLE
  * @return None
  */
void SYS_EnableIRCTrim(uint32_t u32TrimSel,uint32_t u32TrimEnInt)
{
    SYS->IRCTIEN = (SYS->IRCTIEN & ~(SYS_IRCTIEN_TFAILIEN_Msk|SYS_IRCTIEN_CLKEIEN_Msk)) | u32TrimEnInt;
    SYS->IRCTCTL = (SYS->IRCTCTL & ~SYS_IRCTCTL_FREQSEL_Msk)|u32TrimSel;
}

/**
  * @brief  This function disable HIRC trim function.
  * @param  None
  * @return None
  */
void SYS_DisableIRCTrim(void)
{
    SYS->IRCTCTL = 0;
}



/*@}*/ /* end of group MINI55_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI55_SYS_Driver */

/*@}*/ /* end of group MINI55_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
