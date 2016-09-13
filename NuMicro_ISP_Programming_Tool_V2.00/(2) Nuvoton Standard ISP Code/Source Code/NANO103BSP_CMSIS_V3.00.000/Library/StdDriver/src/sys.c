/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 16/01/28 3:37p $
 * @brief    Nano 103 SYS driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Nano103.h"
/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_SYS_Driver SYS Driver
  @{
*/


/** @addtogroup NANO103_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief      Clear reset source
  * @param[in]  u32Src is system reset source. Including :
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_PORF_Msk
    *             - \ref SYS_RSTSTS_LOCKRF_Msk
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSTS |= u32Src;
}

/**
  * @brief      Get Brown-out detector output status
  * @param      None
  * @retval     0 System voltage is higher than BOD_VL setting or BOD_EN is 0.
  * @retval     1 System voltage is lower than BOD_VL setting.
  * @details    This function get Brown-out detector output status.
  */
uint32_t SYS_GetBODStatus(void)
{
    return ((SYS->BODCTL & SYS_BODCTL_BODOUT_Msk) >> SYS_BODCTL_BODOUT_Pos);
}

/**
  * @brief  This function get the system reset source register value
  * @param  None
  * @return Reset source
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief  This function check register write-protection bit setting
  * @param  None
  * @return 0: Write-protection function is disabled.
  *         1: Write-protection function is enabled.
  */
uint32_t SYS_IsRegLocked(void)
{
    return !(SYS->REGLCTL & SYS_REGLCTL_REGLCTL_Msk);
}

/**
  * @brief  This function get product ID.
  * @param  None
  * @return Product ID
  */
uint32_t  SYS_ReadPDID(void)
{
    return SYS->PDID;
}

/**
  * @brief  This function reset chip.
  * @param  None
  * @return None
  */
void SYS_ResetChip(void)
{
    SYS->IPRST1 |= SYS_IPRST1_CHIPRST_Msk;
}

/**
  * @brief  This function reset CPU.
  * @param  None
  * @return None
  */
void SYS_ResetCPU(void)
{
    SYS->IPRST1 |= SYS_IPRST1_CPURST_Msk;
}

/**
  * @brief  This function reset selected modules.
  * @param[in]  u32ModuleIndex is module index. Including :
  *          - \ref CHIP_RST
  *          - \ref CPU_RST
  *          - \ref DMA_RST
  *          - \ref SC1_RST
  *          - \ref SC0_RST
  *          - \ref ADC_RST
  *          - \ref ACMP0_RST
  *          - \ref PWM0_RST
  *          - \ref UART1_RST
  *          - \ref UART0_RST
  *          - \ref SPI3_RST
  *          - \ref SPI2_RST
  *          - \ref SPI1_RST
  *          - \ref SPI0_RST
  *          - \ref I2C1_RST
  *          - \ref I2C0_RST
  *          - \ref TMR3_RST
  *          - \ref TMR2_RST
  *          - \ref TMR1_RST
  *          - \ref TMR0_RST
  *          - \ref GPIO_RST
  * @return None
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    *(volatile uint32_t *)((uint32_t)&(SYS->IPRST1) + (u32ModuleIndex>>24)) |= 1<<(u32ModuleIndex & 0x00ffffff);
    *(volatile uint32_t *)((uint32_t)&(SYS->IPRST1) + (u32ModuleIndex>>24)) &= ~(1<<(u32ModuleIndex & 0x00ffffff));
}

/**
  * @brief  This function configure Normal BOD function.
  *         Configure BOD reset or interrupt mode and set Brown-out voltage level.
  *         Enable Brown-out function
  * @param[in]  i32Mode is reset or interrupt mode.
  *             - \ref SYS_BODCTL_BOD_RST_EN
  *             - \ref SYS_BODCTL_BOD_INTERRUPT_EN
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *         - \ref SYS_BODCTL_BODVL_1_7V
  *         - \ref SYS_BODCTL_BODVL_1_8V
  *         - \ref SYS_BODCTL_BODVL_1_9V
  *         - \ref SYS_BODCTL_BODVL_2_0V
  *         - \ref SYS_BODCTL_BODVL_2_1V
  *         - \ref SYS_BODCTL_BODVL_2_2V
  *         - \ref SYS_BODCTL_BODVL_2_3V
  *         - \ref SYS_BODCTL_BODVL_2_4V
  *         - \ref SYS_BODCTL_BODVL_2_5V
  *         - \ref SYS_BODCTL_BODVL_2_6V
  *         - \ref SYS_BODCTL_BODVL_2_7V
  *         - \ref SYS_BODCTL_BODVL_2_8V
  *         - \ref SYS_BODCTL_BODVL_2_9V
  *         - \ref SYS_BODCTL_BODVL_3_0V
  *         - \ref SYS_BODCTL_BODVL_3_1V
  * @return None
  */
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    SYS->BODCTL = (SYS->BODCTL & ~0xFFFF) | SYS_BODCTL_BODEN_Msk | (i32Mode | u32BODLevel);
}

/**
  * @brief  This function disable Normal BOD function.
  * @param  None
  * @return None
  */
void SYS_DisableBOD(void)
{
    SYS->BODCTL = SYS->BODCTL & ~(SYS_BODCTL_BODEN_Msk);
}

/**
  * @brief  This function configure Low Power BOD function only valid in Power Down mode.
  *         Configure Low Power BOD reset or interrupt mode and set Low Power Brown-out voltage level.
  *         Enable Low Power Brown-out function
  * @param[in]  i32Mode is reset or interrupt mode.
  *         - \ref SYS_BODCTL_LPBOD_RST_EN
  *         - \ref SYS_BODCTL_LPBOD_INTERRUPT_EN
  * @param[in]  u32BODLevel is Low Power Brown-out voltage level. Including :
  *         - \ref SYS_BODCTL_LPBODVL_2_0V
  *         - \ref SYS_BODCTL_LPBODVL_2_5V
  * @return None
  */
void SYS_EnableLPBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    SYS->BODCTL = (SYS->BODCTL & ~0xFFFF) | SYS_BODCTL_LPBODEN_Msk | (i32Mode | u32BODLevel);
}

/**
  * @brief  This function disable Low Power BOD function.
  * @param  None
  * @return None
  */
void SYS_DisableLPBOD(void)
{
    SYS->BODCTL = SYS->BODCTL & ~(SYS_BODCTL_LPBODEN_Msk);
}

/**
  * @brief  This function enable HIRC0 trim function.
  * @param[in]  u32TrimSel is trim frequency selection. Including :
  *         - \ref SYS_IRC0TCTL_TRIM_11_0592M
  *         - \ref SYS_IRC0TCTL_TRIM_12M
  *         - \ref SYS_IRC0TCTL_TRIM_12_288M
  *         - \ref SYS_IRC0TCTL_TRIM_16M
  * @param[in]  u32TrimEnInt is HIRC0 trim interrupt selection. Including :
  *         - \ref SYS_IRCTIEN_FAIL_EN
  *         - \ref SYS_IRCTIEN_32KERR_EN
  *         - \ref SYS_IRCTIEN_DISABLE
  * @return None
  */
void SYS_EnableHIRC0Trim(uint32_t u32TrimSel,uint32_t u32TrimEnInt)
{
    SYS->IRC0TIEN = (SYS->IRC0TIEN & ~(SYS_IRC0TIEN_TFAILIEN_Msk | SYS_IRC0TIEN_CLKEIEN_Msk)) | u32TrimEnInt;
    SYS->IRC0TCTL = (SYS->IRC0TCTL & ~SYS_IRC0TCTL_FREQSEL_Msk)|u32TrimSel;
}

/**
  * @brief  This function disable HIRC0 trim function.
  * @param  None
  * @return None
  */
void SYS_DisableHIRC0Trim(void)
{
    SYS->IRC0TCTL = 0;
}

/**
  * @brief  This function enable HIRC1 trim function.
  * @param[in]  u32TrimSel is trim frequency selection.
  *         - \ref SYS_IRC1TCTL_TRIM_36M
  * @param[in]  u32TrimEnInt is HIRC1 trim interrupt selection. Including :
  *         - \ref SYS_IRCTIEN_FAIL_EN
  *         - \ref SYS_IRCTIEN_32KERR_EN
  *         - \ref SYS_IRCTIEN_DISABLE
  * @return None
  */
void SYS_EnableHIRC1Trim(uint32_t u32TrimSel,uint32_t u32TrimEnInt)
{
    SYS->IRC1TIEN = (SYS->IRC1TIEN & ~(SYS_IRC1TIEN_TFAILIEN_Msk | SYS_IRC1TIEN_CLKEIEN_Msk)) | u32TrimEnInt;
    SYS->IRC1TCTL = (SYS->IRC1TCTL & ~SYS_IRC1TCTL_FREQSEL_Msk)|u32TrimSel;
}

/**
  * @brief  This function disable HIRC1 trim function.
  * @param  None
  * @return None
  */
void SYS_DisableHIRC1Trim(void)
{
    SYS->IRC1TCTL = 0;
}

/**
  * @brief  This function enable MIRC trim function.
  * @param[in]  u32TrimSel is trim frequency selection.
  *         - \ref SYS_MIRCTCTL_TRIM_4M
  * @param[in]  u32TrimEnInt is MIRC trim interrupt selection. Including :
  *         - \ref SYS_IRCTIEN_FAIL_EN
  *         - \ref SYS_IRCTIEN_32KERR_EN
  *         - \ref SYS_IRCTIEN_DISABLE
  * @return None
  */
void SYS_EnableMIRCTrim(uint32_t u32TrimSel,uint32_t u32TrimEnInt)
{
    SYS->MIRCTIEN = (SYS->MIRCTIEN & ~(SYS_MIRCTIEN_TFAILIEN_Msk | SYS_MIRCTIEN_CLKEIEN_Msk)) | u32TrimEnInt;
    SYS->MIRCTCTL = (SYS->MIRCTCTL & ~SYS_MIRCTCTL_FREQSEL_Msk)|u32TrimSel;
}

/**
  * @brief  This function disable HIRC0 trim function.
  * @param  None
  * @return None
  */
void SYS_DisableMIRCTrim(void)
{
    SYS->MIRCTCTL = 0;
}

/*@}*/ /* end of group NANO103_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_SYS_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
