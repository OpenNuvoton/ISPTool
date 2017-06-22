/*************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series SYS driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "Mini57Series.h"
/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_SYS_Driver SYS Driver
  @{
*/


/** @addtogroup Mini57_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
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
  *             If the BOD function is disabled, this function always return 0.
  */
uint32_t SYS_GetBODStatus(void)
{
    return ((SYS->BODCTL & SYS_BODCTL_BODOUT_Msk) >> SYS_BODCTL_BODOUT_Pos);
}

/**
  * @brief      This function get the system reset source register value
  * @param      None
  * @return     Reset source
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief      This function check register write-protection bit setting
  * @param      None
  * @return     0: Write-protection function is disabled.
  *             1: Write-protection function is enabled.
  */
uint32_t SYS_IsRegLocked(void)
{
    return !(SYS->REGLCTL & SYS_REGLCTL_REGLCTL_Msk);
}

/**
  * @brief      This function get product ID.
  * @param      None
  * @return     Product ID
  */
uint32_t  SYS_ReadPDID(void)
{
    return SYS->PDID & SYS_PDID_PDID_Msk;
}

/**
  * @brief      This function reset chip.
  * @param      None
  * @return     None
  */
void SYS_ResetChip(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
}

/**
  * @brief      This function reset CPU.
  * @param      None
  * @return     None
  */
void SYS_ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
}

/**
  * @brief      This function reset selected modules.
  * @param[in]  u32ModuleIndex is module index. Including :
  *             - \ref GPIO_RST
  *             - \ref TMR0_RST
  *             - \ref TMR1_RST
  *             - \ref ECAP_RST
  *             - \ref PGA_RST
  *             - \ref BPWM_RST
  *             - \ref EPWM_RST
  *             - \ref USCI0_RST
  *             - \ref USCI1_RST
  *             - \ref EADC_RST
  *             - \ref ACMP_RST
  * @return     None
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    SYS->IPRST1 |= u32ModuleIndex;      /* generate reset signals to module controller */
    SYS->IPRST1 &= (~u32ModuleIndex);   /* release from reset state */
}

/**
  * @brief      This function configure Normal BOD function.
  *             Configure BOD reset or interrupt mode and set Brown-out voltage level.
  *             Enable Brown-out function
  * @param[in]  i32Mode is always reset mode for Mini57.
  *             - \ref SYS_BODCTL_BOD_RST_EN
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BOD_VL_2_0V
  *             - \ref SYS_BODCTL_BOD_VL_2_2V
  *             - \ref SYS_BODCTL_BOD_VL_2_4V
  *             - \ref SYS_BODCTL_BOD_VL_2_7V
  *             - \ref SYS_BODCTL_BOD_VL_3_0V
  *             - \ref SYS_BODCTL_BOD_VL_3_7V
  *             - \ref SYS_BODCTL_BOD_VL_4_0V
  *             - \ref SYS_BODCTL_BOD_VL_4_3V
  * @return     None
  */
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    SYS->BODCTL = (SYS->BODCTL & ~0xFFFF) | SYS_BODCTL_BODEN_Msk | (i32Mode | u32BODLevel);
}

/**
  * @brief      This function disable Normal BOD function.
  * @param      None
  * @return     None
  */
void SYS_DisableBOD(void)
{
    SYS->BODCTL = SYS->BODCTL & ~(SYS_BODCTL_BODEN_Msk);
}

/*@}*/ /* end of group Mini57_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_SYS_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
