/**************************************************************************//**
 * @file     NuEdu-Basic01_IrDA_NEC.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/17 4:41p $
 * @brief    NuEdu-Basic01 IrDA NEC driver header file
 *
 * @note
 * Copyright (C) 2013-2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __NuEdu_Basic01_IrDA_NEC_H__
#define __NuEdu_Basic01_IrDA_NEC_H__

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS IrDA NEC Exported Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS
#define     Percent             0.04        ///< timing accourcy 
#define     MaxValue            0xFFFF      ///< Max vlue for capture timer

#define     IR_LDC_MAX          (13460 * (1 + Percent)) ///< Max LDC length     
#define     IR_LDC_MIN          (13460 * (1 - Percent)) ///< Min LDC length     
// Repeater code range
#define     IR_RPC_MAX          (11280 * (1 + Percent)) ///< Max RPC length     
#define     IR_RPC_MIN          (11280 * (1 - Percent)) ///< Min RPC length
// Bit = 1 range
#define     IR_BIT_1_MAX        (2236 * (1 + Percent)) ///< Max BIT_1 length 
#define     IR_BIT_1_MIN        (2236 * (1 - Percent)) ///< Min BIT_1 length
// Bit = 0 range
#define     IR_BIT_0_MAX        (1120 * (1 + Percent)) ///< Max BIT_0 length 
#define     IR_BIT_0_MIN        (1120 * (1 - Percent)) ///< Min BIT_0 length
/// @endcond


typedef void (*IrDA_Code_Exe)(volatile uint8_t* IR_CODE);
void SendNEC(uint8_t* data);
void IrDA_NEC_TxRx_Init(IrDA_Code_Exe pfnIrDA_Code_Exe);
void IrDa_NEC_Rx(uint32_t u32Time);



/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS IrDA NEC Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

#endif//__NuEdu_Basic01_IrDA_NEC_H__
/*** (C) COPYRIGHT 2013-2014 Nuvoton Technology Corp. ***/
