/**************************************************************************//**
 * @file     opa.h
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/07/29 9:20a $
 * @brief    OP Amplifier (OPA) driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __OPA_H__
#define __OPA_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup OPA_Driver OPA Driver
  @{
*/

/** @addtogroup OPA_EXPORTED_FUNCTIONS OPA Exported Functions
  @{
*/

/**
  * @brief This macro is used to power on the OPA circuit
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return None
  * @details This macro will set OPx_EN (x=0, 1) bit of OPACR register to power on the OPA circuit.
  */
#define OPA_POWER_ON(opa, u32OpaNum) ((opa)->CR |= (1<<(OPA_CR_OP0_EN_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to power down the OPA circuit
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return None
  * @details This macro will clear OPx_EN (x=0, 1) bit of OPACR register to power down the OPA circuit.
  */
#define OPA_POWER_DOWN(opa, u32OpaNum) ((opa)->CR &= ~(1<<(OPA_CR_OP0_EN_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to enable the OPA Schmitt trigger buffer
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return None
  * @details This macro will set OPSCHx_EN (x=0, 1) bit of OPACR register to enable the OPA Schmitt trigger buffer.
  */
#define OPA_ENABLE_SCH_TRIGGER(opa, u32OpaNum) ((opa)->CR |= (1<<(OPA_CR_OPSCH0_EN_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to disable the OPA Schmitt trigger buffer
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return None
  * @details This macro will clear OPSCHx_EN (x=0, 1) bit of OPACR register to disable the OPA Schmitt trigger buffer.
  */
#define OPA_DISABLE_SCH_TRIGGER(opa, u32OpaNum) ((opa)->CR &= ~(1<<(OPA_CR_OPSCH0_EN_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to enable OPA Schmitt trigger digital output interrupt
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return None
  * @details This macro will set OPDIEx (x=0, 1) bit of OPACR register to enable the OPA Schmitt trigger digital output interrupt.
  */
#define OPA_ENABLE_INT(opa, u32OpaNum) ((opa)->CR |= (1<<(OPA_CR_OPDIE0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to disable OPA Schmitt trigger digital output interrupt
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return None
  * @details This macro will clear OPDIEx (x=0, 1) bit of OPACR register to disable the OPA Schmitt trigger digital output interrupt.
  */
#define OPA_DISABLE_INT(opa, u32OpaNum) ((opa)->CR &= ~(1<<(OPA_CR_OPDIE0_Pos+(u32OpaNum))))

/**
  * @brief This macro is used to get OPA digital output state
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return  OPA digital output state
  * @details This macro will return the OPA digital output value.
  */
#define OPA_GET_DIGITAL_OUTPUT(opa, u32OpaNum) (((opa)->SR & (OPA_SR_OPDO0_Msk<<(u32OpaNum)))?1:0)

/**
  * @brief This macro is used to get OPA interrupt flag
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @retval     0 OPA interrupt does not occur.
  * @retval     1 OPA interrupt occurs.
  * @details This macro will return the ACMP interrupt flag.
  */
#define OPA_GET_INT_FLAG(opa, u32OpaNum) (((opa)->SR & (OPA_SR_OPDF0_Msk<<(u32OpaNum)))?1:0)

/**
  * @brief This macro is used to clear OPA interrupt flag
  * @param[in] opa The pointer of the specified OPA module
  * @param[in] u32OpaNum The OPA number. 0 for OPA0; 1 for OPA1.
  * @return   None
  * @details This macro will write 1 to OPDFx (x=0,1) bit of OPASR register to clear interrupt flag.
  */
#define OPA_CLR_INT_FLAG(opa, u32OpaNum) ((opa)->SR = (OPA_SR_OPDF0_Msk<<(u32OpaNum)))



/*@}*/ /* end of group OPA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group OPA_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__OPA_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
