/**************************************************************************//**
 * @file     mdu.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/04/01 3:00p $
 * @brief    MDU driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __MDU_H__
#define __MDU_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup MDU_Driver MDU Driver
  @{
*/

/** @addtogroup MDU_EXPORTED_CONSTANTS MDU Exported Constants
  @{
*/
#define MAX_PI_dLMT     32767       /*!< Maximum PI_d value */
#define MAX_PI_qLMT     32767       /*!< Maximum PI_q value */

/*@}*/ /* end of group MDU_EXPORTED_CONSTANTS */


/** @addtogroup MDU_EXPORTED_FUNCTIONS MDU Exported Functions
  @{
*/


/**
 * @brief Select active MDU register set.
 * @param[in] mdug The base address of the MDUG module.
 * @param[in] u32MduNo Select the MDU is controlled by MDU0 or MDU1 registers. It can be 0 or 1.
 * @return None.
 * @details This macro is used to select MDU0 or MDU1 registers to control MDU module.
 */
#define MDUG_SELECT(mdug, u32MduNo) \
    (*((__IO uint32_t *) ((uint32_t)&((mdug)->MDUSCON))) = (u32MduNo))

/**
 * @brief Get number of active MDU register set.
 * @param[in] mdug The base address of MDUG module.
 * @return The number of active MDU register set to control MDU module.
 * @details This macro is used to get the number of active MDU register set to control MDU module.
 */
#define MDUG_ACTIVE_MDU(mdug) \
    ((*((__IO uint32_t *) ((uint32_t)&((mdug)->MDUSSTS))) & MDUG_MDUSSTS_MDUACT_Msk) >> MDUG_MDUSSTS_MDUACT_Pos)

/**
 * @brief Return status of MDU control registers.
 * @param[in] mdug The base address of MDUG module.
 * @param[in] u32MduNo Select the MDU register set. It can be 0 or 1.
 * @return TRUE if the specified MDU register set is busy.
 *         FALSE if the specified MDU register set is idle.
 * @details Return the busy or idle state of specified MDU register set.
 */
#define MDUG_ISBUSY(mdug, u32MduNo) \
    ((*((__IO uint32_t *) ((uint32_t)&((mdug)->MDUSSTS))) & MDUG_MDUSSTS_MDU##u32MduNo##BUSY_Msk) >> MDUG_MDUSSTS_MDU##u32MduNo##BUSY_Pos)

/**
 * @brief Enable MDU interrupt(s)
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32IntMask Mask of MDU interrupts. They are MDU_MDUCON_xxx_IE_Msk. xxx can be SVPWM, INVCK, INVPK, PIQ, PID, PK, or CK.
 * @return None.
 * @details This macro is used to enable interrupts of MDU block functions.
 */
#define MDU_ENABLE_INT(mdu, u32IntMask) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= MDU_MDUCON_MDU_IE_Msk | (u32IntMask))

/**
 * @brief Disable MDU interrupt(s)
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32IntMask Mask of MDU interrupts. They are MDU_MDUCON_xxx_IE_Msk. xxx can be SVPWM, INVCK, INVPK, PIQ, PID, PK, or CK.
 * @return None.
 * @details This macro is used to disable interrupt of MDU block functions.
 */
#define MDU_DISABLE_INT(mdu, u32IntMask) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~(u32IntMask))

/**
 * @brief Clear MDU interrupt(s) state
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32IntMask Mask of MDU interrupts. They are MDU_MDUSTS_xxxCFP_Msk. xxx can be SVPWM, INVCK, INVPK, PI_d, PI_q, PK, or CK.
 * @return None.
 * @details This macro is used to clear interrupt state of MDU block functions.
 */
#define MDU_CLEAR_INT(mdu, u32IntMask) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUSTS))) |= (u32IntMask))

/**
 * @brief Enable PI controller Q20 format
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to enable PI controller Q20 format.
 */
#define MDU_ENABLE_PI_Q20(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= MDU_MDUCON_PI_Q20_EN_Msk)

/**
 * @brief Disable PI controller Q20 format
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to disable PI controller Q20 format.
 */
#define MDU_DISABLE_PI_Q20(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~MDU_MDUCON_PI_Q20_EN_Msk)

/**
 * @brief Enable SVPWM Taon Double Value.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to enable SVPWM Taon Double Value.
 */
#define MDU_ENABLE_SVPWM_X2(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= MDU_MDUCON_SVPWM_X2_Msk)

/**
 * @brief Disable SVPWM Taon Double Value.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to disable SVPWM Taon Double Value.
 */
#define MDU_DISABLE_SVPWM_X2(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~MDU_MDUCON_SVPWM_X2_Msk)

/**
 * @brief Enable SVPWM Value Complement.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to enable SVPWM Value Complement.
 */
#define MDU_ENABLE_SVPWM_COMP(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= MDU_MDUCON_SVPWM_COM_Msk)

/**
 * @brief Disable SVPWM Value Complement.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to disable SVPWM Value Complement.
 */
#define MDU_DISABLE_SVPWM_COMP(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~MDU_MDUCON_SVPWM_COM_Msk)

/**
 * @brief Enable SVPWM block function.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to enable SVPWM block function.
 */
#define MDU_ENABLE_SVPWM(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~MDU_MDUCON_SVPWM_DIS_Msk)

/**
 * @brief Disable SVPWM block function.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to disable SVPWM block function.
 */
#define MDU_DISABLE_SVPWM(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= MDU_MDUCON_SVPWM_DIS_Msk)

/**
 * @brief Enable MDU Auto Clear.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to enable auto clear function.
 */
#define MDU_ENABLE_AUTO_CLEAR(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= MDU_MDUCON_ATCLRFG_Msk)

/**
 * @brief Disable MDU Auto Clear.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This macro is used to disable auto clear function for each MDU block.
 */
#define MDU_DISABLE_AUTO_CLEAR(mdu) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~MDU_MDUCON_ATCLRFG_Msk)

/**
 * @brief Enable auto mode of specified MDU blocks
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32AtMask is the MDU_MDUCON_xxxAUTO_Msk. xxx can be SVPWM, INVPK, PIQ, PID, or PK.
 * @return None.
 * @details This macro is used to enable auto mode for each MDU block.
 */
#define MDU_ENABLE_AUTO_MODE(mdu, u32AtMask) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= (u32AtMask))

/**
 * @brief Disable auto mode of specified MDU blocks.
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32AtMask is the MDU_MDUCON_xxxAUTO_Msk. xxx can be SVPWM, INVPK, PIQ, PID, or PK.
 * @return None.
 * @details This macro is used to disable auto mode function for each MDU block.
 */
#define MDU_DISABLE_AUTO_MODE(mdu, u32AtMask) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) &= ~(u32AtMask))

/**
 * @brief Start specified MDU function.
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32StMask is the MDU_MDUCON_xxxSTR_Msk. xxx can be SVPWM, INVPK, INVPK, PIQ, PID, PK, or CK.
 * @return None.
 * @details This macro is used to start the specified MDU block functions.
 */
#define MDU_START(mdu, u32StMask) \
    (*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUCON))) |= (u32StMask))

/**
 * @brief Get SVPWM zone number indicator.
 * @param[in] mdu The base address of the specified MDU module.
 * @return Return SVPWM zone number indicator.
 * @details This macro is used to Get SVPWM zone number indicator.
 */
#define MDU_GET_SVPWM_ZONE(mdu) \
    ((*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUSTS))) & MDU_MDUSTS_ZONE_Msk) >> MDU_MDUSTS_ZONE_Pos)

/**
 * @brief Get complete flags for specified block functions.
 * @param[in] mdu The base address of the specified MDU module.
 * @param[in] u32CfMask is the MDU_MDUSTS_xxxCPF_Msk. xxx can be SVPWM, INVPK, INVPK, PI_d, PI_q, PK, or CK.
 * @return None.
 * @details This macro is used to get complete flags for specified MDU block functions.
 */
#define MDU_GET_FLAG(mdu, u32CfMask) \
    ((*((__IO uint32_t *) ((uint32_t)&((mdu)->MDUSTS))) & (u32CfMask)) ? TRUE : FALSE)


void MDU_Open(MDU_T *mdu);
void MDU_Close(MDU_T *mdu);

/*@}*/ /* end of group MDU_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MDU_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__MDU_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
