/**************************************************************************//**
 * @file     acmp.h
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/04/01 7:31p $
 * @brief    Analog Comparator (ACMP) driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ACMP_H__
#define __ACMP_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup ACMP_Driver ACMP Driver
  @{
*/

/** @addtogroup ACMP_EXPORTED_CONSTANTS ACMP Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* CMPCR constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define ACMP_CR_OUTPUT_INV           (1UL << ACMP_CR_ACMPINV_Pos)      /*!< ACMPCR setting for ACMP output inverse function. */
#define ACMP_CR_VNEG_BANDGAP         (1UL << ACMP_CR_CN_Pos)           /*!< ACMPCR setting for selecting band-gap voltage as the source of ACMP V-. */
#define ACMP_CR_VNEG_PIN             (0UL << ACMP_CR_CN_Pos)           /*!< ACMPCR setting for selecting the voltage of ACMP negative input pin as the source of ACMP V-. */
#define ACMP_CR_VPOS_OPA             (1UL << ACMP_CR_CP_Pos)           /*!< ACMPCR setting for selecting the output of OPA as the source of ACMP V+. */
#define ACMP_CR_VPOS_PIN             (0UL << ACMP_CR_CP_Pos)           /*!< ACMPCR setting for selecting the voltage of ACMP positive input pin as the source of ACMP V+. */
#define ACMP_CR_HYSTERESIS_ENABLE    (1UL << ACMP_CR_ACMP_HYS_EN_Pos)  /*!< ACMPCR setting for enabling the hysteresis function. */
#define ACMP_CR_HYSTERESIS_DISABLE   (0UL << ACMP_CR_ACMP_HYS_EN_Pos)  /*!< ACMPCR setting for disabling the hysteresis function. */
#define ACMP_CR_INT_ENABLE           (1UL << ACMP_CR_ACMPIE_Pos)       /*!< ACMPCR setting for enabling the interrupt function. */
#define ACMP_CR_INT_DISABLE          (0UL << ACMP_CR_ACMPIE_Pos)       /*!< ACMPCR setting for disabling the interrupt function. */
#define ACMP_CR_ACMP_ENABLE          (1UL << ACMP_CR_ACMP_EN_Pos)      /*!< ACMPCR setting for enabling the ACMP analog circuit. */
#define ACMP_CR_ACMP_DISABLE         (0UL << ACMP_CR_ACMP_EN_Pos)      /*!< ACMPCR setting for disabling the ACMP analog circuit. */

/*@}*/ /* end of group ACMP_EXPORTED_CONSTANTS */


/** @addtogroup ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/**
  * @brief This macro is used to enable output inverse
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMPINV bit of ACMPCR register to enable output inverse function.
  */
#define ACMP_ENABLE_OUTPUT_INVERSE(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] |= ACMP_CR_ACMPINV_Msk)

/**
  * @brief This macro is used to disable output inverse function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMPINV bit of ACMPCR register to disable output inverse function.
  */
#define ACMP_DISABLE_OUTPUT_INVERSE(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] &= ~ACMP_CR_ACMPINV_Msk)

/**
  * @brief This macro is used to select ACMP negative input source
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Src Comparator negative input selection.  Including:
  *                  - \ref ACMP_CR_VNEG_PIN
  *                  - \ref ACMP_CR_VNEG_BANDGAP
  * @return None
  * @details This macro will set CN bit of ACMPCR register to determine the source of negative input.
  */
#define ACMP_SET_NEG_SRC(acmp, u32ChNum, u32Src) ((acmp)->CR[(u32ChNum)%3] = ((acmp)->CR[(u32ChNum)%3] & ~ACMP_CR_CN_Msk) | (u32Src))

/**
  * @brief This macro is used to enable hysteresis function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMP_HYS_EN bit of ACMPCR register to enable hysteresis function.
  */
#define ACMP_ENABLE_HYSTERESIS(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] |= ACMP_CR_ACMP_HYS_EN_Msk)

/**
  * @brief This macro is used to disable hysteresis function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMP_HYS_EN bit of ACMPCR register to disable hysteresis function.
  */
#define ACMP_DISABLE_HYSTERESIS(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] &= ~ACMP_CR_ACMP_HYS_EN_Msk)

/**
  * @brief This macro is used to enable interrupt
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMPIE bit of ACMPCR register to enable interrupt function.
  */
#define ACMP_ENABLE_INT(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] |= ACMP_CR_ACMPIE_Msk)

/**
  * @brief This macro is used to disable interrupt
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMPIE bit of ACMPCR register to disable interrupt function.
  */
#define ACMP_DISABLE_INT(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] &= ~ACMP_CR_ACMPIE_Msk)


/**
  * @brief This macro is used to enable ACMP
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMP_EN bit of ACMPCR register to enable analog comparator.
  */
#define ACMP_ENABLE(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] |= ACMP_CR_ACMP_EN_Msk)

/**
  * @brief This macro is used to disable ACMP
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMP_EN bit of ACMPCR register to disable analog comparator.
  */
#define ACMP_DISABLE(acmp, u32ChNum) ((acmp)->CR[(u32ChNum)%3] &= ~ACMP_CR_ACMP_EN_Msk)

/**
  * @brief This macro is used to get ACMP output value
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return  ACMP output value
  * @details This macro will return the ACMP output value.
  */
#define ACMP_GET_OUTPUT(acmp, u32ChNum) (((acmp)->SR & (ACMP_SR_CO0_Msk<<(u32ChNum)))?1:0)

/**
  * @brief This macro is used to get ACMP interrupt flag
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return   ACMP interrupt occurred or not
  * @details This macro will return the ACMP interrupt flag.
  */
#define ACMP_GET_INT_FLAG(acmp, u32ChNum) (((acmp)->SR & (ACMP_SR_ACMPF0_Msk<<(u32ChNum)))?1:0)

/**
  * @brief This macro is used to clear ACMP interrupt flag
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return   None
  * @details This macro will write 1 to ACMPFn bit of ACMPSR register to clear interrupt flag.
  */
#define ACMP_CLR_INT_FLAG(acmp, u32ChNum) ((acmp)->SR = (ACMP_SR_ACMPF0_Msk<<(u32ChNum)))

/**
  * @brief This macro is used to select ACMP positive input source
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Src Comparator positive input selection. Including:
  *                  - \ref ACMP_CR_VPOS_PIN
  *                  - \ref ACMP_CR_VPOS_OPA
  * @return None
  * @details This macro will set CP (ACMPxCR[3]) to determine the comparator positive input source.
  */
#define ACMP_SELECT_P(acmp, u32ChNum, u32Src) ((acmp)->CR[(u32ChNum)%3] = ((acmp)->CR[(u32ChNum)%3] & ~ACMP_CR_CP_Msk) | (u32Src))


/* Function prototype declaration */
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);

/*@}*/ /* end of group ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ACMP_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ACMP_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
