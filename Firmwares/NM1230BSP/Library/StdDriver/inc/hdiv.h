/**************************************************************************//**
 * @file     hdiv.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 2018/07/16 14:16 $
 * @brief    NM1230 series HDIV driver header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HDIV_H__
#define __HDIV_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_HDIV_Driver HDIV Driver
  @{
*/

/** @addtogroup NM1230_HDIV_EXPORTED_FUNCTIONS HDIV Exported Functions
  @{
*/


/**
 * @brief       Division function to calculate (x/y) whit HDIV 0
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The result of (x/y)
 * @details     This is a division function to calculate x/y
 */
static __INLINE int32_t HDIV0_Div(int32_t x, int16_t y)
{
    HDIV0->DIVIDEND = x;
    HDIV0->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV0->QUOTIENT;
}


/**
 * @brief       To calculate the remainder of x/y whit HDIV 0, i.e., the result of x mod y.
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The remainder of (x/y)
 * @details     This function is used to calculate the remainder of x/y.
 */
static __INLINE int16_t HDIV0_Mod(int32_t x, int16_t y)
{
    HDIV0->DIVIDEND = x;
    HDIV0->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV0->REM;
}


/**
 * @brief       Division function to calculate (x/y) whit HDIV 1
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The result of (x/y)
 * @details     This is a division function to calculate x/y
 */
static __INLINE int32_t HDIV1_Div(int32_t x, int16_t y)
{
    HDIV1->DIVIDEND = x;
    HDIV1->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV1->QUOTIENT;
}


/**
 * @brief       To calculate the remainder of x/y whit HDIV 1, i.e., the result of x mod y.
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The remainder of (x/y)
 * @details     This function is used to calculate the remainder of x/y.
 */
static __INLINE int16_t HDIV1_Mod(int32_t x, int16_t y)
{
    HDIV1->DIVIDEND = x;
    HDIV1->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV1->REM;
}


/**
 * @brief       Division function to calculate (x/y) whit HDIV 2
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The result of (x/y)
 * @details     This is a division function to calculate x/y
 */
static __INLINE int32_t HDIV2_Div(int32_t x, int16_t y)
{
    HDIV2->DIVIDEND = x;
    HDIV2->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV2->QUOTIENT;
}


/**
 * @brief       To calculate the remainder of x/y whit HDIV 2, i.e., the result of x mod y.
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The remainder of (x/y)
 * @details     This function is used to calculate the remainder of x/y.
 */
static __INLINE int16_t HDIV2_Mod(int32_t x, int16_t y)
{
    HDIV2->DIVIDEND = x;
    HDIV2->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV2->REM;
}


/**
  * @brief      Get previous HDIV status for divide by zero
  * @param      None
  * @retval     0   Previous HDIV is not divide by zero
  * @retval     1   Previous HDIV is divide by zero
  * @details    This macro get previous HDIV status for divide by zero
  */
#define HDIV_IS_DIVBYZERO()     ((HDIV_STS->STATUS & HDIV_STATUS_DIVBYZERO_Msk) >> HDIV_STATUS_DIVBYZERO_Pos)


/*@}*/ /* end of group NM1230_HDIV_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_HDIV_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __HDIV_H__ */


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
