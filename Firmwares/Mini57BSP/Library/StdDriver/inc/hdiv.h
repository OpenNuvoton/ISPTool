/**************************************************************************//**
 * @file     hdiv.h
 * @version  V2.1
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series series HDIV driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HDIV_H__
#define __HDIV_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_HDIV_Driver HDIV Driver
  @{
*/

/** @addtogroup Mini57_HDIV_EXPORTED_FUNCTIONS HDIV Exported Functions
  @{
*/

/**
 * @brief       Division function to calculate (x/y)
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The result of (x/y)
 * @details     This is a division function to calculate x/y
 */
static __INLINE int32_t HDIV_Div(int32_t x, int16_t y)
{
    HDIV->DIVIDEND = x;
    HDIV->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV->QUOTIENT;
}


/**
 * @brief       To calculate the remainder of x/y, i.e., the result of x mod y.
 * @param[in]   x the dividend of the division
 * @param[in]   y the divisor of the division
 * @return      The remainder of (x/y)
 * @details     This function is used to calculate the remainder of x/y.
 */
static __INLINE int16_t HDIV_Mod(int32_t x, int16_t y)
{
    HDIV->DIVIDEND = x;
    HDIV->DIVISOR = y;      /* HDIV begin to calculate here. */
    return HDIV->REM;
}


/**
  * @brief      Get previous HDIV status for divide by zero
  * @param      None
  * @retval     0   Previous HDIV is not divide by zero
  * @retval     1   Previous HDIV is divide by zero
  * @details    This macro get previous HDIV status for divide by zero
  */
#define HDIV_IS_DIVBYZERO()     ((HDIV->STATUS & HDIV_STATUS_DIVBYZERO_Msk) >> HDIV_STATUS_DIVBYZERO_Pos)


/*@}*/ /* end of group Mini57_HDIV_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_HDIV_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __HDIV_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

