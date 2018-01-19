/**************************************************************************//**
 * @file     hdiv.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/04/07 2:47p $ 
 * @brief    NM1200_NM1100 HDIV driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#ifndef __HDIV_H__
#define __HDIV_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1200_NM1100_Device_Driver NM1200_NM1100 Device Driver
  @{
*/

/** @addtogroup NM1200_NM1100_HDIV_Driver HDIV Driver
  @{
*/

    
/** @addtogroup NM1200_NM1100_HDIV_EXPORTED_FUNCTIONS HDIV Exported Functions
  @{
*/

/**
 * @brief      Division function to calculate (x/y)
 *
 * @param[in]  x the dividend of the division
 * @param[in]  y the divisor of the division
 *
 * @return     The result of (x/y)
 *
 * @details    This is a division function to calculate x/y
 *
 */
static __INLINE int32_t HDIV_Div(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)HDIV_BASE;
    *p32++ = x;
    *p32++ = y;
    return *p32;
}


/**
 * @brief      To calculate the remainder of x/y, i.e., the result of x mod y.
 *
 * @param[in]  x the dividend of the division
 * @param[in]  y the divisor of the division
 *
 * @return     The remainder of (x/y)
 *
 * @details    This function is used to calculate the remainder of x/y.
 */
static __INLINE int16_t HDIV_Mod(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)HDIV_BASE;
    *p32++ = x;
    *p32++ = y;
    return p32[1];
}


/*@}*/ /* end of group NM1200_NM1100_HDIV_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1200_NM1100_HDIV_Driver */

/*@}*/ /* end of group NM1200_NM1100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__HDIV_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
