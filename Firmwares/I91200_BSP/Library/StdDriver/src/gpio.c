/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/01 2:52p $
 * @brief    I91200 GPIO driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup I91200_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 * @param[in]   u32Mode     Operation mode. GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_OPEN_DRAIN, GPIO_MODE_QUASI
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
            gpio->MODE = (gpio->MODE & ~(0x3 << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   gpio            GPIO port. It could be PA, PB.
 * @param[in]   u32Pin          The pin of specified GPIO port. It could be 0 ~ 15.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              GPIO_INT_RISING, GPIO_INT_FALLING, GPIO_INT_BOTH_EDGE, GPIO_INT_HIGH, GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    gpio->INTTYPE |= (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);
    gpio->INTEN |= ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);
}

/**
 * @brief       Disable GPIO interrupt
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 15.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin)
{
    gpio->INTTYPE &= ~(1UL << u32Pin);
    gpio->INTEN &= ~((0x00010001UL) << u32Pin);
}

/*@}*/ /* end of group I91200_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_GPIO_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
