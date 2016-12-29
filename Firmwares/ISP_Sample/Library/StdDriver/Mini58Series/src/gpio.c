/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/02/02 3:19p $
 * @brief    Mini58 series GPIO driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini58Series.h"



/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_GPIO_Driver GPIO Driver
  @{
*/


/** @addtogroup Mini58_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 * @param[in]   u32Mode     Operation mode.
 *                          - \ref GPIO_MODE_INPUT,
 *                          - \ref GPIO_MODE_OUTPUT,
 *                          - \ref GPIO_MODE_OPEN_DRAIN,
 *                          - \ref GPIO_MODE_QUASI
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
 * @param[in]   gpio            GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin          The pin of specified GPIO port. It could be 0 ~ 7.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING,
 *                              - \ref GPIO_INT_FALLING,
 *                              - \ref GPIO_INT_BOTH_EDGE,
 *                              - \ref GPIO_INT_HIGH,
 *                              - \ref GPIO_INT_LOW.
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
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin)
{
    gpio->INTTYPE &= ~(1UL << u32Pin);
    gpio->INTEN &= ~((0x00010001UL) << u32Pin);
}



/*@}*/ /* end of group Mini58_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_GPIO_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
