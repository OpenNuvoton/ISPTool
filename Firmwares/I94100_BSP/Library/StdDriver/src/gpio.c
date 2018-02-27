/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/08 3:13p $
 * @brief    I94100 series GPIO driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "I94100.h"

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup I94100_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 * @param[in]   port        GPIO port. It could be PA, PB, PC or PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT15 for PA, PC and PD.
 *                          It could be BIT0 ~ BIT9, BIT12, BIT13, BIT14 for PB.
 * @param[in]   u32Mode     Operation mode.  It could be \n
 *                          GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_OPEN_DRAIN, GPIO_MODE_QUASI.
 * @return      None
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1 << i))
        {
            port->MODE = (port->MODE & ~(0x3 << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
 * @brief       Enable GPIO interrupt
 * @param[in]   port            GPIO port. It could be PA, PB, PC or PD.
 * @param[in]   u32Pin          The pin of specified GPIO port.
 *                              It could be 0 ~ 15 for PA, PC and PD.
 *                              It could be 0 ~ 9, 12, 13, 14 for PB.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              GPIO_INT_RISING, GPIO_INT_FALLING, GPIO_INT_BOTH_EDGE, GPIO_INT_HIGH, GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    port->INTTYPE |= (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);
    port->INTEN |= ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);
}

/**
 * @brief       Disable GPIO interrupt
 * @param[in]   port            GPIO port. It could be PA, PB, PC or PD.
 * @param[in]   u32Pin          The pin of specified GPIO port.
 *                              It could be 0 ~ 15 for PA, PC and PD.
 *                              It could be 0 ~ 9, 12, 13, 14 for PB.
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin)
{
    port->INTTYPE &= ~(1UL << u32Pin);
    port->INTEN &= ~((0x00010001UL) << u32Pin);
}


/*@}*/ /* end of group I94100_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_GPIO_Driver */

/*@}*/ /* end of group I94100_Device_Driver  */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
