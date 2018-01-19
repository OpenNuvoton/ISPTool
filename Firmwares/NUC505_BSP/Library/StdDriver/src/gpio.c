/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 14/05/29 1:13p $
 * @brief    NUC505 GPIO driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NUC505Series.h"



/** @addtogroup NUC505_Device_Driver NUC505 Device Driver
  @{
*/

/** @addtogroup NUC505_GPIO_Driver GPIO Driver
  @{
*/


/** @addtogroup NUC505_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, ... or PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 * @param[in]   u32Mode     Operation mode. It could be \n
 *          - \ref GPIO_MODE_INPUT
 *          - \ref GPIO_MODE_OUTPUT
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
            gpio->MODE = (gpio->MODE & ~(0x1 << i)) | (u32Mode << i);
        }
    }
}

/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   gpio            GPIO port. It could be PA, PB, ... or PD.
 * @param[in]   u32Pin          The pin of specified GPIO port. It could be 0 ~ 15.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *          - \ref GPIO_INT_RISING
 *          - \ref GPIO_INT_FALLING
 *          - \ref GPIO_INT_BOTH_EDGE
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    uint32_t u32Value;

    GPIO->INTCTL |= GPIO_INTCTL_INTCTL_Msk;
    u32Value = *(__IO uint32_t *) (&PA->GROUP_T.PA_T.INTEN + GPIO_GET_OFFSET((uint32_t)gpio));
    u32Value = (u32Value & ~(0x00010001 << u32Pin)) | (u32IntAttribs << u32Pin);
    *(__IO uint32_t *) (&PA->GROUP_T.PA_T.INTEN + GPIO_GET_OFFSET((uint32_t)gpio)) = u32Value;
}

/**
 * @brief       Disable GPIO interrupt
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, ... or PD.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 15.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin)
{
    uint32_t u32Value;

    u32Value = *(__IO uint32_t *) (&PA->GROUP_T.PA_T.INTEN + GPIO_GET_OFFSET((uint32_t)gpio));
    u32Value &= ~((0x00010001UL) << u32Pin);
    *(__IO uint32_t *) (&PA->GROUP_T.PA_T.INTEN + GPIO_GET_OFFSET((uint32_t)gpio)) = u32Value;
}

/**
 * @brief       Set GPIO interrupt group
 *
 * @param[in]   gpio            GPIO port. It could be PA, PB, ... or PD.
 * @param[in]   u32Pin          The pin of specified GPIO port. It could be 0 ~ 15.
 * @param[in]   u32IntGroup   The interrupt group of specified GPIO pin. It could be \n
 *          - \ref GPIO_INTSRCGP_EINT0
 *          - \ref GPIO_INTSRCGP_EINT1
 *          - \ref GPIO_INTSRCGP_EINT2
 *          - \ref GPIO_INTSRCGP_EINT3
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO pin interrupt group.
 */
void GPIO_SetIntGroup(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntGroup)
{
    uint32_t u32Value;

    u32Value = *(__IO uint32_t *) (&PA->GROUP_T.PA_T.INTSRCGP + GPIO_GET_OFFSET((uint32_t)gpio));
    u32Value = (u32Value & ~(0x03 << (u32Pin<<1))) | (u32IntGroup << (u32Pin<<1));
    *(__IO uint32_t *) (&PA->GROUP_T.PA_T.INTSRCGP + GPIO_GET_OFFSET((uint32_t)gpio)) = u32Value;
}

/**
 * @brief       Set GPIO pull mode
 *
 * @param[in]   gpio            GPIO port. It could be PA, PB, ... or PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 * @param[in]   u32Mode   The pull mode of specified GPIO pin. It could be \n
 *          - \ref GPIO_PULL_DISABLE
 *          - \ref GPIO_PULL_UP_EN
 *          - \ref GPIO_PULL_DOWN_EN
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO pin pull mode.
 */
void GPIO_SetPullMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
            gpio->PUEN = (gpio->PUEN & ~(0x03 << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}



/*@}*/ /* end of group NUC505_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC505_GPIO_Driver */

/*@}*/ /* end of group NUC505_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
