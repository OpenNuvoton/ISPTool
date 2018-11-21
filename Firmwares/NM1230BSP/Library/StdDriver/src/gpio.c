/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 2018/10/05 9:20 $
 * @brief    NM1230 GPIO driver source file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NM1230.h"

static uint32_t volatile * const s_pu32INTEN[6]   = {&PA->INTEN,   &PB->INTEN,   &PC->INTEN,   &PD->INTEN,   &PE->INTEN,    &PF->INTEN};
static uint32_t volatile * const s_pu32INTTYPE[6] = {&PA->INTTYPE, &PB->INTTYPE, &PC->INTTYPE, &PD->INTTYPE, &PE->INTTYPE,  &PF->INTTYPE};

/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_GPIO_Driver GPIO Driver
  @{
*/


/** @addtogroup NM1230_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, or PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT7 for PA.
 *                          It could be BIT0 ~ BIT7 for PB.
 *                          It could be BIT0 ~ BIT7 for PC.
 *                          It could be BIT1 ~ BIT7 for PD.
 *                          It could be BIT0 ~ BIT7 for PE.
 *                          It could be BIT0 ~ BIT4 for PF.
 * @param[in]   u32Mode     Operation mode. It could be
 *                          - \ref GPIO_MODE_INPUT
 *                          - \ref GPIO_MODE_OUTPUT
 *                          - \ref GPIO_MODE_OPEN_DRAIN
 *                          - \ref GPIO_MODE_QUASI
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;
    uint32_t field_len = 2;

    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
            port->MODE = (port->MODE & ~(0x3 << (i*field_len))) | (u32Mode << (i*field_len));
        }
    }
}


/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   port            GPIO port. It could be PA, PB, PC, or PD.
 * @param[in]   u32Pin          The pin of specified GPIO port.
 *                              It could be 0 ~ 7 for PA.
 *                              It could be 0 ~ 7 for PB.
 *                              It could be 0 ~ 7 for PC.
 *                              It could be 1 ~ 7 for PD.
 *                              It could be 0 ~ 7 for PE.
 *                              It could be 0 ~ 4 for PF.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING
 *                              - \ref GPIO_INT_FALLING
 *                              - \ref GPIO_INT_BOTH_EDGE
 *                              - \ref GPIO_INT_HIGH
 *                              - \ref GPIO_INT_LOW
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    uint32_t u32Value;
    uint32_t u32Idx;
    uint32_t u32Type;

    u32Idx = GPIO_GET_OFFSET((uint32_t)port);
    u32Value = *s_pu32INTEN[u32Idx];
    *s_pu32INTEN[u32Idx] = (u32Value & ~(0x00010001UL << u32Pin)) | (u32IntAttribs << u32Pin);

    u32Value = *s_pu32INTTYPE[u32Idx];
    if (u32IntAttribs & 0x01000000)
        u32Type = GPIO_INTTYPE_LEVEL;
    else
        u32Type = GPIO_INTTYPE_EDGE;
    *s_pu32INTTYPE[u32Idx] = (u32Value & ~(0x00000001UL << u32Pin)) | (u32Type << u32Pin);
}


/**
 * @brief       Disable GPIO interrupt
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, or PD.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                              It could be 0 ~ 7 for PA.
 *                              It could be 0 ~ 7 for PB.
 *                              It could be 0 ~ 7 for PC.
 *                              It could be 1 ~ 7 for PD.
 *                              It could be 0 ~ 7 for PE.
 *                              It could be 0 ~ 4 for PF.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin)
{
    uint32_t u32Idx;

    u32Idx = GPIO_GET_OFFSET((uint32_t)port);

    *s_pu32INTEN[u32Idx] &= ~((0x00010001UL) << u32Pin);
}

/*@}*/ /* end of group NM1230_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_GPIO_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
