/**************************************************************************//**
 * @file     gpio.h
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 17/05/19 11:48a $
 * @brief    Mini57 Series series GPIO driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup Mini57_GPIO_EXPORTED_CONSTANTS GPIO Exported Constants
  @{
*/
#define GPIO_PIN_MAX            7       /*!< Specify Maximum Pins of Each GPIO Port */

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_MODE Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_MODE_INPUT         0x0UL   /*!< Input Mode */
#define GPIO_MODE_OUTPUT        0x1UL   /*!< Output Mode */
#define GPIO_MODE_OPEN_DRAIN    0x2UL   /*!< Open-Drain Mode */
#define GPIO_MODE_QUASI         0x3UL   /*!< Quasi-bidirectional Mode */

#define GPIO_PMD_INPUT          0x0UL   /*!< Input Mode */
#define GPIO_PMD_OUTPUT         0x1UL   /*!< Output Mode */
#define GPIO_PMD_OPEN_DRAIN     0x2UL   /*!< Open-Drain Mode */
#define GPIO_PMD_QUASI          0x3UL   /*!< Quasi-bidirectional Mode */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INT_RISING         0x00010000UL /*!< Interrupt enable by Input Rising Edge */
#define GPIO_INT_FALLING        0x00000001UL /*!< Interrupt enable by Input Falling Edge */
#define GPIO_INT_BOTH_EDGE      0x00010001UL /*!< Interrupt enable by both Rising Edge and Falling Edge */
#define GPIO_INT_HIGH           0x01010000UL /*!< Interrupt enable by Level-High */
#define GPIO_INT_LOW            0x01000001UL /*!< Interrupt enable by Level-Low */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_INTTYPE Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INTTYPE_EDGE       0UL     /*!< GPIO_INTTYPE Setting for Edge Trigger Mode */
#define GPIO_INTTYPE_LEVEL      1UL     /*!< GPIO_INTTYPE Setting for Level Trigger Mode */

#define GPIO_IMD_EDGE           0UL     /*!< GPIO_INTTYPE Setting for Edge Trigger Mode */
#define GPIO_IMD_LEVEL          1UL     /*!< GPIO_INTTYPE Setting for Level Trigger Mode */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_DBCTL Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_DBCTL_ICLK_ON            0x00000020UL /*!< GPIO_DBCTL setting for all IO pins edge detection circuit is always active after reset */
#define GPIO_DBCTL_ICLK_OFF           0x00000000UL /*!< GPIO_DBCTL setting for edge detection circuit is active only if IO pin corresponding GPIOx_IEN bit is set to 1 */

#define GPIO_DBCTL_DBCLKSRC_LIRC      0x00000010UL /*!< GPIO_DBCTL setting for de-bounce counter clock source is the internal 10 kHz */
#define GPIO_DBCTL_DBCLKSRC_HCLK      0x00000000UL /*!< GPIO_DBCTL setting for de-bounce counter clock source is the HCLK */

#define GPIO_DBCTL_DBCLKSEL_1         0x00000000UL /*!< GPIO_DBCTL setting for sampling cycle = 1 clocks */
#define GPIO_DBCTL_DBCLKSEL_2         0x00000001UL /*!< GPIO_DBCTL setting for sampling cycle = 2 clocks */
#define GPIO_DBCTL_DBCLKSEL_4         0x00000002UL /*!< GPIO_DBCTL setting for sampling cycle = 4 clocks */
#define GPIO_DBCTL_DBCLKSEL_8         0x00000003UL /*!< GPIO_DBCTL setting for sampling cycle = 8 clocks */
#define GPIO_DBCTL_DBCLKSEL_16        0x00000004UL /*!< GPIO_DBCTL setting for sampling cycle = 16 clocks */
#define GPIO_DBCTL_DBCLKSEL_32        0x00000005UL /*!< GPIO_DBCTL setting for sampling cycle = 32 clocks */
#define GPIO_DBCTL_DBCLKSEL_64        0x00000006UL /*!< GPIO_DBCTL setting for sampling cycle = 64 clocks */
#define GPIO_DBCTL_DBCLKSEL_128       0x00000007UL /*!< GPIO_DBCTL setting for sampling cycle = 128 clocks */
#define GPIO_DBCTL_DBCLKSEL_256       0x00000008UL /*!< GPIO_DBCTL setting for sampling cycle = 256 clocks */
#define GPIO_DBCTL_DBCLKSEL_512       0x00000009UL /*!< GPIO_DBCTL setting for sampling cycle = 512 clocks */
#define GPIO_DBCTL_DBCLKSEL_1024      0x0000000AUL /*!< GPIO_DBCTL setting for sampling cycle = 1024 clocks */
#define GPIO_DBCTL_DBCLKSEL_2048      0x0000000BUL /*!< GPIO_DBCTL setting for sampling cycle = 2048 clocks */
#define GPIO_DBCTL_DBCLKSEL_4096      0x0000000CUL /*!< GPIO_DBCTL setting for sampling cycle = 4096 clocks */
#define GPIO_DBCTL_DBCLKSEL_8192      0x0000000DUL /*!< GPIO_DBCTL setting for sampling cycle = 8192 clocks */
#define GPIO_DBCTL_DBCLKSEL_16384     0x0000000EUL /*!< GPIO_DBCTL setting for sampling cycle = 16384 clocks */
#define GPIO_DBCTL_DBCLKSEL_32768     0x0000000FUL /*!< GPIO_DBCTL setting for sampling cycle = 32768 clocks */


/* Define GPIO Pin Data Input/Output. It could be used to control each I/O pin by pin address mapping.
   Example 1:

       PA0 = 1;

   It is used to set GPIO PA.0 to high;

   Example 2:

       if (PA0)
           PA0 = 0;

   If GPIO PA.0 pin status is high, then set GPIO PA.0 data output to low.
 */
#define GPIO_PIN_DATA(port, pin)    (*((volatile uint32_t *)((GPIO_PIN_DATA_BASE+(0x40*(port))) + ((pin)<<2))))
#define PA0             GPIO_PIN_DATA(0, 0 ) /*!< Specify PA.0 Pin Data Input/Output */
#define PA1             GPIO_PIN_DATA(0, 1 ) /*!< Specify PA.1 Pin Data Input/Output */
#define PA2             GPIO_PIN_DATA(0, 2 ) /*!< Specify PA.2 Pin Data Input/Output */
#define PA3             GPIO_PIN_DATA(0, 3 ) /*!< Specify PA.3 Pin Data Input/Output */
#define PA4             GPIO_PIN_DATA(0, 4 ) /*!< Specify PA.4 Pin Data Input/Output */
#define PA5             GPIO_PIN_DATA(0, 5 ) /*!< Specify PA.5 Pin Data Input/Output */

#define PB0             GPIO_PIN_DATA(1, 0 ) /*!< Specify PB.0 Pin Data Input/Output */
#define PB1             GPIO_PIN_DATA(1, 1 ) /*!< Specify PB.1 Pin Data Input/Output */
#define PB2             GPIO_PIN_DATA(1, 2 ) /*!< Specify PB.2 Pin Data Input/Output */
#define PB3             GPIO_PIN_DATA(1, 3 ) /*!< Specify PB.3 Pin Data Input/Output */
#define PB4             GPIO_PIN_DATA(1, 4 ) /*!< Specify PB.4 Pin Data Input/Output */

#define PC0             GPIO_PIN_DATA(2, 0 ) /*!< Specify PC.0 Pin Data Input/Output */
#define PC1             GPIO_PIN_DATA(2, 1 ) /*!< Specify PC.1 Pin Data Input/Output */
#define PC2             GPIO_PIN_DATA(2, 2 ) /*!< Specify PC.2 Pin Data Input/Output */
#define PC3             GPIO_PIN_DATA(2, 3 ) /*!< Specify PC.3 Pin Data Input/Output */
#define PC4             GPIO_PIN_DATA(2, 4 ) /*!< Specify PC.4 Pin Data Input/Output */

#define PD0             GPIO_PIN_DATA(3, 0 ) /*!< Specify PD.0 Pin Data Input/Output */
#define PD1             GPIO_PIN_DATA(3, 1 ) /*!< Specify PD.1 Pin Data Input/Output */
#define PD2             GPIO_PIN_DATA(3, 2 ) /*!< Specify PD.2 Pin Data Input/Output */
#define PD3             GPIO_PIN_DATA(3, 3 ) /*!< Specify PD.3 Pin Data Input/Output */
#define PD4             GPIO_PIN_DATA(3, 4 ) /*!< Specify PD.4 Pin Data Input/Output */
#define PD5             GPIO_PIN_DATA(3, 5 ) /*!< Specify PD.5 Pin Data Input/Output */
#define PD6             GPIO_PIN_DATA(3, 6 ) /*!< Specify PD.6 Pin Data Input/Output */


/*@}*/ /* end of group Mini57_GPIO_EXPORTED_CONSTANTS */


/** @addtogroup Mini57_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Calculate GPIO port offset
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 *
 * @return      The specified GPIO port offset
 *
 * @details     Calculate GPIO port offset.
 */
#define GPIO_GET_OFFSET(port)                  ((port & 0xFF) / ((uint32_t)PB-(uint32_t)PA))

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Clear the interrupt status of specified GPIO pin.
 */
#define GPIO_CLR_INT_FLAG(port, u32PinMask)         ((port)->INTSRC = (u32PinMask))

/**
 * @brief       Disable Pin De-bounce Function
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Disable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_DISABLE_DEBOUNCE(port, u32PinMask)     ((port)->DBEN &= ~(u32PinMask))

/**
 * @brief       Enable Pin De-bounce Function
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Enable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_ENABLE_DEBOUNCE(port, u32PinMask)      ((port)->DBEN |= (u32PinMask))

/**
 * @brief       Disable I/O Digital Input Path
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Disable I/O digital input path of specified GPIO pin.
 */
#define GPIO_DISABLE_DIGITAL_PATH(port, u32PinMask) ((port)->DINOFF |= ((u32PinMask)<<GPIO_DINOFF_DINOFF0_Pos))

/**
 * @brief       Enable I/O Digital Input Path
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Enable I/O digital input path of specified GPIO pin.
 */
#define GPIO_ENABLE_DIGITAL_PATH(port, u32PinMask)  ((port)->DINOFF &= ~((u32PinMask)<<GPIO_DINOFF_DINOFF0_Pos))

/**
 * @brief       Disable I/O DOUT mask
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Disable I/O DOUT mask of specified GPIO pin. The corresponding DOUT bit protected.
 */
#define GPIO_DISABLE_DOUT_MASK(port, u32PinMask)    ((port)->DATMSK |= (u32PinMask))

/**
 * @brief       Enable I/O DOUT mask
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Enable I/O DOUT mask of specified GPIO pin. The corresponding DOUT bit can be updated.
 */
#define GPIO_ENABLE_DOUT_MASK(port, u32PinMask) ((port)->DATMSK &= ~(u32PinMask))

/**
 * @brief       Get GPIO Pin Interrupt Flag
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @retval      0           No interrupt at specified GPIO pin
 * @retval      Not 0       The specified GPIO pin generate an interrupt
 *
 * @details     Get the interrupt status of specified GPIO pin.
 */
#define GPIO_GET_INT_FLAG(port, u32PinMask)     ((port)->INTSRC & (u32PinMask))

/**
 * @brief       Set De-bounce Sampling Cycle Time
 *
 * @param[in]   u32ClkSrc   The de-bounce counter clock source. It could be \n
 *                              GPIO_DBCTL_DBCLKSRC_HCLK or GPIO_DBCTL_DBCLKSRC_LIRC.
 * @param[in]   u32ClkSel   The de-bounce sampling cycle selection. It could be \n
 *                              GPIO_DBCTL_DBCLKSEL_1, GPIO_DBCTL_DBCLKSEL_2, GPIO_DBCTL_DBCLKSEL_4, GPIO_DBCTL_DBCLKSEL_8, \n
 *                              GPIO_DBCTL_DBCLKSEL_16, GPIO_DBCTL_DBCLKSEL_32, GPIO_DBCTL_DBCLKSEL_64, GPIO_DBCTL_DBCLKSEL_128, \n
 *                              GPIO_DBCTL_DBCLKSEL_256, GPIO_DBCTL_DBCLKSEL_512, GPIO_DBCTL_DBCLKSEL_1024, GPIO_DBCTL_DBCLKSEL_2048, \n
 *                              GPIO_DBCTL_DBCLKSEL_4096, GPIO_DBCTL_DBCLKSEL_8192, GPIO_DBCTL_DBCLKSEL_16384, GPIO_DBCTL_DBCLKSEL_32768.
 *
 * @return      None
 *
 * @details     Set the interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_4). \n
 *              It's meaning the De-debounce counter clock source is internal 10 KHz and sampling cycle selection is 4. \n
 *              Then the target de-bounce sampling cycle time is (2^4)*(1/(10*1000)) s = 16*0.1 ms = 1.6 ms,
 *              and system will sampling interrupt input once per 1.6 ms.
 */
#define GPIO_SET_DEBOUNCE_TIME(u32ClkSrc, u32ClkSel)    (GPIO->GPIO_DBCTL = (GPIO_DBCTL_ICLKON_Msk | (u32ClkSrc) | (u32ClkSel)))

/**
 * @brief       Get GPIO Port IN Data
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 *
 * @return      The specified port data
 *
 * @details     Get the PIN register of specified GPIO port.
 */
#define GPIO_GET_IN_DATA(port)  ((port)->PIN)

/**
 * @brief       Set GPIO Port OUT Data
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32Data     GPIO port data.
 *
 * @return      None
 *
 * @details     Set the Data into specified GPIO port.
 */
#define GPIO_SET_OUT_DATA(port, u32Data)    ((port)->DOUT = (u32Data))

/**
 * @brief       Disable Pin Pull-up resistor Function
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Disable the Pull-up resistor function of specified GPIO pin.
 */
#define GPIO_DISABLE_PULL_UP(port, u32PinMask)   ((port)->PHEN |= u32PinMask)

/**
 * @brief       Enable Pin Pull-up resistor Function
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Enable the Pull-up resistor function of specified GPIO pin.
 */
#define GPIO_ENABLE_PULL_UP(port, u32PinMask)   ((port)->PHEN &= ~u32PinMask)


/**
 * @brief       Disable Pin Pull-low resistor Function
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Disable the Pull-low resistor function of specified GPIO pin.
 */
#define GPIO_DISABLE_PULL_LOW(port, u32PinMask)   ((port)->PLEN &= ~u32PinMask)

/**
 * @brief       Enable Pin Pull-low resistor Function
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT5 for PA.
 *                          It could be BIT0 ~ BIT4 for PB.
 *                          It could be BIT0 ~ BIT4 for PC.
 *                          It could be BIT0 ~ BIT6 for PD.
 *
 * @return      None
 *
 * @details     Enable the Pull-low resistor function of specified GPIO pin.
 */
#define GPIO_ENABLE_PULL_LOW(port, u32PinMask)   ((port)->PLEN |= u32PinMask)

/**
 * @brief       Toggle Specified GPIO pin
 *
 * @param[in]   u32Pin      Pxy
 *
 * @return      None
 *
 * @details     Toggle the specified GPIO pint.
 */
#define GPIO_TOGGLE(u32Pin) ((u32Pin) ^= 1)


/**
 * @brief       Enable External GPIO interrupt
 *
 * @param[in]   port            GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32Pin          The pin of specified GPIO port.
 *                              It could be 0 ~ 5 for PA.
 *                              It could be 0 ~ 4 for PB.
 *                              It could be 0 ~ 4 for PC.
 *                              It could be 0 ~ 6 for PD.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              GPIO_INT_RISING, GPIO_INT_FALLING, GPIO_INT_BOTH_EDGE, GPIO_INT_HIGH, GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_EnableEINT     GPIO_EnableInt

/**
 * @brief       Disable External GPIO interrupt
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, or PD.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 5 for PA.
 *                          It could be 0 ~ 4 for PB.
 *                          It could be 0 ~ 4 for PC.
 *                          It could be 0 ~ 6 for PD.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
#define GPIO_DisableEINT    GPIO_DisableInt


void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);


/*@}*/ /* end of group Mini57_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_GPIO_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif  /* __GPIO_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
