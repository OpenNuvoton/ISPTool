/**************************************************************************//**
 * @file     gpio.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/12/27 2:52p $
 * @brief    I91200 GPIO driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup I91200_GPIO_EXPORTED_CONSTANTS GPIO Exported Constants
  @{
*/
	
#define GPIO_PIN_MAX    16   /*!< Specify Maximum Pins of Each GPIO Port */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODE Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_MODE_INPUT              0x0UL           /*!< Input Mode */
#define GPIO_MODE_OUTPUT             0x1UL           /*!< Output Mode */
#define GPIO_MODE_OPEN_DRAIN         0x2UL           /*!< Open-Drain Mode */
#define GPIO_MODE_QUASI              0x3UL           /*!< Quasi-bidirectional Mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INT_RISING             0x00010000UL    /*!< Interrupt enable by Input Rising Edge */
#define GPIO_INT_FALLING            0x00000001UL    /*!< Interrupt enable by Input Falling Edge */
#define GPIO_INT_BOTH_EDGE          0x00010001UL    /*!< Interrupt enable by both Rising Edge and Falling Edge */
#define GPIO_INT_HIGH               0x01010000UL    /*!< Interrupt enable by Level-High */
#define GPIO_INT_LOW                0x01000001UL    /*!< Interrupt enable by Level-Level */

/*---------------------------------------------------------------------------------------------------------*/
/*  INTTYPE Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INTTYPE_EDGE               0UL             /*!< INTTYPE Setting for Edge Trigger Mode */
#define GPIO_INTTYPE_LEVEL              1UL             /*!< INTTYPE Setting for Edge Level Mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  DBCTL Constant Definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_DBCTL_ICLK_ON           0x00000020UL /*!< DBCTL setting for all IO pins edge detection circuit is always active after reset */
#define GPIO_DBCTL_ICLK_OFF          0x00000000UL /*!< DBCTL setting for edge detection circuit is active only if IO pin corresponding GPIOx_IEN bit is set to 1 */

#define GPIO_DBCTL_DBCLKSRC_IRC10K   0x00000010UL /*!< DBCTL setting for de-bounce counter clock source is the internal 10 kHz */
#define GPIO_DBCTL_DBCLKSRC_HCLK     0x00000000UL /*!< DBCTL setting for de-bounce counter clock source is the internal HCLK */

#define GPIO_DBCTL_DBCLKSEL_1        0x00000000UL /*!< DBCTL setting for sampling cycle = 1 clocks */
#define GPIO_DBCTL_DBCLKSEL_2        0x00000001UL /*!< DBCTL setting for sampling cycle = 2 clocks */
#define GPIO_DBCTL_DBCLKSEL_4        0x00000002UL /*!< DBCTL setting for sampling cycle = 4 clocks */
#define GPIO_DBCTL_DBCLKSEL_8        0x00000003UL /*!< DBCTL setting for sampling cycle = 8 clocks */
#define GPIO_DBCTL_DBCLKSEL_16       0x00000004UL /*!< DBCTL setting for sampling cycle = 16 clocks */
#define GPIO_DBCTL_DBCLKSEL_32       0x00000005UL /*!< DBCTL setting for sampling cycle = 32 clocks */
#define GPIO_DBCTL_DBCLKSEL_64       0x00000006UL /*!< DBCTL setting for sampling cycle = 64 clocks */
#define GPIO_DBCTL_DBCLKSEL_128      0x00000007UL /*!< DBCTL setting for sampling cycle = 128 clocks */
#define GPIO_DBCTL_DBCLKSEL_256      0x00000008UL /*!< DBCTL setting for sampling cycle = 256 clocks */
#define GPIO_DBCTL_DBCLKSEL_512      0x00000009UL /*!< DBCTL setting for sampling cycle = 512 clocks */
#define GPIO_DBCTL_DBCLKSEL_1024     0x0000000AUL /*!< DBCTL setting for sampling cycle = 1024 clocks */
#define GPIO_DBCTL_DBCLKSEL_2048     0x0000000BUL /*!< DBCTL setting for sampling cycle = 2048 clocks */
#define GPIO_DBCTL_DBCLKSEL_4096     0x0000000CUL /*!< DBCTL setting for sampling cycle = 4096 clocks */
#define GPIO_DBCTL_DBCLKSEL_8192     0x0000000DUL /*!< DBCTL setting for sampling cycle = 8192 clocks */
#define GPIO_DBCTL_DBCLKSEL_16384    0x0000000EUL /*!< DBCTL setting for sampling cycle = 16384 clocks */
#define GPIO_DBCTL_DBCLKSEL_32768    0x0000000FUL /*!< DBCTL setting for sampling cycle = 32768 clocks */

/*@}*/ /* end of group I91200_GPIO_EXPORTED_CONSTANTS */

/** @addtogroup I91200_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Clear the interrupt status of specified GPIO pin.
 */
#define GPIO_CLR_INT_FLAG(gpio, u32PinMask)   ((gpio)->INTSRC = u32PinMask)

/**
 * @brief       Disable Pin De-bounce Function
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Disable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_DISABLE_DEBOUNCE(gpio, u32PinMask)   ((gpio)->DBEN &= ~u32PinMask)

/**
 * @brief       Enable Pin De-bounce Function
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Enable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_ENABLE_DEBOUNCE(gpio, u32PinMask)    ((gpio)->DBEN |= u32PinMask)

/**
 * @brief       Disable I/O Digital Input Path
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Disable I/O digital input path of specified GPIO pin.
 */
#define GPIO_DISABLE_DIGITAL_PATH(gpio, u32PinMask)   ((gpio)->DINOFF |= (u32PinMask << 16))

/**
 * @brief       Enable I/O Digital Input Path
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Enable I/O digital input path of specified GPIO pin.
 */
#define GPIO_ENABLE_DIGITAL_PATH(gpio, u32PinMask)    ((gpio)->DINOFF &= ~(u32PinMask << 16))

/**
 * @brief       Disable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Disable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_DISABLE_DOUT_MASK(gpio, u32PinMask)   ((gpio)->DATMSK |= u32PinMask)

/**
 * @brief       Enable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Enable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_ENABLE_DOUT_MASK(gpio, u32PinMask)   ((gpio)->DATMSK &= ~u32PinMask)

/**
 * @brief       Get GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @retval      0           No interrupt at specified GPIO pin
 * @retval      1           The specified GPIO pin generate an interrupt
 *
 * @details     Get the interrupt status of specified GPIO pin.
 */
#define GPIO_GET_INT_FLAG(gpio, u32PinMask)   ((gpio)->INTSRC & u32PinMask)

/**
 * @brief       Set De-bounce Sampling Cycle Time
 *
 * @param[in]   u32ClkSrc   The de-bounce counter clock source. It could be GPIO_DBCTL_DBCLKSRC_HCLK or GPIO_DBCTL_DBCLKSRC_IRC10K.
 * @param[in]   u32ClkSel   The de-bounce sampling cycle selection. It could be \n
 *                              GPIO_DBCTL_DBCLKSEL_1, GPIO_DBCTL_DBCLKSEL_2, GPIO_DBCTL_DBCLKSEL_4, GPIO_DBCTL_DBCLKSEL_8, \n
 *                              GPIO_DBCTL_DBCLKSEL_16, GPIO_DBCTL_DBCLKSEL_32, GPIO_DBCTL_DBCLKSEL_64, GPIO_DBCTL_DBCLKSEL_128, \n
 *                              GPIO_DBCTL_DBCLKSEL_256, GPIO_DBCTL_DBCLKSEL_512, GPIO_DBCTL_DBCLKSEL_1024, GPIO_DBCTL_DBCLKSEL_2048, \n
 *                              GPIO_DBCTL_DBCLKSEL_4096, GPIO_DBCTL_DBCLKSEL_8192, GPIO_DBCTL_DBCLKSEL_16384, GPIO_DBCTL_DBCLKSEL_32768.
 *
 * @return      None
 *
 * @details     Set the interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: _GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_IRC10K, GPIO_DBCTL_DBCLKSEL_4). \n
 *              It's meaning the De-debounce counter clock source is internal 10 KHz and sampling cycle selection is 4. \n
 *              Then the target de-bounce sampling cycle time is (2^4)*(1/(10*1000)) s = 16*0.0001 s = 1600 us,
 *              and system will sampling interrupt input once per 1600 us.
 */
#define GPIO_SET_DEBOUNCE_TIME(u32ClkSrc, u32ClkSel)  (GPIO->DBCTL = (GPIO_DBCTL_ICLKON_Msk | u32ClkSrc | u32ClkSel))

/**
 * @brief       Get GPIO Port IN Data
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 *
 * @retval      The specified port data
 *
 * @details     Get the PIN register of specified GPIO port.
 */
#define GPIO_GET_IN_DATA(gpio)   ((gpio)->PIN)

/**
 * @brief       Set GPIO Port OUT Data
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 * @param[in]   u32Data     GPIO port data.
 *
 * @retval      None
 *
 * @details     Set the Data into specified GPIO port.
 */
#define GPIO_SET_OUT_DATA(gpio, u32Data)   ((gpio)->DOUT = (u32Data))

/**
 * @brief       Get GPIO Port OUT Data
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB.
 *
 * @retval      None
 *
 * @details     Get the Data into specified GPIO port.
 */
#define GPIO_GET_OUT_DATA(gpio)   ((gpio)->DOUT)

/**
 * @brief       Toggle Specified GPIO pin
 *
 * @param[in]   u32Pin       Pxy
 *
 * @retval      None
 *
 * @details     Toggle the specified GPIO pint.
 */
#define GPIO_TOGGLE(u32Pin)   ((u32Pin) ^= 1)


void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);

/*@}*/ /* end of group I91200_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_GPIO_Driver */

/*@}*/ /* end of group I91200_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif //__GPIO_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
