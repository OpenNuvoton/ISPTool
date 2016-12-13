/**************************************************************************//**
 * @file     gpio.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/11 4:13p $ 
 * @brief    Mini58 series GPIO driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup Mini58_GPIO_EXPORTED_CONSTANTS GPIO Exported Constants
  @{
*/
#define GPIO_PIN_MAX    8   /*!< Specify Maximum Pins of Each GPIO Port */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODE Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_MODE_INPUT              0x0UL                  /*!< Input Mode */
#define GPIO_MODE_OUTPUT             0x1UL                  /*!< Output Mode */
#define GPIO_MODE_OPEN_DRAIN         0x2UL                  /*!< Open-Drain Mode */
#define GPIO_MODE_QUASI              0x3UL                  /*!< Quasi-bidirectional Mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INT_RISING             0x00010000UL /*!< Interrupt enable by Input Rising Edge */
#define GPIO_INT_FALLING            0x00000001UL /*!< Interrupt enable by Input Falling Edge */
#define GPIO_INT_BOTH_EDGE          0x00010001UL /*!< Interrupt enable by both Rising Edge and Falling Edge */
#define GPIO_INT_HIGH               0x01010000UL /*!< Interrupt enable by Level-High */    
#define GPIO_INT_LOW                0x01000001UL /*!< Interrupt enable by Level-Level */

/*---------------------------------------------------------------------------------------------------------*/
/*  IMD Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INTTYPE_EDGE               0UL               /*!< INTTYPE Setting for Edge Trigger Mode */
#define GPIO_INTTYPE_LEVEL              1UL               /*!< INTTYPE Setting for Edge Level Mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  DBNCECON Constant Definitions                                                                          */
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

/** Define GPIO Pin Data Input/Output. It could be used to control each I/O pin by pin address mapping.
 *  Example 1:
 *  
 *      P00 = 1; 
 *  
 *  It is used to set P0.0 to high;
 *  
 *  Example 2:
 *  
 *      if (P00)
 *          P00 = 0;
 *  
 *  If P0.0 pin status is high, then set P0.0 data output to low.
 */
#define GPIO_PIN_ADDR(port, pin)    (*((volatile uint32_t *)((GPIOBIT0_BASE+(0x20*(port))) + ((pin)<<2)))) 
#define P00             GPIO_PIN_ADDR(0, 0) /*!< Specify P00 Pin Data Input/Output */
#define P01             GPIO_PIN_ADDR(0, 1) /*!< Specify P01 Pin Data Input/Output */
#define P02             GPIO_PIN_ADDR(0, 2) /*!< Specify P02 Pin Data Input/Output */
#define P03             GPIO_PIN_ADDR(0, 3) /*!< Specify P03 Pin Data Input/Output */
#define P04             GPIO_PIN_ADDR(0, 4) /*!< Specify P04 Pin Data Input/Output */
#define P05             GPIO_PIN_ADDR(0, 5) /*!< Specify P05 Pin Data Input/Output */
#define P06             GPIO_PIN_ADDR(0, 6) /*!< Specify P06 Pin Data Input/Output */
#define P07             GPIO_PIN_ADDR(0, 7) /*!< Specify P07 Pin Data Input/Output */
#define P10             GPIO_PIN_ADDR(1, 0) /*!< Specify P10 Pin Data Input/Output */
#define P11             GPIO_PIN_ADDR(1, 1) /*!< Specify P11 Pin Data Input/Output */
#define P12             GPIO_PIN_ADDR(1, 2) /*!< Specify P12 Pin Data Input/Output */
#define P13             GPIO_PIN_ADDR(1, 3) /*!< Specify P13 Pin Data Input/Output */
#define P14             GPIO_PIN_ADDR(1, 4) /*!< Specify P14 Pin Data Input/Output */
#define P15             GPIO_PIN_ADDR(1, 5) /*!< Specify P15 Pin Data Input/Output */
#define P16             GPIO_PIN_ADDR(1, 6) /*!< Specify P16 Pin Data Input/Output */
#define P17             GPIO_PIN_ADDR(1, 7) /*!< Specify P17 Pin Data Input/Output */
#define P20             GPIO_PIN_ADDR(2, 0) /*!< Specify P20 Pin Data Input/Output */
#define P21             GPIO_PIN_ADDR(2, 1) /*!< Specify P21 Pin Data Input/Output */
#define P22             GPIO_PIN_ADDR(2, 2) /*!< Specify P22 Pin Data Input/Output */
#define P23             GPIO_PIN_ADDR(2, 3) /*!< Specify P23 Pin Data Input/Output */
#define P24             GPIO_PIN_ADDR(2, 4) /*!< Specify P24 Pin Data Input/Output */
#define P25             GPIO_PIN_ADDR(2, 5) /*!< Specify P25 Pin Data Input/Output */
#define P26             GPIO_PIN_ADDR(2, 6) /*!< Specify P26 Pin Data Input/Output */
#define P27             GPIO_PIN_ADDR(2, 7) /*!< Specify P27 Pin Data Input/Output */
#define P30             GPIO_PIN_ADDR(3, 0) /*!< Specify P30 Pin Data Input/Output */
#define P31             GPIO_PIN_ADDR(3, 1) /*!< Specify P31 Pin Data Input/Output */
#define P32             GPIO_PIN_ADDR(3, 2) /*!< Specify P32 Pin Data Input/Output */
#define P33             GPIO_PIN_ADDR(3, 3) /*!< Specify P33 Pin Data Input/Output */
#define P34             GPIO_PIN_ADDR(3, 4) /*!< Specify P34 Pin Data Input/Output */
#define P35             GPIO_PIN_ADDR(3, 5) /*!< Specify P35 Pin Data Input/Output */
#define P36             GPIO_PIN_ADDR(3, 6) /*!< Specify P36 Pin Data Input/Output */
#define P37             GPIO_PIN_ADDR(3, 7) /*!< Specify P37 Pin Data Input/Output */
#define P40             GPIO_PIN_ADDR(4, 0) /*!< Specify P40 Pin Data Input/Output */
#define P41             GPIO_PIN_ADDR(4, 1) /*!< Specify P41 Pin Data Input/Output */
#define P42             GPIO_PIN_ADDR(4, 2) /*!< Specify P42 Pin Data Input/Output */
#define P43             GPIO_PIN_ADDR(4, 3) /*!< Specify P43 Pin Data Input/Output */
#define P44             GPIO_PIN_ADDR(4, 4) /*!< Specify P44 Pin Data Input/Output */
#define P45             GPIO_PIN_ADDR(4, 5) /*!< Specify P45 Pin Data Input/Output */
#define P46             GPIO_PIN_ADDR(4, 6) /*!< Specify P46 Pin Data Input/Output */
#define P47             GPIO_PIN_ADDR(4, 7) /*!< Specify P47 Pin Data Input/Output */
#define P50             GPIO_PIN_ADDR(5, 0) /*!< Specify P50 Pin Data Input/Output */
#define P51             GPIO_PIN_ADDR(5, 1) /*!< Specify P51 Pin Data Input/Output */
#define P52             GPIO_PIN_ADDR(5, 2) /*!< Specify P52 Pin Data Input/Output */
#define P53             GPIO_PIN_ADDR(5, 3) /*!< Specify P53 Pin Data Input/Output */
#define P54             GPIO_PIN_ADDR(5, 4) /*!< Specify P54 Pin Data Input/Output */
#define P55             GPIO_PIN_ADDR(5, 5) /*!< Specify P55 Pin Data Input/Output */

/*@}*/ /* end of group Mini58_GPIO_EXPORTED_CONSTANTS */

/** @addtogroup Mini58_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Clear the interrupt status of specified GPIO pin.
 */
#define GPIO_CLR_INT_FLAG(gpio, u32PinMask)   ((gpio)->INTSRC = u32PinMask)

/**
 * @brief       Disable Pin De-bounce Function
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_DISABLE_DEBOUNCE(gpio, u32PinMask)   ((gpio)->DBEN &= ~u32PinMask)

/**
 * @brief       Enable Pin De-bounce Function
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable the interrupt de-bounce function of specified GPIO pin.
 */
#define GPIO_ENABLE_DEBOUNCE(gpio, u32PinMask)    ((gpio)->DBEN |= u32PinMask)

/**
 * @brief       Disable I/O Digital Input Path
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable I/O digital input path of specified GPIO pin.
 */
#define GPIO_DISABLE_DIGITAL_PATH(gpio, u32PinMask)   ((gpio)->DINOFF |= (u32PinMask << 16))

/**
 * @brief       Enable I/O Digital Input Path
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable I/O digital input path of specified GPIO pin.
 */
#define GPIO_ENABLE_DIGITAL_PATH(gpio, u32PinMask)    ((gpio)->DINOFF &= ~(u32PinMask << 16))

/**
 * @brief       Disable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Disable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_DISABLE_DOUT_MASK(gpio, u32PinMask)   ((gpio)->DATMSK &= ~u32PinMask)

/**
 * @brief       Enable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 *
 * @return      None
 *
 * @details     Enable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_ENABLE_DOUT_MASK(gpio, u32PinMask)   ((gpio)->DATMSK |= u32PinMask)

/**
 * @brief       Get GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
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
 * @param[in]   clksrc      The de-bounce counter clock source. It could be GPIO_DBCTL_DBCLKSRC_HCLK or GPIO_DBCTL_DBCLKSRC_IRC10K.
 * @param[in]   clksel      The de-bounce sampling cycle selection. It could be \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_1, \ref GPIO_DBCTL_DBCLKSEL_2, \ref GPIO_DBCTL_DBCLKSEL_4, \ref GPIO_DBCTL_DBCLKSEL_8, \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_16, \ref GPIO_DBCTL_DBCLKSEL_32, \ref GPIO_DBCTL_DBCLKSEL_64, \ref GPIO_DBCTL_DBCLKSEL_128, \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_256, \ref GPIO_DBCTL_DBCLKSEL_512, \ref GPIO_DBCTL_DBCLKSEL_1024, \ref GPIO_DBCTL_DBCLKSEL_2048, \n
 *                              - \ref GPIO_DBCTL_DBCLKSEL_4096, \ref GPIO_DBCTL_DBCLKSEL_8192, \ref GPIO_DBCTL_DBCLKSEL_16384, \ref GPIO_DBCTL_DBCLKSEL_32768.
 *
 * @return      None
 *
 * @details     Set the interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_IRC10K, GPIO_DBCTL_DBCLKSEL_4). \n
 *              It's meaning the De-debounce counter clock source is internal 10 KHz and sampling cycle selection is 4. \n
 *              Then the target de-bounce sampling cycle time is (2^4)*(1/(10*1000)) s = 16*0.0001 s = 1600 us,
 *              and system will sampling interrupt input once per 1600 us.
 */
#define GPIO_SET_DEBOUNCE_TIME(clksrc, clksel)  (GPIO->DBCTL = (GP_DBCTL_ICLKON_Msk | clksrc | clksel))

/**
 * @brief       Get GPIO Port IN Data
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 *
 * @retval      The specified port data
 *
 * @details     Get the PIN register of specified GPIO port.
 */
#define GPIO_GET_IN_DATA(gpio)   ((gpio)->PIN)

/**
 * @brief       Set GPIO Port OUT Data
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   data        GPIO port data.
 *
 * @retval      None
 *
 * @details     Set the Data into specified GPIO port.
 */
#define GPIO_SET_OUT_DATA(gpio, data)   ((gpio)->DOUT = (data))

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

/**
 * @brief       Enable External GPIO interrupt 0
 *
 * @param[in]   gpio            GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin          The pin of specified GPIO port.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING, \ref GPIO_INT_FALLING, \ref GPIO_INT_BOTH_EDGE, \ref GPIO_INT_HIGH, \ref GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_EnableEINT0    GPIO_EnableInt


/**
 * @brief       Disable External GPIO interrupt 0
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_DisableEINT0   GPIO_DisableInt


/**
 * @brief       Enable External GPIO interrupt 1
 *
 * @param[in]   gpio            GPIO port. It could \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin          The pin of specified GPIO port.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING, \ref GPIO_INT_FALLING, \ref GPIO_INT_BOTH_EDGE, \ref GPIO_INT_HIGH, \ref GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_EnableEINT1    GPIO_EnableInt


/**
 * @brief       Disable External GPIO interrupt 1
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
#define GPIO_DisableEINT1   GPIO_DisableInt


void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);



/*@}*/ /* end of group Mini58_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_GPIO_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__GPIO_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
