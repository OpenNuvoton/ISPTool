/**************************************************************************//**
 * @file     sys.h
 * @version  V3.00
 * $Revision: 12 $
 * $Date: 15/07/24 5:39p $
 * @brief    SYS Driver Header File
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/

/** @addtogroup SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/



/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_RST    ((0x4<<24) | SYS_IPRSTC2_GPIO_RST_Pos  ) /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24) | SYS_IPRSTC2_TMR0_RST_Pos  ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24) | SYS_IPRSTC2_TMR1_RST_Pos  ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24) | SYS_IPRSTC2_TMR2_RST_Pos  ) /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24) | SYS_IPRSTC2_TMR3_RST_Pos  ) /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x4<<24) | SYS_IPRSTC2_I2C1_RST_Pos  ) /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24) | SYS_IPRSTC2_SPI0_RST_Pos  ) /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define SPI1_RST    ((0x4<<24) | SYS_IPRSTC2_SPI1_RST_Pos  ) /*!< SPI1 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24) | SYS_IPRSTC2_UART0_RST_Pos ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4<<24) | SYS_IPRSTC2_UART1_RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define PWM03_RST   ((0x4<<24) | SYS_IPRSTC2_PWM03_RST_Pos ) /*!< PWM03 reset is one of the SYS_ResetModule parameter */
#define PS2_RST     ((0x4<<24) | SYS_IPRSTC2_PS2_RST_Pos   ) /*!< PS2 reset is one of the SYS_ResetModule parameter */
#define USBD_RST    ((0x4<<24) | SYS_IPRSTC2_USBD_RST_Pos  ) /*!< USBD reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCR_BOD_RST_EN            (1UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCR_BOD_INTERRUPT_EN      (0UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCR_BOD_VL_4_5V           (3UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.5V */
#define SYS_BODCR_BOD_VL_3_8V           (2UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.8V */
#define SYS_BODCR_BOD_VL_2_7V           (1UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCR_BOD_VL_2_2V           (0UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?
Example: If user want to set PA.10 as I2C1_SDA and PA.11 as I2C1_SCL in initial function,
         user can issue following command to achieve it.

         SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA10_Msk | SYS_GPA_MFP_PA11_Msk);
         SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA10_Msk | SYS_ALT_MFP_PA11_Msk);

         SYS->GPA_MFP |= (SYS_GPA_MFP_PA10_I2C1_SDA | SYS_GPA_MFP_PA11_I2C1_SCL);
         SYS->ALT_MFP |= (SYS_ALT_MFP_PA10_I2C1_SDA | SYS_ALT_MFP_PA11_I2C1_SCL);
*/

//PA.10
#define SYS_GPA_MFP_PA10_GPIO           0x00000000UL    /*!< GPA_MFP PA.10 setting for GPIO */
#define SYS_ALT_MFP_PA10_GPIO           0x00000000UL    /*!< ALT_MFP PA.10 setting for GPIO */

#define SYS_GPA_MFP_PA10_I2C1_SDA       (1UL<<10)       /*!< GPA_MFP PA.10 setting for I2C1_SDA */
#define SYS_ALT_MFP_PA10_I2C1_SDA       0x00000000UL    /*!< ALT_MFP PA.10 setting for I2C1_SDA */

#define SYS_GPA_MFP_PA10_Msk            (1UL<<10)       /*!< GPA_MFP PA.10 mask */
#define SYS_ALT_MFP_PA10_Msk            (1UL<<11)       /*!< ALT_MFP PA.10 mask */

//PA.11
#define SYS_GPA_MFP_PA11_GPIO           0x00000000UL    /*!< GPA_MFP PA.11 setting for GPIO */
#define SYS_ALT_MFP_PA11_GPIO           0x00000000UL    /*!< ALT_MFP PA.11 setting for GPIO */

#define SYS_GPA_MFP_PA11_I2C1_SCL       (1UL<<11)       /*!< GPA_MFP PA.11 setting for I2C1_SCL */
#define SYS_ALT_MFP_PA11_I2C1_SCL       0x00000000UL    /*!< ALT_MFP PA.11 setting for I2C1_SCL */

#define SYS_GPA_MFP_PA11_Msk            (1UL<<11)       /*!< GPA_MFP PA.11 mask */
#define SYS_ALT_MFP_PA11_Msk            (1UL<<11)       /*!< ALT_MFP PA.11 mask */

//PA.12
#define SYS_GPA_MFP_PA12_GPIO           0x00000000UL    /*!< GPA_MFP PA.12 setting for GPIO */
#define SYS_ALT_MFP_PA12_GPIO           0x00000000UL    /*!< ALT_MFP PA.12 setting for GPIO */

#define SYS_GPA_MFP_PA12_PWM0           (1UL<<12)       /*!< GPA_MFP PA.12 setting for PWM0 */
#define SYS_ALT_MFP_PA12_PWM0           0x00000000UL    /*!< ALT_MFP PA.12 setting for PWM0 */

#define SYS_GPA_MFP_PA12_Msk            (1UL<<12)       /*!< GPA_MFP PA.12 mask */
#define SYS_ALT_MFP_PA12_Msk            (1UL<<11)       /*!< ALT_MFP PA.12 mask */

//PA.13
#define SYS_GPA_MFP_PA13_GPIO           0x00000000UL    /*!< GPA_MFP PA.13 setting for GPIO */
#define SYS_ALT_MFP_PA13_GPIO           0x00000000UL    /*!< ALT_MFP PA.13 setting for GPIO */

#define SYS_GPA_MFP_PA13_PWM1           (1UL<<13)       /*!< GPA_MFP PA.13 setting for PWM1 */
#define SYS_ALT_MFP_PA13_PWM1           0x00000000UL    /*!< ALT_MFP PA.13 setting for PWM1 */

#define SYS_GPA_MFP_PA13_Msk            (1UL<<13)       /*!< GPA_MFP PA.13 mask */
#define SYS_ALT_MFP_PA13_Msk            (1UL<<11)       /*!< ALT_MFP PA.13 mask */

//PA.14
#define SYS_GPA_MFP_PA14_GPIO           0x00000000UL    /*!< GPA_MFP PA.14 setting for GPIO */
#define SYS_ALT_MFP_PA14_GPIO           0x00000000UL    /*!< ALT_MFP PA.14 setting for GPIO */

#define SYS_GPA_MFP_PA14_PWM2           (1UL<<14)       /*!< GPA_MFP PA.14 setting for PWM2 */
#define SYS_ALT_MFP_PA14_PWM2           0x00000000UL    /*!< ALT_MFP PA.14 setting for PWM2 */

#define SYS_GPA_MFP_PA14_Msk            (1UL<<14)       /*!< GPA_MFP PA.14 mask */
#define SYS_ALT_MFP_PA14_Msk            (1UL<<11)       /*!< ALT_MFP PA.14 mask */

//PA.15
#define SYS_GPA_MFP_PA15_GPIO           0x00000000UL    /*!< GPA_MFP PA.15 setting for GPIO */
#define SYS_ALT_MFP_PA15_GPIO           0x00000000UL    /*!< ALT_MFP PA.15 setting for GPIO */

#define SYS_GPA_MFP_PA15_PWM3           (1UL<<15)       /*!< GPA_MFP PA.15 setting for PWM3 */
#define SYS_ALT_MFP_PA15_PWM3           0x00000000UL    /*!< ALT_MFP PA.15 setting for PWM3 */

#define SYS_GPA_MFP_PA15_Msk            (1UL<<15)       /*!< GPA_MFP PA.15 mask */
#define SYS_ALT_MFP_PA15_Msk            (1UL<<9)        /*!< ALT_MFP PA.15 mask */

//PB.0
#define SYS_GPB_MFP_PB0_GPIO            0x00000000UL    /*!< GPB_MFP PB.0 setting for GPIO */
#define SYS_ALT_MFP_PB0_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PB.0 */

#define SYS_GPB_MFP_PB0_UART0_RXD       (1UL<<0)        /*!< GPB_MFP PB.0 setting for UART0_RXD */
#define SYS_ALT_MFP_PB0_UART0_RXD       (uint32_t)NULL  /*!< No ALT_MFP setting for PB.0 */

#define SYS_GPB_MFP_PB0_Msk             (1UL<<0)        /*!< GPB_MFP PB.0 mask */
#define SYS_ALT_MFP_PB0_Msk             (uint32_t)NULL  /*!< No ALT_MFP PB.0 mask */


//PB.1
#define SYS_GPB_MFP_PB1_GPIO            0x00000000UL    /*!< GPB_MFP PB.1 setting for GPIO */
#define SYS_ALT_MFP_PB1_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PB.1 */

#define SYS_GPB_MFP_PB1_UART0_TXD       (1UL<<1)        /*!< GPB_MFP PB.1 setting for UART0_TXD */
#define SYS_ALT_MFP_PB1_UART0_TXD       (uint32_t)NULL  /*!< No ALT_MFP setting for PB.1 */

#define SYS_GPB_MFP_PB1_Msk             (1UL<<1)        /*!< GPB_MFP PB.1 mask */
#define SYS_ALT_MFP_PB1_Msk             (uint32_t)NULL  /*!< No ALT_MFP PB.1 mask */

//PB.2
#define SYS_GPB_MFP_PB2_GPIO            0x00000000UL    /*!< GPB_MFP PB.2 setting for GPIO */
#define SYS_ALT_MFP_PB2_GPIO            0x00000000UL    /*!< ALT_MFP PB.2 setting for GPIO */

#define SYS_GPB_MFP_PB2_UART0_nRST      (1UL<<2)        /*!< GPB_MFP PB.2 setting for UART0_nRST */
#define SYS_ALT_MFP_PB2_UART0_nRST      0x00000000UL    /*!< ALT_MFP PB.2 setting for UART0_nRST */

#define SYS_GPB_MFP_PB2_Msk             (1UL<<2)        /*!< GPB_MFP PB.2 mask */
#define SYS_ALT_MFP_PB2_Msk             (1UL<<11)       /*!< ALT_MFP PB.2 mask */

//PB.3
#define SYS_GPB_MFP_PB3_GPIO            0x00000000UL    /*!< GPB_MFP PB.3 setting for GPIO */
#define SYS_ALT_MFP_PB3_GPIO            0x00000000UL    /*!< ALT_MFP PB.3 setting for GPIO */

#define SYS_GPB_MFP_PB3_UART0_nCTS      (1UL<<3)        /*!< GPB_MFP PB.3 setting for UART0_nCTS */
#define SYS_ALT_MFP_PB3_UART0_nCTS      0x00000000UL    /*!< ALT_MFP PB.3 setting for UART0_nCTS */

#define SYS_GPB_MFP_PB3_Msk             (1UL<<3)        /*!< GPB_MFP PB.3 mask */
#define SYS_ALT_MFP_PB3_Msk             (1UL<<11)       /*!< ALT_MFP PB.3 mask */

//PB.4
#define SYS_GPB_MFP_PB4_GPIO            0x00000000UL    /*!< GPB_MFP PB.4 setting for GPIO */
#define SYS_ALT_MFP_PB4_GPIO            0x00000000UL    /*!< ALT_MFP PB.4 setting for GPIO */

#define SYS_GPB_MFP_PB4_UART1_RXD       (1UL<<4)        /*!< GPB_MFP PB.4 setting for UART1_RXD */
#define SYS_ALT_MFP_PB4_UART1_RXD       0x00000000UL    /*!< ALT_MFP PB.4 setting for UART1_RXD */

#define SYS_GPB_MFP_PB4_SPI1_SS1        (1UL<<4)        /*!< GPB_MFP PB.4 setting for SPI1_SS1 */
#define SYS_ALT_MFP_PB4_SPI1_SS1        (1UL<<15)       /*!< ALT_MFP PB.4 setting for SPI1_SS1 */

#define SYS_GPB_MFP_PB4_Msk             (1UL<<4)        /*!< GPB_MFP PB.4 mask */
#define SYS_ALT_MFP_PB4_Msk             (1UL<<15)       /*!< ALT_MFP PB.4 mask */

//PB.5
#define SYS_GPB_MFP_PB5_GPIO            0x00000000UL    /*!< GPB_MFP PB.5 setting for GPIO */
#define SYS_ALT_MFP_PB5_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PB.5 */

#define SYS_GPB_MFP_PB5_UART1_TXD       (1UL<<5)        /*!< GPB_MFP PB.5 setting for UART1_TXD */
#define SYS_ALT_MFP_PB5_UART1_TXD       (uint32_t)NULL  /*!< No ALT_MFP setting for PB.5 */

#define SYS_GPB_MFP_PB5_Msk             (1UL<<5)        /*!< GPB_MFP PB.5 mask */
#define SYS_ALT_MFP_PB5_Msk             (uint32_t)NULL  /*!< No ALT_MFP PB.5 mask */

//PB.6
#define SYS_GPB_MFP_PB6_GPIO            0x00000000UL    /*!< GPB_MFP PB.6 setting for GPIO */
#define SYS_ALT_MFP_PB6_GPIO            0x00000000UL    /*!< ALT_MFP PB.6 setting for GPIO */

#define SYS_GPB_MFP_PB6_UART1_nRTS      (1UL<<6)        /*!< GPB_MFP PB.6 setting for UART1_nRTS */
#define SYS_ALT_MFP_PB6_UART1_nRTS      0x00000000UL    /*!< ALT_MFP PB.6 setting for UART1_nRTS */

#define SYS_GPB_MFP_PB6_Msk             (1UL<<6)        /*!< GPB_MFP PB.6 mask */
#define SYS_ALT_MFP_PB6_Msk             (1UL<<17)       /*!< ALT_MFP PB.6 mask */

//PB.7
#define SYS_GPB_MFP_PB7_GPIO            0x00000000UL    /*!< GPB_MFP PB.7 setting for GPIO */
#define SYS_ALT_MFP_PB7_GPIO            0x00000000UL    /*!< ALT_MFP PB.7 setting for GPIO */

#define SYS_GPB_MFP_PB7_UART1_nCTS      (1UL<<7)        /*!< GPB_MFP PB.7 setting for UART1_nCTS */
#define SYS_ALT_MFP_PB7_UART1_nCTS      0x00000000UL    /*!< ALT_MFP PB.7 setting for UART1_nCTS */

#define SYS_GPB_MFP_PB7_Msk             (1UL<<7)        /*!< GPB_MFP PB.7 mask */
#define SYS_ALT_MFP_PB7_Msk             (1UL<<16)       /*!< ALT_MFP PB.7 mask */

//PB.8
#define SYS_GPB_MFP_PB8_GPIO            0x00000000UL    /*!< GPB_MFP PB.8 setting for GPIO */
#define SYS_ALT_MFP_PB8_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PB.8 */

#define SYS_GPB_MFP_PB8_TM0             (1UL<<8)        /*!< GPB_MFP PB.8 setting for TM0 */
#define SYS_ALT_MFP_PB8_TM0             (uint32_t)NULL  /*!< No ALT_MFP setting for PB.8 */

#define SYS_GPB_MFP_PB8_Msk             (1UL<<8)        /*!< GPB_MFP PB.8 mask */
#define SYS_ALT_MFP_PB8_Msk             (uint32_t)NULL  /*!< ALT_MFP PB.8 mask */

//PB.9
#define SYS_GPB_MFP_PB9_GPIO            0x00000000UL    /*!< GPB_MFP PB.9 setting for GPIO */
#define SYS_ALT_MFP_PB9_GPIO            0x00000000UL    /*!< ALT_MFP PB.9 setting for GPIO */

#define SYS_GPB_MFP_PB9_TM1             (1UL<<9)        /*!< GPB_MFP PB.9 setting for TM1 */
#define SYS_ALT_MFP_PB9_TM1             0x00000000UL    /*!< ALT_MFP PB.9 setting for TM1 */

#define SYS_GPB_MFP_PB9_SPI1_SS1        (1UL<<9)        /*!< GPB_MFP PB.9 setting for SPI1_SS1 */
#define SYS_ALT_MFP_PB9_SPI1_SS1        (1UL<<1)        /*!< ALT_MFP PB.9 setting for SPI1_SS1 */

#define SYS_GPB_MFP_PB9_Msk             (1UL<<9)        /*!< GPB_MFP PB.9 mask */
#define SYS_ALT_MFP_PB9_Msk             (1UL<<1)        /*!< ALT_MFP PB.9 mask */

//PB.10
#define SYS_GPB_MFP_PB10_GPIO           0x00000000UL    /*!< GPB_MFP PB.10 setting for GPIO */
#define SYS_ALT_MFP_PB10_GPIO           0x00000000UL    /*!< ALT_MFP PB.10 setting for GPIO */

#define SYS_GPB_MFP_PB10_TM2            (1UL<<10)       /*!< GPB_MFP PB.10 setting for TM2 */
#define SYS_ALT_MFP_PB10_TM2            0x00000000UL    /*!< ALT_MFP PB.10 setting for TM2 */

#define SYS_GPB_MFP_PB10_SPI0_SS1       (1UL<<10)       /*!< GPB_MFP PB.10 setting for SPI0_SS1 */
#define SYS_ALT_MFP_PB10_SPI0_SS1       (1UL<<0)        /*!< ALT_MFP PB.10 setting for SPI0_SS1 */

#define SYS_GPB_MFP_PB10_Msk            (1UL<<10)       /*!< GPB_MFP PB.10 mask */
#define SYS_ALT_MFP_PB10_Msk            (1UL<<0)        /*!< ALT_MFP PB.10 mask */

//PB.14
#define SYS_GPB_MFP_PB14_GPIO           0x00000000UL    /*!< GPB_MFP PB.14 setting for GPIO */
#define SYS_ALT_MFP_PB14_GPIO           0x00000000UL    /*!< ALT_MFP PB.14 setting for GPIO */

#define SYS_GPB_MFP_PB14_INT0           (1UL<<14)       /*!< GPB_MFP PB.14 setting for INT0 */
#define SYS_ALT_MFP_PB14_INT0           0x00000000UL    /*!< ALT_MFP PB.14 setting for INT0 */

#define SYS_GPB_MFP_PB14_Msk            (1UL<<14)       /*!< GPB_MFP PB.14 mask */
#define SYS_ALT_MFP_PB14_Msk            (1UL<<3)        /*!< ALT_MFP PB.14 mask */

//PB.15
#define SYS_GPB_MFP_PB15_GPIO           0x00000000UL    /*!< GPB_MFP PB.15 setting for GPIO */
#define SYS_ALT_MFP_PB15_GPIO           (uint32_t)NULL  /*!< No ALT_MFP1 setting for PB.15 */

#define SYS_GPB_MFP_PB15_INT1           (1UL<<15)       /*!< GPB_MFP PB.15 setting for INT1 */
#define SYS_ALT_MFP_PB15_INT1           (uint32_t)NULL  /*!< No ALT_MFP1 setting for PB.15 */

#define SYS_GPB_MFP_PB15_Msk            (1UL<<15)       /*!< GPB_MFP PB.15 mask */
#define SYS_ALT_MFP_PB15_Msk            (uint32_t)NULL  /*!< No ALT_MFP PB.15 mask */

//PC.0
#define SYS_GPC_MFP_PC0_GPIO            0x00000000UL    /*!< GPB_MFP PC.0 setting for GPIO */
#define SYS_ALT_MFP_PC0_GPIO            0x00000000UL    /*!< ALT_MFP PC.0 setting for GPIO */

#define SYS_GPC_MFP_PC0_SPI0_SS0        (1UL<<0)        /*!< GPB_MFP PC.0 setting for SPI0_SS0 */
#define SYS_ALT_MFP_PC0_SPI0_SS0        0x00000000UL    /*!< ALT_MFP PC.0 setting for SPI0_SS0 */

#define SYS_GPC_MFP_PC0_Msk             (1UL<<0)        /*!< GPC_MFP PC.0 mask */
#define SYS_ALT_MFP_PC0_Msk             (1UL<<5)        /*!< ALT_MFP PC.0 mask */

//PC.1
#define SYS_GPC_MFP_PC1_GPIO            0x00000000UL    /*!< GPC_MFP PC.1 setting for GPIO */
#define SYS_ALT_MFP_PC1_GPIO            0x00000000UL    /*!< ALT_MFP PC.1 setting for GPIO */

#define SYS_GPC_MFP_PC1_SPI0_CLK        (1UL<<1)        /*!< GPC_MFP PC.1 setting for SPI0_CLK */
#define SYS_ALT_MFP_PC1_SPI0_CLK        0x00000000UL    /*!< ALT_MFP PC.1 setting for SPI0_CLK */

#define SYS_GPC_MFP_PC1_Msk             (1UL<<1)        /*!< GPC_MFP PC.1 mask */
#define SYS_ALT_MFP_PC1_Msk             (1UL<<6)        /*!< ALT_MFP PC.1 mask */

//PC.2
#define SYS_GPC_MFP_PC2_GPIO            0x00000000UL    /*!< GPC_MFP PC.2 setting for GPIO */
#define SYS_ALT_MFP_PC2_GPIO            0x00000000UL    /*!< ALT_MFP PC.2 setting for GPIO */

#define SYS_GPC_MFP_PC2_SPI0_MISO0      (1UL<<2)        /*!< GPC_MFP PC.2 setting for SPI0_MISO0 */
#define SYS_ALT_MFP_PC2_SPI0_MISO0      0x00000000UL    /*!< ALT_MFP PC.2 setting for SPI0_MISO0 */

#define SYS_GPC_MFP_PC2_Msk             (1UL<<2)        /*!< GPC_MFP PC.2 mask */
#define SYS_ALT_MFP_PC2_Msk             (1UL<<7)        /*!< ALT_MFP PC.2 mask */

//PC.3
#define SYS_GPC_MFP_PC3_GPIO            0x00000000UL    /*!< GPC_MFP PC.3 setting for GPIO */
#define SYS_ALT_MFP_PC3_GPIO            0x00000000UL    /*!< ALT_MFP PC.3 setting for GPIO */

#define SYS_GPC_MFP_PC3_SPI0_MOSI0      (1UL<<3)        /*!< GPC_MFP PC.3 setting for SPI0_MOSI0 */
#define SYS_ALT_MFP_PC3_SPI0_MOSI0      0x00000000UL    /*!< ALT_MFP PC.3 setting for SPI0_MOSI0 */

#define SYS_GPC_MFP_PC3_Msk             (1UL<<3)        /*!< GPC_MFP PC.3 mask */
#define SYS_ALT_MFP_PC3_Msk             (1UL<<8)        /*!< ALT_MFP PC.3 mask */

//PC.4
#define SYS_GPC_MFP_PC4_GPIO            0x00000000UL    /*!< GPC_MFP PC.4 setting for GPIO */
#define SYS_ALT_MFP_PC4_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PC.4 */

#define SYS_GPC_MFP_PC4_Msk             (1UL<<4)        /*!< GPC_MFP PC.4 mask */
#define SYS_ALT_MFP_PC4_Msk             (uint32_t)NULL  /*!< No ALT_MFP PC.4 mask */

//PC.5
#define SYS_GPC_MFP_PC5_GPIO            0x00000000UL    /*!< GPC_MFP PC.5 setting for GPIO */
#define SYS_ALT_MFP_PC5_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PC.5 */

#define SYS_GPC_MFP_PC5_Msk             (1UL<<5)        /*!< GPC_MFP PC.5 mask */
#define SYS_ALT_MFP_PC5_Msk             (uint32_t)NULL  /*!< ALT_MFP PC.5 mask */

//PC.8
#define SYS_GPC_MFP_PC8_GPIO            0x00000000UL    /*!< GPC_MFP PC.8 setting for GPIO */
#define SYS_ALT_MFP_PC8_GPIO            0x00000000UL    /*!< ALT_MFP PC.8 setting for GPIO */

#define SYS_GPC_MFP_PC8_SPI1_SS0        (1UL<<8)        /*!< GPC_MFP PC.8 setting for SPI1_SS0 */
#define SYS_ALT_MFP_PC8_SPI1_SS0        0x00000000UL    /*!< ALT_MFP PC.8 setting for SPI1_SS0 */

#define SYS_GPC_MFP_PC8_Msk             (1UL<<8)        /*!< GPC_MFP PC.8 mask */
#define SYS_ALT_MFP_PC8_Msk             (1UL<<16)       /*!< ALT_MFP PC.8 mask */

//PC.9
#define SYS_GPC_MFP_PC9_GPIO            0x00000000UL    /*!< GPC_MFP PC.9 setting for GPIO */
#define SYS_ALT_MFP_PC9_GPIO            0x00000000UL    /*!< ALT_MFP PC.9 setting for GPIO */

#define SYS_GPC_MFP_PC9_SPI1_CLK        (1UL<<9)        /*!< GPC_MFP PC.9 setting for SPI1_CLK */
#define SYS_ALT_MFP_PC9_SPI1_CLK        0x00000000UL    /*!< ALT_MFP PC.9 setting for SPI1_CLK */

#define SYS_GPC_MFP_PC9_Msk             (1UL<<9)        /*!< GPC_MFP PC.9 mask */
#define SYS_ALT_MFP_PC9_Msk             (1UL<<17)       /*!< ALT_MFP PC.9 mask */

//PC.10
#define SYS_GPC_MFP_PC10_GPIO           0x00000000UL    /*!< GPC_MFP PC.10 setting for GPIO */
#define SYS_ALT_MFP_PC10_GPIO           0x00000000UL    /*!< ALT_MFP PC.10 setting for GPIO */

#define SYS_GPC_MFP_PC10_SPI1_MISO0     (1UL<<10)       /*!< GPC_MFP PC.10 setting for SPI1_MISO0 */
#define SYS_ALT_MFP_PC10_SPI1_MISO0     0x00000000UL    /*!< ALT_MFP PC.10 setting for SPI1_MISO0 */

#define SYS_GPC_MFP_PC10_Msk            (1UL<<10)       /*!< GPC_MFP PC.10 mask */
#define SYS_ALT_MFP_PC10_Msk            (1UL<<18)       /*!< ALT_MFP PC.10 mask */

//PC.11
#define SYS_GPC_MFP_PC11_GPIO           0x00000000UL    /*!< GPC_MFP PC.11 setting for GPIO */
#define SYS_ALT_MFP_PC11_GPIO           (1UL<<18)       /*!< ALT_MFP PC.11 setting for GPIO */

#define SYS_GPC_MFP_PC11_SPI1_MOSI0     (1UL<<11)       /*!< GPC_MFP PC.11 setting for SPI1_MOSI0 */
#define SYS_ALT_MFP_PC11_SPI1_MOSI0     0x00000000UL    /*!< ALT_MFP PC.11 setting for SPI1_MOSI0 */

#define SYS_GPC_MFP_PC11_Msk            (1UL<<11)       /*!< GPC_MFP PC.11 mask */
#define SYS_ALT_MFP_PC11_Msk            (1UL<<18)       /*!< ALT_MFP PC.11 mask */

//PC.12
#define SYS_GPC_MFP_PC12_GPIO           0x00000000UL    /*!< GPC_MFP PC.12 setting for GPIO */
#define SYS_ALT_MFP_PC12_GPIO           (uint32_t)NULL  /*!< No ALT_MFP setting for PC.12 */

#define SYS_GPC_MFP_PC12_Msk            (1UL<<12)       /*!< GPC_MFP PC.12 mask */
#define SYS_ALT_MFP_PC12_Msk            (uint32_t)NULL  /*!< No ALT_MFP PC.12 mask */

//PC.13
#define SYS_GPC_MFP_PC13_GPIO           0x00000000UL    /*!< GPC_MFP PC.13 setting for GPIO */
#define SYS_ALT_MFP_PC13_GPIO           (uint32_t)NULL  /*!< No ALT_MFP setting for PC.13 */

#define SYS_GPC_MFP_PC13_Msk            (1UL<<12)       /*!< GPC_MFP PC.13 mask */
#define SYS_ALT_MFP_PC13_Msk            (uint32_t)NULL  /*!< No ALT_MFP PC.13 mask */

//PD.0
#define SYS_GPD_MFP_PD0_GPIO            0x00000000UL    /*!< GPD_MFP PD.0 setting for GPIO */
#define SYS_ALT_MFP_PD0_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PD.0 */

#define SYS_GPD_MFP_PD0_Msk             (1UL<<0)        /*!< GPD_MFP PD.0 mask */
#define SYS_ALT_MFP_PD0_Msk             (uint32_t)NULL  /*!< No ALT_MFP PD.0 mask */

//PD.1
#define SYS_GPD_MFP_PD1_GPIO            0x00000000UL    /*!< GPD_MFP PD.1 setting for GPIO */
#define SYS_ALT_MFP_PD1_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PD.1 */

#define SYS_GPD_MFP_PD1_SPI0_SS1        (1UL<<1)        /*!< GPD_MFP PD.1 setting for SPI0_SS1 */
#define SYS_ALT_MFP_PD1_SPI0_SS1        (uint32_t)NULL  /*!< No ALT_MFP setting for PD.1 */

#define SYS_GPD_MFP_PD1_Msk             (1UL<<1)        /*!< GPD_MFP PD.1 mask */
#define SYS_ALT_MFP_PD1_Msk             (uint32_t)NULL  /*!< No ALT_MFP PD.1 mask */

//PD.2
#define SYS_GPD_MFP_PD2_GPIO            0x00000000UL    /*!< GPD_MFP PD.2 setting for GPIO */
#define SYS_ALT_MFP_PD2_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PD.2 */

#define SYS_GPD_MFP_PD2_Msk             (1UL<<2)        /*!< GPD_MFP PD.2 mask */
#define SYS_ALT_MFP_PD2_Msk             (uint32_t)NULL  /*!< No ALT_MFP PD.2 mask */

//PD.3
#define SYS_GPD_MFP_PD3_GPIO            0x00000000UL    /*!< GPD_MFP PD.3 setting for GPIO */
#define SYS_ALT_MFP_PD3_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PD.3 */

#define SYS_GPD_MFP_PD3_Msk             (1UL<<3)        /*!< GPD_MFP PD.3 mask */
#define SYS_ALT_MFP_PD3_Msk             (uint32_t)NULL  /*!< No ALT_MFP PD.3 mask */

//PD.4
#define SYS_GPD_MFP_PD4_GPIO            0x00000000UL    /*!< GPD_MFP PD.4 setting for GPIO */
#define SYS_ALT_MFP_PD4_GPIO            (uint32_t)NULL  /*!< No ALT_MFP setting for PD.4 */

#define SYS_GPD_MFP_PD4_Msk             (1UL<<4)        /*!< GPD_MFP PD.4 mask */
#define SYS_ALT_MFP_PD4_Msk             (uint32_t)NULL  /*!< No ALT_MFP PD.4 mask */

//PD.5
#define SYS_GPD_MFP_PD5_GPIO            0x00000000UL    /*!< GPD_MFP PD.5 setting for GPIO */
#define SYS_ALT_MFP_PD5_GPIO            (uint32_t)NULL   /*!< No ALT_MFP setting for PD.5 */

#define SYS_GPD_MFP_PD5_Msk             (1UL<<5)        /*!< GPD_MFP PD.5 mask */
#define SYS_ALT_MFP_PD5_Msk             (uint32_t)NULL  /*!< No ALT_MFP PD.5 mask */

//PD.8
#define SYS_GPD_MFP_PD8_GPIO            0x00000000UL    /*!< GPD_MFP PD.8 setting for GPIO */
#define SYS_ALT_MFP_PD8_GPIO            0x00000000UL    /*!< ALT_MFP PD.8 setting for GPIO */

#define SYS_GPD_MFP_PD8_Msk             (1UL<<8)        /*!< GPD_MFP PD.8 mask */
#define SYS_ALT_MFP_PD8_Msk             (1UL<<18)       /*!< ALT_MFP PD.8 mask */

//PD.9
#define SYS_GPD_MFP_PD9_GPIO            0x00000000UL    /*!< GPD_MFP PD.9 setting for GPIO */
#define SYS_ALT_MFP_PD9_GPIO            0x00000000UL    /*!< ALT_MFP PD.9 setting for GPIO */

#define SYS_GPD_MFP_PD9_Msk             (1UL<<9)        /*!< GPD_MFP PD.9 mask */
#define SYS_ALT_MFP_PD9_Msk             (1UL<<19)       /*!< ALT_MFP PD.9 mask */

//PD.10
#define SYS_GPD_MFP_PD10_GPIO           0x00000000UL    /*!< GPD_MFP PD.10 setting for GPIO */
#define SYS_ALT_MFP_PD10_GPIO           0x00000000UL    /*!< ALT_MFP PD.10 setting for GPIO */

#define SYS_GPD_MFP_PD10_Msk            (1UL<<10)       /*!< GPD_MFP PD.10 mask */
#define SYS_ALT_MFP_PD10_Msk            (1UL<<20)       /*!< ALT_MFP PD.10 mask */

//PD.11
#define SYS_GPD_MFP_PD11_GPIO           0x00000000UL    /*!< GPD_MFP PD.11 setting for GPIO */
#define SYS_ALT_MFP_PD11_GPIO           0x00000000UL    /*!< ALT_MFP PD.11 setting for GPIO */

#define SYS_GPD_MFP_PD11_Msk            (1UL<<11)       /*!< GPD_MFP PD.11 mask */
#define SYS_ALT_MFP_PD11_Msk            (1UL<<21)       /*!< ALT_MFP PD.11 mask */


/*@}*/ /* end of group SYS_EXPORTED_CONSTANTS */

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/


/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCR |= SYS_BODCR_BOD_INTF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCR &= ~SYS_BODCR_BOD_LPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCR &= ~SYS_BODCR_BOD_EN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCR |= SYS_BODCR_BOD_EN_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCR & SYS_BODCR_BOD_INTF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCR & SYS_BODCR_BOD_OUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCR &= ~SYS_BODCR_BOD_RSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCR |= SYS_BODCR_BOD_RSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCR |= SYS_BODCR_BOD_LPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCR_BOD_VL_4_5V
  *             - \ref SYS_BODCR_BOD_VL_3_8V
  *             - \ref SYS_BODCR_BOD_VL_2_7V
  *             - \ref SYS_BODCR_BOD_VL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCR = (SYS->BODCR & ~SYS_BODCR_BOD_VL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_BOD_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_CPU_Msk)

/**
  * @brief      Get reset source is from Low-Voltage-Reset
  * @param      None     
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.   
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_LVR_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_POR_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSRC & SYS_RSTSRC_RSTS_RESET_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSRC & SYS_RSTSRC_RSTS_SYS_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_WDT_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_LVR()               (SYS->BODCR &= ~SYS_BODCR_LVR_EN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_LVR()                (SYS->BODCR |= SYS_BODCR_LVR_EN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCR = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCR = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including:
  *             - \ref SYS_RSTSRC_RSTS_CPU_Msk
  *             - \ref SYS_RSTSRC_RSTS_SYS_Msk
  *             - \ref SYS_RSTSRC_RSTS_BOD_Msk
  *             - \ref SYS_RSTSRC_RSTS_LVR_Msk
  *             - \ref SYS_RSTSRC_RSTS_WDT_Msk
  *             - \ref SYS_RSTSRC_RSTS_RESET_Msk
  *             - \ref SYS_RSTSRC_RSTS_POR_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) (SYS->RSTSRC = (u32RstSrc) )


/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
static __INLINE void SYS_LockReg(void)
{
    SYS->REGWRPROT = 0;
}

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  *
  */
static __INLINE void SYS_UnlockReg(void)
{
    while(SYS->REGWRPROT != SYS_REGWRPROT_REGPROTDIS_Msk)
    {
        SYS->REGWRPROT = 0x59;
        SYS->REGWRPROT = 0x16;
        SYS->REGWRPROT = 0x88;
    }
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


/*@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SYS_Driver */

/*@}*/ /* end of group Device_Driver */


#ifdef __cplusplus
}
#endif

#endif //__SYS_H__

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
