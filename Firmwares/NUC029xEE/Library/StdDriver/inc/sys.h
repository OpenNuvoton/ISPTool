/**************************************************************************//**
 * @file     sys.h
 * @version  V3.00
 * @brief    NUC029xEE Series SYS Driver Header File
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
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
#define PDMA_RST    ((0x0<<24) | SYS_IPRSTC1_PDMA_RST_Pos  ) /*!< PDMA reset is one of the SYS_ResetModule parameter */
#define EBI_RST     ((0x0<<24) | SYS_IPRSTC1_EBI_RST_Pos   ) /*!< EBI reset is one of the SYS_ResetModule parameter */
#define GPIO_RST    ((0x4<<24) | SYS_IPRSTC2_GPIO_RST_Pos  ) /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24) | SYS_IPRSTC2_TMR0_RST_Pos  ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24) | SYS_IPRSTC2_TMR1_RST_Pos  ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24) | SYS_IPRSTC2_TMR2_RST_Pos  ) /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24) | SYS_IPRSTC2_TMR3_RST_Pos  ) /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4<<24) | SYS_IPRSTC2_I2C0_RST_Pos  ) /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x4<<24) | SYS_IPRSTC2_I2C1_RST_Pos  ) /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24) | SYS_IPRSTC2_SPI0_RST_Pos  ) /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define SPI1_RST    ((0x4<<24) | SYS_IPRSTC2_SPI1_RST_Pos  ) /*!< SPI1 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24) | SYS_IPRSTC2_UART0_RST_Pos ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4<<24) | SYS_IPRSTC2_UART1_RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART2_RST   ((0x4<<24) | SYS_IPRSTC2_UART2_RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define PWM03_RST   ((0x4<<24) | SYS_IPRSTC2_PWM03_RST_Pos ) /*!< PWM03 reset is one of the SYS_ResetModule parameter */
#define PWM45_RST   ((0x4<<24) | SYS_IPRSTC2_PWM45_RST_Pos ) /*!< PWM45 reset is one of the SYS_ResetModule parameter */
#define USBD_RST    ((0x4<<24) | SYS_IPRSTC2_USBD_RST_Pos  ) /*!< USBD reset is one of the SYS_ResetModule parameter */
#define ADC_RST     ((0x4<<24) | SYS_IPRSTC2_ADC_RST_Pos   ) /*!< ADC reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCR_BOD_RST_EN            (1UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCR_BOD_INTERRUPT_EN      (0UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCR_BOD_VL_4_4V           (3UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.4V */
#define SYS_BODCR_BOD_VL_3_7V           (2UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCR_BOD_VL_2_7V           (1UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCR_BOD_VL_2_2V           (0UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?
Example: If user want to set PA.0 as ADC0 and PA1 as ADC1 in initial function,
         user can issue following command to achieve it.

         SYS->GPA_MFP  = SYS_GPA_MFP_PA0_ADC0 | SYS_GPA_MFP_PA1_ADC1;
         SYS->ALT_MFP1 = SYS_ALT_MFP1_PA0_ADC0| SYS_ALT_MFP1_PA1_ADC1;
*/

//PA.0
#define SYS_GPA_MFP_PA0_GPIO        0x00000000UL        /*!< GPA_MFP PA.0 setting for GPIO */
#define SYS_ALT_MFP_PA0_GPIO        0UL                 /*!< No ALT_MFP setting for PA.0 */
#define SYS_ALT_MFP2_PA0_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.0 */

#define SYS_GPA_MFP_PA0_ADC0        (1UL<<0)            /*!< GPA_MFP PA.0 setting for ADC0 */
#define SYS_ALT_MFP_PA0_ADC0        0UL                 /*!< No ALT_MFP setting for PA.0 */
#define SYS_ALT_MFP2_PA0_ADC0       0UL                 /*!< No ALT_MFP2 setting for PA.0 */

#define SYS_GPA_MFP_PA0_Msk         (1UL<<0)            /*!< GPA_MFP PA.0 mask */
#define SYS_ALT_MFP_PA0_Msk         0UL                 /*!< ALT_MFP PA.0 mask */
#define SYS_ALT_MFP2_PA0_Msk        0UL                 /*!< ALT_MFP2 PA.0 mask */

//PA.1
#define SYS_GPA_MFP_PA1_GPIO        0x00000000UL        /*!< GPA_MFP PA.1 setting for GPIO */
#define SYS_ALT_MFP_PA1_GPIO        0x00000000UL        /*!< ALT_MFP PA.1 setting for GPIO */
#define SYS_ALT_MFP2_PA1_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.1 */

#define SYS_GPA_MFP_PA1_ADC1        (1UL<<1)            /*!< GPA_MFP PA.1 setting for ADC1 */
#define SYS_ALT_MFP_PA1_ADC1        0x00000000UL        /*!< ALT_MFP PA.1 setting for ADC1 */
#define SYS_ALT_MFP2_PA1_ADC1       0UL                 /*!< No ALT_MFP2 setting for PA.1 */

#define SYS_GPA_MFP_PA1_AD12        (1UL<<1)                /*!< GPA_MFP PA.1 setting for AD12 */
#define SYS_ALT_MFP_PA1_AD12        ((1UL<<11) | (1UL<<20)) /*!< GPA_MFP PA.1 setting for AD12 */
#define SYS_ALT_MFP2_PA1_AD12       0UL                     /*!< No ALT_MFP2 setting for PA.1 */

#define SYS_GPA_MFP_PA1_Msk         (1UL<<1)                /*!< GPA_MFP PA.1 mask */
#define SYS_ALT_MFP_PA1_Msk         ((1UL<<11) | (1UL<<20)) /*!< ALT_MFP PA.1 mask */
#define SYS_ALT_MFP2_PA1_Msk        0UL                     /*!< ALT_MFP2 PA.1 mask */

//PA.2
#define SYS_GPA_MFP_PA2_GPIO        0x00000000UL        /*!< GPA_MFP PA.2 setting for GPIO */
#define SYS_ALT_MFP_PA2_GPIO        0x00000000UL        /*!< ALT_MFP PA.2 setting for GPIO */
#define SYS_ALT_MFP2_PA2_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.2 */

#define SYS_GPA_MFP_PA2_ADC2        (1UL<<2)            /*!< GPA_MFP PA.2 setting for ADC2 */
#define SYS_ALT_MFP_PA2_ADC2        0x00000000UL        /*!< ALT_MFP PA.2 setting for ADC2 */
#define SYS_ALT_MFP2_PA2_ADC2       0UL                 /*!< No ALT_MFP2 setting for PA.2 */

#define SYS_GPA_MFP_PA2_AD11        (1UL<<2)                /*!< GPA_MFP PA.2 setting for AD11 */
#define SYS_ALT_MFP_PA2_AD11        ((1UL<<11) | (1UL<<19)) /*!< ALT_MFP PA.2 setting for AD11 */
#define SYS_ALT_MFP2_PA2_AD11       0UL                     /*!< No ALT_MFP2 setting for PA.2 */

#define SYS_GPA_MFP_PA2_Msk         (1UL<<2)                /*!< GPA_MFP PA.2 mask */
#define SYS_ALT_MFP_PA2_Msk         ((1UL<<11) | (1UL<<19)) /*!< ALT_MFP PA.2 mask */
#define SYS_ALT_MFP2_PA2_Msk        0UL                     /*!< ALT_MFP2 PA.2 mask */

//PA.3
#define SYS_GPA_MFP_PA3_GPIO        0x00000000UL        /*!< GPA_MFP PA.3 setting for GPIO */
#define SYS_ALT_MFP_PA3_GPIO        0x00000000UL        /*!< ALT_MFP PA.3 setting for GPIO */
#define SYS_ALT_MFP2_PA3_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.3 */

#define SYS_GPA_MFP_PA3_ADC3        (1UL<<3)            /*!< GPA_MFP PA.3 setting for ADC3 */
#define SYS_ALT_MFP_PA3_ADC3        0x00000000UL        /*!< ALT_MFP PA.3 setting for ADC3 */
#define SYS_ALT_MFP2_PA3_ADC3       0UL                 /*!< No ALT_MFP2 setting for PA.3 */

#define SYS_GPA_MFP_PA3_AD10        (1UL<<3)                /*!< GPA_MFP PA.3 setting for AD10 */
#define SYS_ALT_MFP_PA3_AD10        ((1UL<<11) | (1UL<<18)) /*!< ALT_MFP PA.3 setting for AD10 */
#define SYS_ALT_MFP2_PA3_AD10       0UL                     /*!< No ALT_MFP2 setting for PA.3 */

#define SYS_GPA_MFP_PA3_Msk         (1UL<<3)                /*!< GPA_MFP PA.3 mask */
#define SYS_ALT_MFP_PA3_Msk         ((1UL<<11) | (1UL<<18)) /*!< ALT_MFP PA.3 mask */
#define SYS_ALT_MFP2_PA3_Msk        0UL                     /*!< ALT_MF2 PA.3 mask */

//PA.4
#define SYS_GPA_MFP_PA4_GPIO        0x00000000UL        /*!< GPA_MFP PA.4 setting for GPIO */
#define SYS_ALT_MFP_PA4_GPIO        0x00000000UL        /*!< ALT_MFP PA.4 setting for GPIO */
#define SYS_ALT_MFP2_PA4_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.4 */

#define SYS_GPA_MFP_PA4_ADC4        (1UL<<4)            /*!< GPA_MFP PA.4 setting for ADC4 */
#define SYS_ALT_MFP_PA4_ADC4        0x00000000UL        /*!< ALT_MFP PA.4 setting for ADC4 */
#define SYS_ALT_MFP2_PA4_ADC4       0UL                 /*!< No ALT_MFP2 setting for PA.4 */

#define SYS_GPA_MFP_PA4_AD9         (1UL<<4)                /*!< GPA_MFP PA.4 setting for AD9 */
#define SYS_ALT_MFP_PA4_AD9         ((1UL<<11) | (1UL<<17)) /*!< ALT_MFP PA.4 setting for AD9 */
#define SYS_ALT_MFP2_PA4_AD9        0UL                     /*!< No ALT_MFP2 setting for PA.4 */

#define SYS_GPA_MFP_PA4_Msk         (1UL<<4)                /*!< GPA_MFP PA.4 mask */
#define SYS_ALT_MFP_PA4_Msk         ((1UL<<11) | (1UL<<17)) /*!< ALT_MFP PA.4 mask */
#define SYS_ALT_MFP2_PA4_Msk        0UL                     /*!< ALT_MFP2 PA.4 mask */

//PA.5
#define SYS_GPA_MFP_PA5_GPIO        0x00000000UL        /*!< GPA_MFP PA.5 setting for GPIO */
#define SYS_ALT_MFP_PA5_GPIO        0x00000000UL        /*!< ALT_MFP PA.5 setting for GPIO */
#define SYS_ALT_MFP2_PA5_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.5 */

#define SYS_GPA_MFP_PA5_ADC5        (1UL<<5)            /*!< GPA_MFP PA.5 setting for ADC5 */
#define SYS_ALT_MFP_PA5_ADC5        0x00000000UL        /*!< ALT_MFP PA.5 setting for ADC5 */
#define SYS_ALT_MFP2_PA5_ADC5       0UL                 /*!< No ALT_MFP2 setting for PA.5 */

#define SYS_GPA_MFP_PA5_AD8         (1UL<<5)                /*!< GPA_MFP PA.5 setting for AD8 */
#define SYS_ALT_MFP_PA5_AD8         ((1UL<<11) | (1UL<<16)) /*!< ALT_MFP PA.5 setting for AD8 */
#define SYS_ALT_MFP2_PA5_AD8        0UL                     /*!< No ALT_MFP2 setting for PA.5 */

#define SYS_GPA_MFP_PA5_Msk         (1UL<<5)                /*!< GPA_MFP PA.5 mask */
#define SYS_ALT_MFP_PA5_Msk         ((1UL<<11) | (1UL<<16)) /*!< ALT_MFP PA.5 mask */
#define SYS_ALT_MFP2_PA5_Msk        0UL                     /*!< ALT_MFP2 PA.5 mask */

//PA.6
#define SYS_GPA_MFP_PA6_GPIO        0x00000000UL        /*!< GPA_MFP PA.6 setting for GPIO */
#define SYS_ALT_MFP_PA6_GPIO        0x00000000UL        /*!< ALT_MFP PA.6 setting for GPIO */
#define SYS_ALT_MFP2_PA6_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.6 */

#define SYS_GPA_MFP_PA6_ADC6        (1UL<<6)            /*!< GPA_MFP PA.6 setting for ADC6 */
#define SYS_ALT_MFP_PA6_ADC6        0x00000000UL        /*!< ALT_MFP PA.6 setting for ADC6 */
#define SYS_ALT_MFP2_PA6_ADC6       0UL                 /*!< No ALT_MFP2 setting for PA.6 */

#define SYS_GPA_MFP_PA6_AD7         (1UL<<6)            /*!< GPA_MFP PA.6 setting for AD7 */
#define SYS_ALT_MFP_PA6_AD7         (1UL<<11)           /*!< ALT_MFP PA.6 setting for AD7 */
#define SYS_ALT_MFP2_PA6_AD7        0UL                 /*!< No ALT_MFP2 setting for PA.6 */

#define SYS_GPA_MFP_PA6_Msk         (1UL<<6)            /*!< GPA_MFP PA.6 mask */
#define SYS_ALT_MFP_PA6_Msk         (1UL<<11)           /*!< ALT_MFP PA.6 mask */
#define SYS_ALT_MFP2_PA6_Msk        0UL                 /*!< ALT_MFP2 PA.6 mask */

//PA.8
#define SYS_GPA_MFP_PA8_GPIO        0x00000000UL        /*!< GPA_MFP PA.8 setting for GPIO */
#define SYS_ALT_MFP_PA8_GPIO        0UL                 /*!< No ALT_MFP setting for PA.8 */
#define SYS_ALT_MFP2_PA8_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.8 */

#define SYS_GPA_MFP_PA8_I2C0_SDA    (1UL<<8)            /*!< GPA_MFP PA.8 setting for I2C0_SDA */
#define SYS_ALT_MFP_PA8_I2C0_SDA    0UL                 /*!< No ALT_MFP setting for PA.8 */
#define SYS_ALT_MFP2_PA8_I2C0_SDA   0UL                 /*!< No ALT_MFP2 setting for PA.8 */

#define SYS_GPA_MFP_PA8_Msk         (1UL<<8)            /*!< GPA_MFP PA.8 mask */
#define SYS_ALT_MFP_PA8_Msk         0UL                 /*!< ALT_MFP PA.8 mask */
#define SYS_ALT_MFP2_PA8_Msk        0UL                 /*!< ALT_MFP2 PA.8 mask */

//PA.9
#define SYS_GPA_MFP_PA9_GPIO        0x00000000UL        /*!< GPA_MFP PA.9 setting for GPIO */
#define SYS_ALT_MFP_PA9_GPIO        0UL                 /*!< No ALT_MFP setting for PA.9 */
#define SYS_ALT_MFP2_PA9_GPIO       0UL                 /*!< No ALT_MFP2 setting for PA.9 */

#define SYS_GPA_MFP_PA9_I2C0_SCL    (1UL<<9)            /*!< GPA_MFP PA.9 setting for I2C0_SCL */
#define SYS_ALT_MFP_PA9_I2C0_SCL    0UL                 /*!< No ALT_MFP setting for PA.9  */
#define SYS_ALT_MFP2_PA9_I2C0_SCL   0UL                 /*!< No ALT_MFP2 setting for PA.9 */

#define SYS_GPA_MFP_PA9_Msk         (1UL<<9)            /*!< GPA_MFP PA.9 mask */
#define SYS_ALT_MFP_PA9_Msk         0UL                 /*!< ALT_MFP PA.8 mask */
#define SYS_ALT_MFP2_PA9_Msk        0UL                 /*!< ALT_MFP2 PA.8 mask */

//PA.10
#define SYS_GPA_MFP_PA10_GPIO       0x00000000UL        /*!< GPA_MFP PA.10 setting for GPIO */
#define SYS_ALT_MFP_PA10_GPIO       0x00000000UL        /*!< ALT_MFP PA.10 setting for GPIO */
#define SYS_ALT_MFP2_PA10_GPIO      0UL                 /*!< No ALT_MFP2 setting for PA.10  */

#define SYS_GPA_MFP_PA10_I2C1_SDA   (1UL<<10)           /*!< GPA_MFP PA.10 setting for I2C1_SDA */
#define SYS_ALT_MFP_PA10_I2C1_SDA   0x00000000UL        /*!< ALT_MFP PA.10 setting for I2C1_SDA */
#define SYS_ALT_MFP2_PA10_I2C1_SDA  0UL                 /*!< No ALT_MFP2 setting for PA.10 */

#define SYS_GPA_MFP_PA10_nWR        (1UL<<10)           /*!< GPA_MFP PA.10 setting for nWR */
#define SYS_ALT_MFP_PA10_nWR        (1UL<<11)           /*!< ALT_MFP PA.10 setting for nWR */
#define SYS_ALT_MFP2_PA10_nWR       0UL                 /*!< No ALT_MFP2 setting for PA.10 */

#define SYS_GPA_MFP_PA10_Msk        (1UL<<10)           /*!< GPA_MFP PA.10 mask */
#define SYS_ALT_MFP_PA10_Msk        (1UL<<11)           /*!< ALT_MFP PA.10 mask */
#define SYS_ALT_MFP2_PA10_Msk       0UL                 /*!< ALT_MFP2 PA.10 mask */

//PA.11
#define SYS_GPA_MFP_PA11_GPIO       0x00000000UL        /*!< GPA_MFP PA.11 setting for GPIO */
#define SYS_ALT_MFP_PA11_GPIO       0x00000000UL        /*!< ALT_MFP PA.11 setting for GPIO */
#define SYS_ALT_MFP2_PA11_GPIO      0UL                 /*!< No ALT_MFP2 setting for PA.11 */

#define SYS_GPA_MFP_PA11_I2C1_SCL   (1UL<<11)           /*!< GPA_MFP PA.11 setting for I2C1_SCL */
#define SYS_ALT_MFP_PA11_I2C1_SCL   0x00000000UL        /*!< ALT_MFP PA.11 setting for I2C1_SCL */
#define SYS_ALT_MFP2_PA11_I2C1_SCL  0UL                 /*!< No ALT_MFP2 setting for PA.11 */

#define SYS_GPA_MFP_PA11_nRD        (1UL<<11)           /*!< GPA_MFP PA.11 setting for nRD */
#define SYS_ALT_MFP_PA11_nRD        (1UL<<11)           /*!< ALT_MFP PA.11 setting for nRD */
#define SYS_ALT_MFP2_PA11_nRD       0UL                 /*!< No ALT_MFP2 setting for PA.11 */

#define SYS_GPA_MFP_PA11_Msk        (1UL<<11)           /*!< GPA_MFP PA.11 mask */
#define SYS_ALT_MFP_PA11_Msk        (1UL<<11)           /*!< ALT_MFP PA.11 mask */
#define SYS_ALT_MFP2_PA11_Msk       0UL                 /*!< ALT_MFP2 PA.11 mask */

//PA.12
#define SYS_GPA_MFP_PA12_GPIO       0x00000000UL        /*!< GPA_MFP PA.12 setting for GPIO */
#define SYS_ALT_MFP_PA12_GPIO       0x00000000UL        /*!< ALT_MFP PA.12 setting for GPIO */
#define SYS_ALT_MFP2_PA12_GPIO      0UL                 /*!< No ALT_MFP2 setting for PA.12 */

#define SYS_GPA_MFP_PA12_PWM0       (1UL<<12)           /*!< GPA_MFP PA.12 setting for PWM0 */
#define SYS_ALT_MFP_PA12_PWM0       0x00000000UL        /*!< ALT_MFP PA.12 setting for PWM0 */
#define SYS_ALT_MFP2_PA12_PWM0      0UL                 /*!< No ALT_MFP2 setting for PA.12 */

#define SYS_GPA_MFP_PA12_AD13       (1UL<<12)               /*!< GPA_MFP PA.12 setting for AD13 */
#define SYS_ALT_MFP_PA12_AD13       ((1UL<<21) | (1UL<<11)) /*!< ALT_MFP PA.12 setting for AD13 */
#define SYS_ALT_MFP2_PA12_AD13      0UL                     /*!< No ALT_MFP2 setting for PA.12 */

#define SYS_GPA_MFP_PA12_Msk        (1UL<<12)               /*!< GPA_MFP PA.12 mask */
#define SYS_ALT_MFP_PA12_Msk        ((1UL<<21) | (1UL<<11)) /*!< ALT_MFP PA.12 mask */
#define SYS_ALT_MFP2_PA12_Msk       0UL                     /*!< ALT_MFP2 PA.12 mask */

//PA.13
#define SYS_GPA_MFP_PA13_GPIO       0x00000000UL        /*!< GPA_MFP PA.13 setting for GPIO */
#define SYS_ALT_MFP_PA13_GPIO       0x00000000UL        /*!< ALT_MFP PA.13 setting for GPIO */
#define SYS_ALT_MFP2_PA13_GPIO      0UL                 /*!< No ALT_MFP2 setting for PA.13 */

#define SYS_GPA_MFP_PA13_PWM1       (1UL<<13)           /*!< GPA_MFP PA.13 setting for PWM1 */
#define SYS_ALT_MFP_PA13_PWM1       0x00000000UL        /*!< ALT_MFP PA.13 setting for PWM1 */
#define SYS_ALT_MFP2_PA13_PWM1      0UL                 /*!< No ALT_MFP2 setting for PA.13 */

#define SYS_GPA_MFP_PA13_AD14       (1UL<<13)               /*!< GPA_MFP PA.13 setting for AD14 */
#define SYS_ALT_MFP_PA13_AD14       ((1UL<<22) | (1UL<<11)) /*!< ALT_MFP PA.13 setting for AD14 */
#define SYS_ALT_MFP2_PA13_AD14      0UL                     /*!< No ALT_MFP2 setting for PA.13 */

#define SYS_GPA_MFP_PA13_Msk        (1UL<<13)               /*!< GPA_MFP PA.13 mask */
#define SYS_ALT_MFP_PA13_Msk        ((1UL<<22) | (1UL<<11)) /*!< ALT_MFP PA.13 mask */
#define SYS_ALT_MFP2_PA13_Msk       0UL                     /*!< ALT_MFP2 PA.13 mask */

//PA.14
#define SYS_GPA_MFP_PA14_GPIO       0x00000000UL        /*!< GPA_MFP PA.14 setting for GPIO */
#define SYS_ALT_MFP_PA14_GPIO       0x00000000UL        /*!< ALT_MFP PA.14 setting for GPIO */
#define SYS_ALT_MFP2_PA14_GPIO      0UL                 /*!< No ALT_MFP2 setting for PA.14 */

#define SYS_GPA_MFP_PA14_PWM2       (1UL<<14)           /*!< GPA_MFP PA.14 setting for PWM2 */
#define SYS_ALT_MFP_PA14_PWM2       0x00000000UL        /*!< ALT_MFP PA.14 setting for PWM2 */
#define SYS_ALT_MFP2_PA14_PWM2      0UL                 /*!< No ALT_MFP2 setting for PA.14 */

#define SYS_GPA_MFP_PA14_AD15       (1UL<<14)               /*!< GPA_MFP PA.14 setting for AD15 */
#define SYS_ALT_MFP_PA14_AD15       ((1UL<<23) | (1UL<<11)) /*!< ALT_MFP PA.14 setting for AD15 */
#define SYS_ALT_MFP2_PA14_AD15      0UL                     /*!< No ALT_MFP2 setting for PA.14  */

#define SYS_GPA_MFP_PA14_Msk        (1UL<<14)               /*!< GPA_MFP PA.14 mask */
#define SYS_ALT_MFP_PA14_Msk        ((1UL<<23) | (1UL<<11)) /*!< ALT_MFP PA.14 mask */
#define SYS_ALT_MFP2_PA14_Msk       0UL                     /*!< ALT_MFP2 PA.14 mask */

//PA.15
#define SYS_GPA_MFP_PA15_GPIO       0x00000000UL        /*!< GPA_MFP PA.15 setting for GPIO */
#define SYS_ALT_MFP_PA15_GPIO       0UL                 /*!< ALT_MFP PA.15 setting for GPIO */
#define SYS_ALT_MFP2_PA15_GPIO      0UL                 /*!< No ALT_MFP2 setting for PA.15 */

#define SYS_GPA_MFP_PA15_PWM3       (1UL<<15)           /*!< GPA_MFP PA.15 setting for PWM3 */
#define SYS_ALT_MFP_PA15_PWM3       0UL                 /*!< ALT_MFP PA.15 setting for PWM3 */
#define SYS_ALT_MFP2_PA15_PWM3      0UL                 /*!< No ALT_MFP2 setting for PA.15 */

#define SYS_GPA_MFP_PA15_Msk        (1UL<<15)           /*!< GPA_MFP PA.15 mask */
#define SYS_ALT_MFP_PA15_Msk        0UL                 /*!< ALT_MFP PA.15 mask */
#define SYS_ALT_MFP2_PA15_Msk       0UL                 /*!< ALT_MFP2 PA.15 mask */

//PB.0
#define SYS_GPB_MFP_PB0_GPIO        0x00000000UL        /*!< GPB_MFP PB.0 setting for GPIO */
#define SYS_ALT_MFP_PB0_GPIO        0UL                 /*!< No ALT_MFP setting for PB.0 */
#define SYS_ALT_MFP2_PB0_GPIO       0UL                 /*!< No ALT_MFP2 setting for PB.0 */

#define SYS_GPB_MFP_PB0_UART0_RXD   (1UL<<0)            /*!< GPB_MFP PB.0 setting for UART0_RXD */
#define SYS_ALT_MFP_PB0_UART0_RXD   0UL                 /*!< No ALT_MFP setting for PB.0 */
#define SYS_ALT_MFP2_PB0_UART0_RXD  0UL                 /*!< No ALT_MFP2 setting for PB.0 */

#define SYS_GPB_MFP_PB0_Msk         (1UL<<0)            /*!< GPB_MFP PB.0 mask */
#define SYS_ALT_MFP_PB0_Msk         0UL                 /*!< ALT_MFP PB.0 mask */
#define SYS_ALT_MFP2_PB0_Msk        0UL                 /*!< ALT_MFP2 PB.0 mask */

//PB.1
#define SYS_GPB_MFP_PB1_GPIO        0x00000000UL        /*!< GPB_MFP PB.1 setting for GPIO */
#define SYS_ALT_MFP_PB1_GPIO        0UL                 /*!< No ALT_MFP setting for PB.1 */
#define SYS_ALT_MFP2_PB1_GPIO       0UL                 /*!< No ALT_MFP2 setting for PB.1 */

#define SYS_GPB_MFP_PB1_UART0_TXD   (1UL<<1)            /*!< GPB_MFP PB.1 setting for UART0_TXD */
#define SYS_ALT_MFP_PB1_UART0_TXD   0UL                 /*!< No ALT_MFP setting for PB.1 */
#define SYS_ALT_MFP2_PB1_UART0_TXD  0UL                 /*!< No ALT_MFP2 setting for PB.1 */

#define SYS_GPB_MFP_PB1_Msk         (1UL<<1)            /*!< GPB_MFP PB.1 mask */
#define SYS_ALT_MFP_PB1_Msk         0UL                 /*!< ALT_MFP PB.1 mask */
#define SYS_ALT_MFP2_PB1_Msk        0UL                 /*!< ALT_MFP2 PB.1 mask */

//PB.2
#define SYS_GPB_MFP_PB2_GPIO        0x00000000UL        /*!< GPB_MFP PB.2 setting for GPIO */
#define SYS_ALT_MFP_PB2_GPIO        0x00000000UL        /*!< ALT_MFP PB.2 setting for GPIO */
#define SYS_ALT_MFP2_PB2_GPIO       0x00000000UL        /*!< ALT_MFP2 PB.2 setting for GPIO */

#define SYS_GPB_MFP_PB2_UART0_nRTS      (1UL<<2)        /*!< GPB_MFP PB.2 setting for UART0_nRTS */
#define SYS_ALT_MFP_PB2_UART0_nRTS      0x00000000UL    /*!< ALT_MFP PB.2 setting for UART0_nRTS */
#define SYS_ALT_MFP2_PB2_UART0_nRTS     0x00000000UL    /*!< ALT_MFP2 PB.2 setting for UART0_nRTS */

#define SYS_GPB_MFP_PB2_TM2_EXT     (1UL<<2)            /*!< GPB_MFP PB.2 setting for TM2_EXT */
#define SYS_ALT_MFP_PB2_TM2_EXT     (1UL<<26)           /*!< ALT_MFP PB.2 setting for TM2_EXT */
#define SYS_ALT_MFP2_PB2_TM2_EXT    0x00000000UL        /*!< ALT_MFP2 PB.2 setting for TM2_EXT */

#define SYS_GPB_MFP_PB2_TM2         (1UL<<2)            /*!< GPB_MFP PB.2 setting for TM2 */
#define SYS_ALT_MFP_PB2_TM2         0x00000000UL        /*!< ALT_MFP PB.2 setting for TM2 */
#define SYS_ALT_MFP2_PB2_TM2        (1UL<<4)            /*!< ALT_MFP2 PB.2 setting for TM2 */

#define SYS_GPB_MFP_PB2_nWRL        (1UL<<2)                /*!< GPB_MFP PB.2 setting for nWRL */
#define SYS_ALT_MFP_PB2_nWRL        ((1UL<<13)|(1UL<<11))   /*!< ALT_MFP PB.2 setting for nWRL */
#define SYS_ALT_MFP2_PB2_nWRL       0x00000000UL            /*!< ALT_MFP2 PB.2 setting for nWRL */

#define SYS_GPB_MFP_PB2_Msk     (1UL<<2)                            /*!< GPB_MFP PB.2 mask */
#define SYS_ALT_MFP_PB2_Msk     ((1UL<<26)|(1UL<<13)|(1UL<<11))     /*!< ALT_MFP PB.2 mask */
#define SYS_ALT_MFP2_PB2_Msk    (1UL<<4)                            /*!< ALT_MFP2 PB.2 mask */

//PB.3
#define SYS_GPB_MFP_PB3_GPIO        0x00000000UL        /*!< GPB_MFP PB.3 setting for GPIO */
#define SYS_ALT_MFP_PB3_GPIO        0x00000000UL        /*!< ALT_MFP PB.3 setting for GPIO */
#define SYS_ALT_MFP2_PB3_GPIO       0x00000000UL        /*!< ALT_MFP2 PB.3 setting for GPIO */

#define SYS_GPB_MFP_PB3_UART0_nCTS    (1UL<<3)          /*!< GPB_MFP PB.3 setting for UART0_nCTS */
#define SYS_ALT_MFP_PB3_UART0_nCTS    0x00000000UL      /*!< ALT_MFP PB.3 setting for UART0_nCTS */
#define SYS_ALT_MFP2_PB3_UART0_nCTS   0x00000000UL      /*!< ALT_MFP2 PB.3 setting for UART0_nCTS */

#define SYS_GPB_MFP_PB3_TM3_EXT     (1UL<<3)            /*!< GPB_MFP PB.3 setting for TM3_EXT */
#define SYS_ALT_MFP_PB3_TM3_EXT     (1UL<<27)           /*!< ALT_MFP PB.3 setting for TM3_EXT */
#define SYS_ALT_MFP2_PB3_TM3_EXT    0x00000000UL        /*!< ALT_MFP2 PB.3 setting for TM3_EXT */

#define SYS_GPB_MFP_PB3_TM3         (1UL<<3)            /*!< GPB_MFP PB.3 setting for TM3 */
#define SYS_ALT_MFP_PB3_TM3         0x00000000UL        /*!< ALT_MFP PB.3 setting for TM3 */
#define SYS_ALT_MFP2_PB3_TM3        (1UL<<5)            /*!< ALT_MFP2 PB.3 setting for TM3 */

#define SYS_GPB_MFP_PB3_nWRH        (1UL<<3)                /*!< GPB_MFP PB.3 setting for nWRH */
#define SYS_ALT_MFP_PB3_nWRH        ((1UL<<14)|(1UL<<11))   /*!< ALT_MFP PB.3 setting for nWRH */
#define SYS_ALT_MFP2_PB3_nWRH       0x00000000UL            /*!< ALT_MFP2 PB.3 setting for nWRH */

#define SYS_GPB_MFP_PB3_Msk         (1UL<<3)                        /*!< GPB_MFP PB.3 mask */
#define SYS_ALT_MFP_PB3_Msk         ((1UL<<27)|(1UL<<14)|(1UL<<11)) /*!< ALT_MFP PB.3 mask */
#define SYS_ALT_MFP2_PB3_Msk        (1UL<<5)                        /*!< ALT_MFP2 PB.3 mask */

//PB.4
#define SYS_GPB_MFP_PB4_GPIO        0x00000000UL        /*!< GPA_MFP PB.4 setting for GPIO */
#define SYS_ALT_MFP_PB4_GPIO        0UL                 /*!< No ALT_MFP setting for PB.4 */
#define SYS_ALT_MFP2_PB4_GPIO       0UL                 /*!< No ALT_MFP2 setting for PB.4 */

#define SYS_GPB_MFP_PB4_UART1_RXD       (1UL<<4)        /*!< GPA_MFP PB.4 setting for UART1_RXD */
#define SYS_ALT_MFP_PB4_UART1_RXD       0UL             /*!< No ALT_MFP setting for PB.4 */
#define SYS_ALT_MFP2_PB4_UART1_RXD      0UL             /*!< No ALT_MFP2 setting for PB.4 */

#define SYS_GPB_MFP_PB4_Msk             (1UL<<4)        /*!< GPA_MFP PB.4 mask */
#define SYS_ALT_MFP_PB4_Msk             0UL             /*!< ALT_MFP PB.4 mask */
#define SYS_ALT_MFP2_PB4_Msk            0UL             /*!< ALT_MFP2 PB.4 mask */

//PB.5
#define SYS_GPB_MFP_PB5_GPIO            0x00000000UL    /*!< GPA_MFP PB.5 setting for GPIO */
#define SYS_ALT_MFP_PB5_GPIO            0UL             /*!< No ALT_MFP setting for PB.5 */
#define SYS_ALT_MFP2_PB5_GPIO           0UL             /*!< No ALT_MFP2 setting for PB.5 */

#define SYS_GPB_MFP_PB5_UART1_TXD       (1UL<<5)        /*!< GPA_MFP PB.5 setting for UART1_TXD */
#define SYS_ALT_MFP_PB5_UART1_TXD       0UL             /*!< No ALT_MFP setting for PB.5 */
#define SYS_ALT_MFP2_PB5_UART1_TXD      0UL             /*!< No ALT_MFP2 setting for PB.5 */

#define SYS_GPB_MFP_PB5_Msk             (1UL<<5)        /*!< GPA_MFP PB.5 mask */
#define SYS_ALT_MFP_PB5_Msk             0UL             /*!< ALT_MFP PB.5 mask */
#define SYS_ALT_MFP2_PB5_Msk            0UL             /*!< ALT_MFP2 PB.5 mask */

//PB.6
#define SYS_GPB_MFP_PB6_GPIO            0x00000000UL    /*!< GPB_MFP PB.6 setting for GPIO */
#define SYS_ALT_MFP_PB6_GPIO            0x00000000UL    /*!< ALT_MFP PB.6 setting for GPIO */
#define SYS_ALT_MFP2_PB6_GPIO           0UL             /*!< No ALT_MFP2 setting for PB.6 */

#define SYS_GPB_MFP_PB6_UART1_nRTS      (1UL<<6)        /*!< GPB_MFP PB.6 setting for UART1_nRTS */
#define SYS_ALT_MFP_PB6_UART1_nRTS      0x00000000UL    /*!< ALT__MFP PB.6 setting for UART1_nRTS */
#define SYS_ALT_MFP2_PB6_UART1_nRTS     0UL             /*!< No ALT_MFP2 setting for PB.6 */

#define SYS_GPB_MFP_PB6_ALE             (1UL<<6)        /*!< GPB_MFP PB.6 setting for ALE */
#define SYS_ALT_MFP_PB6_ALE             (1UL<<11)       /*!< ALT_MFP PB.6 setting for ALE */
#define SYS_ALT_MFP2_PB6_ALE            0UL             /*!< No ALT_MFP2 setting for PB.6 */

#define SYS_GPB_MFP_PB6_Msk             (1UL<<6)        /*!< GPB_MFP PB.6 mask */
#define SYS_ALT_MFP_PB6_Msk             (1UL<<11)       /*!< ALT_MFP PB.6 mask */
#define SYS_ALT_MFP2_PB6_Msk            0UL             /*!< ALT_MFP2 PB.6 mask */

//PB.7
#define SYS_GPB_MFP_PB7_GPIO            0x00000000UL    /*!< GPB_MFP PB.7 setting for GPIO */
#define SYS_ALT_MFP_PB7_GPIO            0x00000000UL    /*!< ALT_MFP PB.7 setting for GPIO */
#define SYS_ALT_MFP2_PB7_GPIO           0UL             /*!< No ALT_MFP2 setting for PB.7 */

#define SYS_GPB_MFP_PB7_UART1_nCTS      (1UL<<7)        /*!< GPB_MFP PB.7 setting for UART1_nCTS */
#define SYS_ALT_MFP_PB7_UART1_nCTS      0x00000000UL    /*!< ALT_MFP PB.7 setting for UART1_nCTS */
#define SYS_ALT_MFP2_PB7_UART1_nCTS     0UL             /*!< No ALT_MFP2 setting for PB.7  */

#define SYS_GPB_MFP_PB7_nCS             (1UL<<7)        /*!< GPB_MFP PB.7 setting for nCS */
#define SYS_ALT_MFP_PB7_nCS             (1UL<<11)       /*!< ALT_MFP PB.7 setting for nCS */
#define SYS_ALT_MFP2_PB7_nCS            0UL             /*!< No ALT_MFP2 setting for PB.7 */

#define SYS_GPB_MFP_PB7_Msk             (1UL<<7)        /*!< GPB_MFP PB.7 mask */
#define SYS_ALT_MFP_PB7_Msk             (1UL<<11)       /*!< ALT_MFP PB.7 mask */
#define SYS_ALT_MFP2_PB7_Msk            0UL             /*!< ALT_MFP PB.7 mask */

//PB.8
#define SYS_GPB_MFP_PB8_GPIO        0x00000000UL        /*!< GPA_MFP PB.8 setting for GPIO */
#define SYS_ALT_MFP_PB8_GPIO        0x00000000UL        /*!< ALT_MFP PB.8 setting for GPIO */
#define SYS_ALT_MFP2_PB8_GPIO       0UL                 /*!< No ALT_MFP2 setting for PB.8  */

#define SYS_GPB_MFP_PB8_TM0         (1UL<<8)            /*!< GPA_MFP PB.8 setting for TM0 */
#define SYS_ALT_MFP_PB8_TM0         0x00000000UL        /*!< ALT_MFP PB.8 setting for TM0 */
#define SYS_ALT_MFP2_PB8_TM0        0UL                 /*!< No ALT_MFP2 setting for PB.8 */

#define SYS_GPB_MFP_PB8_STADC       0x00000000UL        /*!< GPA_MFP PB.8 setting for STADC */
#define SYS_ALT_MFP_PB8_STADC       (1UL<<29)           /*!< ALT_MFP PB.8 setting for STADC */
#define SYS_ALT_MFP2_PB8_STADC      0UL                 /*!< No ALT_MFP2 setting for PB.8 */

#define SYS_GPB_MFP_PB8_CLKO        (1UL<<8)            /*!< GPA_MFP PB.8 setting for CLKO */
#define SYS_ALT_MFP_PB8_CLKO        (1UL<<29)           /*!< ALT_MFP PB.8 setting for CLKO */
#define SYS_ALT_MFP2_PB8_CLKO       0UL                 /*!< No ALT_MFP2 setting for PB.8  */

#define SYS_GPB_MFP_PB8_Msk         (1UL<<8)            /*!< GPA_MFP PB.8 mask */
#define SYS_ALT_MFP_PB8_Msk         (1UL<<29)           /*!< ALT_MFP PB.8 mask */
#define SYS_ALT_MFP2_PB8_Msk        0UL                 /*!< ALT_MFP2 PB.8 mask */

//PB.9
#define SYS_GPB_MFP_PB9_GPIO        0x00000000UL        /*!< GPB_MFP PB.9 setting for GPIO */
#define SYS_ALT_MFP_PB9_GPIO        0x00000000UL        /*!< ALT_MFP PB.9 setting for GPIO */
#define SYS_ALT_MFP2_PB9_GPIO       0UL                 /*!< No ALT_MFP2 setting for PB.9 */

#define SYS_GPB_MFP_PB9_TM1         (1UL<<9)            /*!< GPB_MFP PB.9 setting for TM1 */
#define SYS_ALT_MFP_PB9_TM1         0x00000000UL        /*!< ALT_MFP PB.9 setting for TM1 */
#define SYS_ALT_MFP2_PB9_TM1        0UL                 /*!< No ALT_MFP2 setting for PB.9 */

#define SYS_GPB_MFP_PB9_UART2_TXD       (1UL<<9)        /*!< GPB_MFP PB.9 setting for UART2_TXD */
#define SYS_ALT_MFP_PB9_UART2_TXD       (1UL<<1)        /*!< ALT_MFP PB.9 setting for UART2_TXD */
#define SYS_ALT_MFP2_PB9_UART2_TXD      0UL             /*!< No ALT_MFP2 setting for PB.9  */

#define SYS_GPB_MFP_PB9_Msk         (1UL<<9)            /*!< GPB_MFP PB.9 mask */
#define SYS_ALT_MFP_PB9_Msk         (1UL<<1)            /*!< ALT_MFP PB.9 mask */
#define SYS_ALT_MFP2_PB9_Msk        0UL                 /*!< ALT_MFP2 PB.9 mask */

//PB.10
#define SYS_GPB_MFP_PB10_GPIO       0x00000000UL        /*!< GPB_MFP PB.10 setting for GPIO */
#define SYS_ALT_MFP_PB10_GPIO       0x00000000UL        /*!< ALT_MFP PB.10 setting for GPIO */
#define SYS_ALT_MFP2_PB10_GPIO      0UL                 /*!< No ALT_MFP2 setting for PB.10 */

#define SYS_GPB_MFP_PB10_TM2        (1UL<<10)           /*!< GPB_MFP PB.10 setting for TM2 */
#define SYS_ALT_MFP_PB10_TM2        0x00000000UL        /*!< ALT_MFP PB.10 setting for TM2 */
#define SYS_ALT_MFP2_PB10_TM2       0UL                 /*!< No ALT_MFP2 setting for PB.10 */

#define SYS_GPB_MFP_PB10_UART2_RXD      (1UL<<10)       /*!< GPB_MFP PB.10 setting for UART2_RXD */
#define SYS_ALT_MFP_PB10_UART2_RXD      (1UL<<0)        /*!< ALT_MFP PB.10 setting for UART2_RXD */
#define SYS_ALT_MFP2_PB10_UART2_RXD     0UL             /*!< No ALT_MFP2 setting for PB.10 */

#define SYS_GPB_MFP_PB10_Msk        (1UL<<10)           /*!< GPB_MFP PB.10 mask */
#define SYS_ALT_MFP_PB10_Msk        (1UL<<0)            /*!< ALT_MFP PB.10 mask */
#define SYS_ALT_MFP2_PB10_Msk       0UL                /*!< ALT_MFP2 PB.10 mask */

//PB.11
#define SYS_GPB_MFP_PB11_GPIO       0x00000000UL        /*!< GPB_MFP PB.11 setting for GPIO */
#define SYS_ALT_MFP_PB11_GPIO       0x00000000UL        /*!< ALT_MFP PB.11 setting for GPIO */
#define SYS_ALT_MFP2_PB11_GPIO      0UL                 /*!< No ALT_MFP2 setting for PB.11 */

#define SYS_GPB_MFP_PB11_TM3        (1UL<<11)           /*!< GPB_MFP PB.11 setting for TM3 */
#define SYS_ALT_MFP_PB11_TM3        0x00000000UL        /*!< ALT_MFP PB.11 setting for TM3 */
#define SYS_ALT_MFP2_PB11_TM3       0UL                 /*!< No ALT_MFP2 setting for PB.11 */

#define SYS_GPB_MFP_PB11_PWM4       (1UL<<11)           /*!< GPB_MFP PB.11 setting for PWM4 */
#define SYS_ALT_MFP_PB11_PWM4       (1UL<<4)            /*!< ALT_MFP PB.11 setting for PWM4 */
#define SYS_ALT_MFP2_PB11_PWM4      0UL                 /*!< No ALT_MFP2 setting for PB.11 */

#define SYS_GPB_MFP_PB11_Msk        (1UL<<11)           /*!< GPB_MFP PB.11 mask */
#define SYS_ALT_MFP_PB11_Msk        (1UL<<4)            /*!< ALT_MFP PB.11 mask */
#define SYS_ALT_MFP2_PB11_Msk       0UL                 /*!< ALT_MFP PB.11 mask */

//PB.13
#define SYS_GPB_MFP_PB13_GPIO       0x00000000UL        /*!< GPB_MFP PB.13 setting for GPIO */
#define SYS_ALT_MFP_PB13_GPIO       0UL                 /*!< No ALT_MFP setting for PB.13 */
#define SYS_ALT_MFP2_PB13_GPIO      0UL                 /*!< No ALT_MFP2 setting for PB.13 */

#define SYS_GPB_MFP_PB13_AD1        (1UL<<13)           /*!< GPB_MFP PB.13 setting for AD1 */
#define SYS_ALT_MFP_PB13_AD1        0UL                 /*!< No ALT_MFP setting for PB.13 */
#define SYS_ALT_MFP2_PB13_AD1       0UL                 /*!< No ALT_MFP2 setting for PB.13 */

#define SYS_GPB_MFP_PB13_Msk        (1UL<<13)           /*!< GPB_MFP PB.13 mask */
#define SYS_ALT_MFP_PB13_Msk        0UL                 /*!< ALT_MFP PB.13 mask */
#define SYS_ALT_MFP2_PB13_Msk       0UL                 /*!< ALT_MFP2 PB.13 mask */

//PB.14
#define SYS_GPB_MFP_PB14_GPIO       0x00000000UL        /*!< GPB_MFP PB.14 setting for GPIO */
#define SYS_ALT_MFP_PB14_GPIO       0UL                 /*!< No ALT_MFP setting for PB.14 */
#define SYS_ALT_MFP2_PB14_GPIO      0x00000000UL        /*!< ALT_MFP2 PB.14 setting for GPIO */

#define SYS_GPB_MFP_PB14_INT0       (1UL<<14)           /*!< GPB_MFP PB.14 setting for INT0 */
#define SYS_ALT_MFP_PB14_INT0       0UL                 /*!< No ALT_MFP setting for  PB.14 */
#define SYS_ALT_MFP2_PB14_INT0      0x00000000UL        /*!< ALT_MFP2 PB.14 setting for INT0 */

#define SYS_GPB_MFP_PB14_AD0        (1UL<<14)           /*!< GPB_MFP PB.14 setting for AD0 */
#define SYS_ALT_MFP_PB14_AD0        0UL                 /*!< No ALT_MFP setting for PB.14 */
#define SYS_ALT_MFP2_PB14_AD0       (1UL<<1)            /*!< ALT_MFP2 PB.14 setting for AD0 */

#define SYS_GPB_MFP_PB14_Msk        (1UL<<14)           /*!< GPB_MFP PB.14 mask */
#define SYS_ALT_MFP_PB14_Msk        0UL                 /*!< ALT_MFP PB.14 mask */
#define SYS_ALT_MFP2_PB14_Msk       (1UL<<1)            /*!< ALT_MFP2 PB.14 mask */

//PB.15
#define SYS_GPB_MFP_PB15_GPIO       0x00000000UL        /*!< GPB_MFP PB.15 setting for GPIO */
#define SYS_ALT_MFP_PB15_GPIO       0x00000000UL        /*!< ALT_MFP PB.15 setting for GPIO */
#define SYS_ALT_MFP2_PB15_GPIO      0x00000000UL        /*!< ALT_MFP2 PB.15 setting for GPIO */

#define SYS_GPB_MFP_PB15_INT1       (1UL<<15)           /*!< GPB_MFP PB.15 setting for INT1 */
#define SYS_ALT_MFP_PB15_INT1       0x00000000UL        /*!< ALT_MFP PB.15 setting for INT1 */
#define SYS_ALT_MFP2_PB15_INT1      0x00000000UL        /*!< ALT_MFP2 PB.15 setting for INT1 */

#define SYS_GPB_MFP_PB15_TM0_EXT    (1UL<<15)           /*!< GPB_MFP PB.15 setting for TM0_EXT */
#define SYS_ALT_MFP_PB15_TM0_EXT    (1UL<<24)           /*!< ALT_MFP PB.15 setting for TM0_EXT */
#define SYS_ALT_MFP2_PB15_TM0_EXT   0x00000000UL        /*!< ALT_MFP2 PB.15 setting for TM0_EXT */

#define SYS_GPB_MFP_PB15_TM0        (1UL<<15)           /*!< GPB_MFP PB.15 setting for TM0 */
#define SYS_ALT_MFP_PB15_TM0        0x00000000UL        /*!< ALT_MFP PB.15 setting for TM0 */
#define SYS_ALT_MFP2_PB15_TM0       (1UL<<2)            /*!< ALT_MFP2 PB.15 setting for TM0 */

#define SYS_GPB_MFP_PB15_AD6        (1UL<<15)           /*!< GPB_MFP PB.15 setting for AD6 */
#define SYS_ALT_MFP_PB15_AD6        0x00000000UL        /*!< ALT_MFP PB.15 setting for AD6 */
#define SYS_ALT_MFP2_PB15_AD6       (1UL<<1)            /*!< ALT_MFP2 PB.15 setting for AD6 */

#define SYS_GPB_MFP_PB15_ADC11      0x00000000UL        /*!< GPB_MFP PB.15 setting for ADC11 */
#define SYS_ALT_MFP_PB15_ADC11      (1UL<<24)           /*!< ALT_MFP PB.15 setting for ADC11 */
#define SYS_ALT_MFP2_PB15_ADC11     0x00000000UL        /*!< ALT_MFP2 PB.15 setting for ADC11 */

#define SYS_GPB_MFP_PB15_Msk        (1UL<<15)           /*!< GPB_MFP PB.15 mask */
#define SYS_ALT_MFP_PB15_Msk        (1UL<<24)           /*!< ALT_MFP PB.15 mask */
#define SYS_ALT_MFP2_PB15_Msk       ((1UL<<1)|(1UL<<2)) /*!< ALT_MFP2 PB.15 mask */

//PC.0
#define SYS_GPC_MFP_PC0_GPIO            0x00000000UL    /*!< GPC_MFP PC.0 setting for GPIO */
#define SYS_ALT_MFP_PC0_GPIO            0UL             /*!< No ALT_MFP setting for PC.0 */
#define SYS_ALT_MFP2_PC0_GPIO           0UL             /*!< No ALT_MFP2 setting for PC.0 */

#define SYS_GPC_MFP_PC0_SPI0_SS0        (1UL<<0)        /*!< GPC_MFP PC.0 setting for SPI0_SS0 */
#define SYS_ALT_MFP_PC0_SPI0_SS0        0UL             /*!< No ALT_MFP setting for PC.0 */
#define SYS_ALT_MFP2_PC0_SPI0_SS0       0UL             /*!< No ALT_MFP2 setting for PC.0 */

#define SYS_GPC_MFP_PC0_Msk             (1UL<<0)        /*!< GPC_MFP PC.0 mask */
#define SYS_ALT_MFP_PC0_Msk             0UL             /*!< ALT_MFP PC.0 mask */
#define SYS_ALT_MFP2_PC0_Msk            0UL             /*!< ALT_MFP2 PC.0 mask */

//PC.1
#define SYS_GPC_MFP_PC1_GPIO            0x00000000UL    /*!< GPC_MFP PC.1 setting for GPIO */
#define SYS_ALT_MFP_PC1_GPIO            0UL             /*!< ALT_MFP PC.1 setting for GPIO */
#define SYS_ALT_MFP2_PC1_GPIO           0UL             /*!< No ALT_MFP2 setting for PC.1 */

#define SYS_GPC_MFP_PC1_SPI0_CLK        (1UL<<1)        /*!< GPC_MFP PC.1 setting for SPI0_CLK */
#define SYS_ALT_MFP_PC1_SPI0_CLK        0UL             /*!< ALT_MFP PC.1 setting for SPI0_CLK */
#define SYS_ALT_MFP2_PC1_SPI0_CLK       0UL             /*!< No ALT_MFP2 setting for PC.1 */

#define SYS_GPC_MFP_PC1_Msk             (1UL<<1)        /*!< GPC_MFP PC.1 mask */
#define SYS_ALT_MFP_PC1_Msk             0UL             /*!< ALT_MFP PC.1 mask */
#define SYS_ALT_MFP2_PC1_Msk            0UL             /*!< ALT_MFP2 PC.1 mask */

//PC.2
#define SYS_GPC_MFP_PC2_GPIO            0x00000000UL    /*!< GPC_MFP PC.2 setting for GPIO */
#define SYS_ALT_MFP_PC2_GPIO            0UL             /*!< No ALT_MFP setting for PC.2 */
#define SYS_ALT_MFP2_PC2_GPIO           0UL             /*!< No ALT_MFP2 setting for PC.2 */

#define SYS_GPC_MFP_PC2_SPI0_MISO0      (1UL<<2)        /*!< GPC_MFP PC.2 setting for SPI0_MISO0 */
#define SYS_ALT_MFP_PC2_SPI0_MISO0      0UL             /*!< No ALT_MFP setting for PC.2 */
#define SYS_ALT_MFP2_PC2_SPI0_MISO0     0UL             /*!< No ALT_MFP2 setting for PC.2 */

#define SYS_GPC_MFP_PC2_Msk             (1UL<<2)        /*!< GPC_MFP PC.2 mask */
#define SYS_ALT_MFP_PC2_Msk             0UL             /*!< ALT_MFP PC.2 mask */
#define SYS_ALT_MFP2_PC2_Msk            0UL             /*!< ALT_MFP2 PC.2 mask */

//PC.3
#define SYS_GPC_MFP_PC3_GPIO            0x00000000UL    /*!< GPC_MFP PC.3 setting for GPIO */
#define SYS_ALT_MFP_PC3_GPIO            0UL             /*!< No ALT_MFP setting for PC.3 */
#define SYS_ALT_MFP2_PC3_GPIO           0UL             /*!< No ALT_MFP2 setting for PC.3 */

#define SYS_GPC_MFP_PC3_SPI0_MOSI0      (1UL<<3)        /*!< GPC_MFP PC.3 setting for SPI0_MOSI0 */
#define SYS_ALT_MFP_PC3_SPI0_MOSI0      0UL             /*!< No ALT_MFP setting for PC.3 */
#define SYS_ALT_MFP2_PC3_SPI0_MOSI0     0UL             /*!< No ALT_MFP2 setting for PC.3 */

#define SYS_GPC_MFP_PC3_Msk             (1UL<<3)        /*!< GPC_MFP PC.3 mask */
#define SYS_ALT_MFP_PC3_Msk             0UL             /*!< ALT_MFP PC.3 mask */
#define SYS_ALT_MFP2_PC3_Msk            0UL             /*!< ALT_MFP2 PC.3 mask */

//PC.6
#define SYS_GPC_MFP_PC6_GPIO        0x00000000UL        /*!< GPC_MFP PC.6 setting for GPIO */
#define SYS_ALT_MFP_PC6_GPIO        0x00000000UL        /*!< ALT_MFP PC.6 setting for GPIO */
#define SYS_ALT_MFP2_PC6_GPIO       0UL                 /*!< No ALT_MFP2 setting for PC.6 */

#define SYS_GPC_MFP_PC6_ADC8        (1UL<<6)            /*!< GPC_MFP PC.6 setting for ADC8 */
#define SYS_ALT_MFP_PC6_ADC8        0x00000000UL        /*!< ALT_MFP PC.6 setting for ADC8 */
#define SYS_ALT_MFP2_PC6_ADC8       0UL                 /*!< No ALT_MFP2 setting for PC.6 */

#define SYS_GPC_MFP_PC6_AD4         (1UL<<6)            /*!< GPC_MFP PC.6 setting for AD4 */
#define SYS_ALT_MFP_PC6_AD4         (1UL<<11)           /*!< ALT_MFP PC.6 setting for AD4 */
#define SYS_ALT_MFP2_PC6_AD4        0UL                 /*!< No ALT_MFP2 setting for PC.6 */

#define SYS_GPC_MFP_PC6_Msk         (1UL<<6)            /*!< GPC_MFP PC.6 mask */
#define SYS_ALT_MFP_PC6_Msk         (1UL<<11)           /*!< ALT_MFP PC.6 mask */
#define SYS_ALT_MFP2_PC6_Msk        0UL                 /*!< ALT_MFP2 PC.6 mask */

//PC.7
#define SYS_GPC_MFP_PC7_GPIO        0x00000000UL        /*!< GPC_MFP PC.7 setting for GPIO */
#define SYS_ALT_MFP_PC7_GPIO        0x00000000UL        /*!< ALT_MFP PC.7 setting for GPIO */
#define SYS_ALT_MFP2_PC7_GPIO       0UL                 /*!< No ALT_MFP2 setting for PC.7 */

#define SYS_GPC_MFP_PC7_ADC7        (1UL<<7)            /*!< GPC_MFP PC.7 setting for ADC7 */
#define SYS_ALT_MFP_PC7_ADC7        0x00000000UL        /*!< ALT_MFP PC.7 setting for ADC7 */
#define SYS_ALT_MFP2_PC7_ADC7       0UL                 /*!< No ALT_MFP2 setting for PC.7 */

#define SYS_GPC_MFP_PC7_AD5         (1UL<<7)            /*!< GPC_MFP PC.7 setting for AD5 */
#define SYS_ALT_MFP_PC7_AD5         (1UL<<11)           /*!< ALT_MFP PC.7 setting for AD5  */
#define SYS_ALT_MFP2_PC7_AD5        0UL                 /*!< No ALT_MFP2 setting for PC.7 */

#define SYS_GPC_MFP_PC7_Msk         (1UL<<7)            /*!< GPC_MFP PC.7 mask */
#define SYS_ALT_MFP_PC7_Msk         (1UL<<11)           /*!< ALT_MFP PC.7 mask */
#define SYS_ALT_MFP2_PC7_Msk        0UL                 /*!< ALT_MFP2 PC.7 mask */

//PC.8
#define SYS_GPC_MFP_PC8_GPIO        0x00000000UL        /*!< GPC_MFP PC.8 setting for GPIO */
#define SYS_ALT_MFP_PC8_GPIO        0x00000000UL        /*!< ALT_MFP PC.8 setting for GPIO */
#define SYS_ALT_MFP2_PC8_GPIO       0UL                 /*!< No ALT_MFP2 setting for PC.8 */

#define SYS_GPC_MFP_PC8_SPI1_SS0    (1UL<<8)            /*!< GPC_MFP PC.8 setting for SPI1_SS0 */
#define SYS_ALT_MFP_PC8_SPI1_SS0    0x00000000UL        /*!< ALT_MFP PC.8 setting for SPI1_SS0 */
#define SYS_ALT_MFP2_PC8_SPI1_SS0   0UL                 /*!< No ALT_MFP2 setting for PC.8 */

#define SYS_GPC_MFP_PC8_MCLK        (1UL<<8)                /*!< GPC_MFP PC.8 setting for MCLK */
#define SYS_ALT_MFP_PC8_MCLK        ((1UL<<11) | (1UL<<12)) /*!< ALT_MFP PC.8 setting for MCLK */
#define SYS_ALT_MFP2_PC8_MCLK       0UL                     /*!< No ALT_MFP2 setting for PC.8 */

#define SYS_GPC_MFP_PC8_Msk         (1UL<<8)                /*!< GPC_MFP PC.8 mask */
#define SYS_ALT_MFP_PC8_Msk         ((1UL<<11) | (1UL<<12)) /*!< ALT_MFP PC.8 mask */
#define SYS_ALT_MFP2_PC8_Msk        0UL                     /*!< ALT_MFP2 PC.8 mask */

//PC.9
#define SYS_GPC_MFP_PC9_GPIO        0x00000000UL        /*!< GPC_MFP PC.9 setting for GPIO */
#define SYS_ALT_MFP_PC9_GPIO        0UL                 /*!< No ALT_MFP setting for PC.9 */
#define SYS_ALT_MFP2_PC9_GPIO       0UL                 /*!< No ALT_MFP2 setting for PC.9 */

#define SYS_GPC_MFP_PC9_SPI1_CLK        (1UL<<9)        /*!< GPC_MFP PC.9 setting for SPI1_CLK */
#define SYS_ALT_MFP_PC9_SPI1_CLK        0UL             /*!< No ALT_MFP setting for PC.9 */
#define SYS_ALT_MFP2_PC9_SPI1_CLK       0UL             /*!< No ALT_MFP2 setting for PC.9 */

#define SYS_GPC_MFP_PC9_Msk         (1UL<<9)            /*!< GPC_MFP PC.9 mask */
#define SYS_ALT_MFP_PC9_Msk         0UL                 /*!< ALT_MFP PC.9 mask */
#define SYS_ALT_MFP2_PC9_Msk        0UL                 /*!< ALT_MFP2 PC.9 mask */

//PC.10
#define SYS_GPC_MFP_PC10_GPIO       0x00000000UL        /*!< GPC_MFP PC.10 setting for GPIO */
#define SYS_ALT_MFP_PC10_GPIO       0UL                 /*!< No ALT_MFP setting for PC.10 */
#define SYS_ALT_MFP2_PC10_GPIO      0UL                 /*!< No ALT_MFP2 setting for PC.10 */

#define SYS_GPC_MFP_PC10_SPI1_MISO0     (1UL<<10)       /*!< GPC_MFP PC.10 setting for SPI1_MISO0 */
#define SYS_ALT_MFP_PC10_SPI1_MISO0     0UL             /*!< No ALT_MFP setting for PC.10 */
#define SYS_ALT_MFP2_PC10_SPI1_MISO0    0UL             /*!< No ALT_MFP2 setting for PC.10 */

#define SYS_GPC_MFP_PC10_Msk        (1UL<<10)            /*!< GPC_MFP PC.10 mask */
#define SYS_ALT_MFP_PC10_Msk         0UL                 /*!< ALT_MFP PC.10 mask */
#define SYS_ALT_MFP2_PC10_Msk        0UL                 /*!< ALT_MFP2 PC.10 mask */

//PC.11
#define SYS_GPC_MFP_PC11_GPIO       0x00000000UL        /*!< GPC_MFP PC.11 setting for GPIO */
#define SYS_ALT_MFP_PC11_GPIO       0UL                 /*!< No ALT_MFP setting for PC.11 */
#define SYS_ALT_MFP2_PC11_GPIO      0UL                 /*!< No ALT_MFP2 setting for PC.11 */

#define SYS_GPC_MFP_PC11_SPI1_MOSI0     (1UL<<11)       /*!< GPC_MFP PC.11 setting for SPI1_MOSI0 */
#define SYS_ALT_MFP_PC11_SPI1_MOSI0     0UL             /*!< No ALT_MFP setting for PC.11 */
#define SYS_ALT_MFP2_PC11_SPI1_MOSI0    0UL             /*!< No ALT_MFP2 setting for PC.11 */

#define SYS_GPC_MFP_PC11_Msk         (1UL<<11)          /*!< GPC_MFP PC.11 mask */
#define SYS_ALT_MFP_PC11_Msk         0UL                /*!< ALT_MFP PC.11 mask */
#define SYS_ALT_MFP2_PC11_Msk        0UL                /*!< ALT_MFP2 PC.11 mask */

//PC.14
#define SYS_GPC_MFP_PC14_GPIO       0x00000000UL        /*!< GPC_MFP PC.14 setting for GPIO */
#define SYS_ALT_MFP_PC14_GPIO       0x00000000UL        /*!< ALT_MFP PC.14 setting for GPIO */
#define SYS_ALT_MFP2_PC14_GPIO      0UL                 /*!< No ALT_MFP2 setting for PC.14 */

#define SYS_GPC_MFP_PC14_ADC10      (1UL<<14)           /*!< GPC_MFP PC.14 setting for ADC10 */
#define SYS_ALT_MFP_PC14_ADC10      0x00000000UL        /*!< ALT_MFP PC.14 setting for ADC10 */
#define SYS_ALT_MFP2_PC14_ADC10     0UL                 /*!< No ALT_MFP2 setting for PC.14 */

#define SYS_GPC_MFP_PC14_AD2        (1UL<<14)           /*!< GPC_MFP PC.14 setting for AD2 */
#define SYS_ALT_MFP_PC14_AD2        (1UL<<11)           /*!< ALT_MFP PC.14 setting for AD2 */
#define SYS_ALT_MFP2_PC14_AD2       0UL                 /*!< No ALT_MFP2 setting for PC.14 */

#define SYS_GPC_MFP_PC14_Msk        (1UL<<14)           /*!< GPC_MFP PC.14 mask */
#define SYS_ALT_MFP_PC14_Msk        (1UL<<11)           /*!< ALT_MFP PC.14 mask */
#define SYS_ALT_MFP2_PC14_Msk       0UL                 /*!< ALT_MFP2 PC.14 mask */

//PC.15
#define SYS_GPC_MFP_PC15_GPIO       0x00000000UL        /*!< GPC_MFP PC.15 setting for GPIO */
#define SYS_ALT_MFP_PC15_GPIO       0x00000000UL        /*!< ALT_MFP PC.15 setting for GPIO */
#define SYS_ALT_MFP2_PC15_GPIO      0UL                 /*!< No ALT_MFP2 setting for PC.15 */

#define SYS_GPC_MFP_PC15_ADC9       (1UL<<15)           /*!< GPC_MFP PC.15 setting for ADC9 */
#define SYS_ALT_MFP_PC15_ADC9       0x00000000UL        /*!< ALT_MFP PC.15 setting for ADC9 */
#define SYS_ALT_MFP2_PC15_ADC9      0UL                 /*!< No ALT_MFP2 setting for PC.15 */

#define SYS_GPC_MFP_PC15_AD3        (1UL<<15)           /*!< GPB_MFP PC.15 setting for AD3 */
#define SYS_ALT_MFP_PC15_AD3        (1UL<<11)           /*!< ALT_MFP PC.15 setting for AD3 */
#define SYS_ALT_MFP2_PC15_AD3       0UL                 /*!< No ALT_MFP2 setting for PC.15 */

#define SYS_GPC_MFP_PC15_Msk        (1UL<<15)           /*!< GPC_MFP PC.15 mask */
#define SYS_ALT_MFP_PC15_Msk        (1UL<<11)           /*!< ALT_MFP PC.15 mask */
#define SYS_ALT_MFP2_PC15_Msk       0UL                 /*!< ALT_MFP2 PC.15 mask */

//PE.5
#define SYS_GPE_MFP_PE5_GPIO        0x00000000UL        /*!< GPE_MFP PE.5 setting for GPIO */
#define SYS_ALT_MFP_PE5_GPIO        0x00000000UL        /*!< ALT_MFP PE.5 setting for GPIO */
#define SYS_ALT_MFP1_PE5_GPIO       0UL                 /*!< No ALT_MFP1 setting for PE.5 */
#define SYS_ALT_MFP2_PE5_GPIO       0x00000000UL        /*!< ALT_MFP2 PE.5 setting for GPIO */

#define SYS_GPE_MFP_PE5_PWM5        (1UL<<5)            /*!< GPE_MFP PE.5 setting for PWM5 */
#define SYS_ALT_MFP_PE5_PWM5        0x00000000UL        /*!< ALT_MFP PE.5 setting for PWM5 */
#define SYS_ALT_MFP2_PE5_PWM5       0x00000000UL        /*!< ALT_MFP2 PE.5 setting for PWM5 */

#define SYS_GPE_MFP_PE5_TM1_EXT     (1UL<<5)            /*!< GPE_MFP PE.5 setting for TM1_EXT */
#define SYS_ALT_MFP_PE5_TM1_EXT     (1UL<<25)           /*!< ALT_MFP PE.5 setting for TM1_EXT */
#define SYS_ALT_MFP2_PE5_TM1_EXT    0x00000000UL        /*!< ALT_MFP2 PE.5 setting for TM1_EXT */

#define SYS_GPE_MFP_PE5_TM1         (1UL<<5)            /*!< GPE_MFP PE.5 setting for TM1 */
#define SYS_ALT_MFP_PE5_TM1         0x00000000UL        /*!< ALT_MFP PE.5 setting for TM1 */
#define SYS_ALT_MFP2_PE5_TM1        (1UL<<3)            /*!< ALT_MFP2 PE.5 setting for TM1 */

#define SYS_GPE_MFP_PE5_Msk         (1UL<<5)            /*!< GPE_MFP PE.5 mask */
#define SYS_ALT_MFP_PE5_Msk         (1UL<<25)           /*!< ALT_MFP PE.5 mask */
#define SYS_ALT_MFP2_PE5_Msk        (1UL<<3)            /*!< ALT_MFP2 PE.5 mask */

//PF.0
#define SYS_GPF_MFP_PF0_GPIO        0x00000000UL        /*!< GPF_MFP PF.0 setting for GPIO */
#define SYS_ALT_MFP_PF0_GPIO        0UL                 /*!< No ALT_MFP setting for PF.0 */
#define SYS_ALT_MFP2_PF0_GPIO       0UL                 /*!< No ALT_MFP2 setting for PF.0 */

#define SYS_GPF_MFP_PF0_XT1_OUT     (1UL<<0)            /*!< GPF_MFP PF.0 setting for XT1_OUT */
#define SYS_ALT_MFP_PF0_XT1_OUT     0UL                 /*!< No ALT_MFP setting for PF.0 */
#define SYS_ALT_MFP2_PF0_XT1_OUT    0UL                 /*!< No ALT_MFP2 setting for PF.0 */

#define SYS_GPF_MFP_PF0_Msk         (1UL<<0)            /*!< GPF_MFP PF.0 mask */
#define SYS_ALT_MFP_PF0_Msk         0UL                 /*!< ALT_MFP PF.0 mask */
#define SYS_ALT_MFP2_PF0_Msk        0UL                 /*!< ALT_MFP2 PF.0 mask */

//PF.1
#define SYS_GPF_MFP_PF1_GPIO        0x00000000UL        /*!< GPF_MFP PF.1 setting for GPIO */
#define SYS_ALT_MFP_PF1_GPIO        0UL                 /*!< No ALT_MFP setting for PF.1 */
#define SYS_ALT_MFP2_PF1_GPIO       0UL                 /*!< No ALT_MFP2 setting for PF.1 */

#define SYS_GPF_MFP_PF1_XT1_IN      (1UL<<1)            /*!< GPF_MFP PF.1 setting for XT1_IN */
#define SYS_ALT_MFP_PF1_XT1_IN      0UL                 /*!< No ALT_MFP setting for PF.1 */
#define SYS_ALT_MFP2_PF1_XT1_IN     0UL                 /*!< No ALT_MFP2 setting for PF.1 */

#define SYS_GPF_MFP_PF1_Msk         (1UL<<1)            /*!< GPF_MFP PF.1 mask */
#define SYS_ALT_MFP_PF1_Msk         0UL                 /*!< ALT_MFP PF.1 mask */
#define SYS_ALT_MFP2_PF1_Msk        0UL                 /*!< ALT_MFP2 PF.1 mask */


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
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCR &= ~SYS_BODCR_BOD_LPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCR &= ~SYS_BODCR_BOD_EN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
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
  * @retval     0   System voltage is higher than BOD_VL setting or BOD_EN is 0.
  * @retval     >=1 System voltage is lower than BOD_VL setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD_EN is 0, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCR & SYS_BODCR_BOD_OUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCR &= ~SYS_BODCR_BOD_RSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCR |= SYS_BODCR_BOD_RSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCR |= SYS_BODCR_BOD_LPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCR_BOD_VL_4_4V
  *             - \ref SYS_BODCR_BOD_VL_3_7V
  *             - \ref SYS_BODCR_BOD_VL_2_7V
  *             - \ref SYS_BODCR_BOD_VL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
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
  */
#define SYS_DISABLE_LVR()               (SYS->BODCR &= ~SYS_BODCR_LVR_EN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro ensable Low-Voltage-Reset function.
  */
#define SYS_ENABLE_LVR()                (SYS->BODCR |= SYS_BODCR_LVR_EN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  */
#define SYS_DISABLE_POR()               (SYS->PORCR = 0x5AA5)

/**
  * @brief      Ensable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
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
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) (SYS->RSTSRC = (u32RstSrc))


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

/*@}*/ /* end of group Standard_Driver */


#ifdef __cplusplus
}
#endif

#endif //__SYS_H__

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
