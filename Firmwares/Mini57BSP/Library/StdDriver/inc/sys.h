/**************************************************************************//**
 * @file     sys.h
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series SYS Driver Header File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup Mini57_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_RST            (SYS_IPRST1_GPIORST_Msk)    /*!< GPIO reset is one of the SYS_ResetModule parameter  */
#define TMR0_RST            (SYS_IPRST1_TMR0RST_Msk)    /*!< TMR0 reset is one of the SYS_ResetModule parameter  */
#define TMR1_RST            (SYS_IPRST1_TMR1RST_Msk)    /*!< TMR1 reset is one of the SYS_ResetModule parameter  */
#define ECAP_RST            (SYS_IPRST1_CAPRST_Msk)     /*!< ECAP reset is one of the SYS_ResetModule parameter  */
#define PGA_RST             (SYS_IPRST1_PGARST_Msk)     /*!< PGA  reset is one of the SYS_ResetModule parameter  */
#define BPWM_RST            (SYS_IPRST1_BPWMRST_Msk)    /*!< BPWM reset is one of the SYS_ResetModule parameter  */
#define EPWM_RST            (SYS_IPRST1_EPWMRST_Msk)    /*!< EPWM reset is one of the SYS_ResetModule parameter  */
#define USCI0_RST           (SYS_IPRST1_USCI0RST_Msk)   /*!< USCI0 reset is one of the SYS_ResetModule parameter */
#define USCI1_RST           (SYS_IPRST1_USCI1RST_Msk)   /*!< USCI1 reset is one of the SYS_ResetModule parameter */
#define EADC_RST            (SYS_IPRST1_ADCRST_Msk)     /*!< EADC reset is one of the SYS_ResetModule parameter  */
#define ACMP_RST            (SYS_IPRST1_ACMPRST_Msk)    /*!< ACMP reset is one of the SYS_ResetModule parameter  */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN        (1UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_VL_2_0V       (0UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 2.0V */
#define SYS_BODCTL_BOD_VL_2_2V       (1UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */
#define SYS_BODCTL_BOD_VL_2_4V       (2UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 2.4V */
#define SYS_BODCTL_BOD_VL_2_7V       (3UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BOD_VL_3_0V       (4UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 3.0V */
#define SYS_BODCTL_BOD_VL_3_7V       (5UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BOD_VL_4_0V       (6UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 4.0V */
#define SYS_BODCTL_BOD_VL_4_3V       (7UL<<SYS_BODCTL_BODVL_Pos)        /*!< Setting Brown Out Detector Threshold Voltage as 4.3V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?
Example: If user want to set PA.0 as ADC0 and PA.1 as ADC1 in initial function,
         user can issue following command to achieve it.

         SYS->GPA_MFP  = SYS_GPA_MFP_PA0_ADC0  | SYS_GPA_MFP_PA1_ADC1;
         SYS->ALT_MFP3 = SYS_ALT_MFP3_PA0_ADC0 | SYS_ALT_MFP3_PA1_ADC1;
         SYS->ALT_MFP4 = SYS_ALT_MFP4_PA0_ADC0 | SYS_ALT_MFP4_PA1_ADC1;
*/

/********************* Bit definition of GPA_MFP register **********************/
/* PA.0 */
#define SYS_GPA_MFP_PA0_GPIO        (0x0UL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for GPIO       */
#define SYS_GPA_MFP_PA0_CLKO        (0x1UL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for CLKO       */
#define SYS_GPA_MFP_PA0_EPWM_CH0    (0x3UL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for EPWM_CH0   */
#define SYS_GPA_MFP_PA0_I2C1_SCL    (0x8UL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for I2C1_SCL   */
#define SYS_GPA_MFP_PA0_SPI0_SS     (0x9UL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for SPI0_SS    */
#define SYS_GPA_MFP_PA0_SPI1_CLK    (0xAUL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for SPI1_CLK   */
#define SYS_GPA_MFP_PA0_UART1_TXD   (0xBUL<<SYS_GPA_MFP_PA0MFP_Pos)     /*!< GPA_MFP PA0 setting for UART1_TXD  */

/* PA.1 */
#define SYS_GPA_MFP_PA1_GPIO        (0x0UL<<SYS_GPA_MFP_PA1MFP_Pos)     /*!< GPA_MFP PA1 setting for GPIO       */
#define SYS_GPA_MFP_PA1_EPWM_CH1    (0x3UL<<SYS_GPA_MFP_PA1MFP_Pos)     /*!< GPA_MFP PA1 setting for EPWM_CH1   */
#define SYS_GPA_MFP_PA1_I2C1_SDA    (0x8UL<<SYS_GPA_MFP_PA1MFP_Pos)     /*!< GPA_MFP PA1 setting for I2C1_SDA   */
#define SYS_GPA_MFP_PA1_SPI0_MISO   (0x9UL<<SYS_GPA_MFP_PA1MFP_Pos)     /*!< GPA_MFP PA1 setting for SPI0_MISO  */
#define SYS_GPA_MFP_PA1_SPI1_MOSI   (0xAUL<<SYS_GPA_MFP_PA1MFP_Pos)     /*!< GPA_MFP PA1 setting for SPI1_MOSI  */
#define SYS_GPA_MFP_PA1_UART1_RXD   (0xBUL<<SYS_GPA_MFP_PA1MFP_Pos)     /*!< GPA_MFP PA1 setting for UART1_RXD  */

/* PA.2 */
#define SYS_GPA_MFP_PA2_GPIO        (0x0UL<<SYS_GPA_MFP_PA2MFP_Pos)     /*!< GPA_MFP PA2 setting for GPIO       */
#define SYS_GPA_MFP_PA2_EPWM_CH2    (0x3UL<<SYS_GPA_MFP_PA2MFP_Pos)     /*!< GPA_MFP PA2 setting for EPWM_CH2   */
#define SYS_GPA_MFP_PA2_I2C0_SDA    (0x8UL<<SYS_GPA_MFP_PA2MFP_Pos)     /*!< GPA_MFP PA2 setting for I2C0_SDA   */
#define SYS_GPA_MFP_PA2_SPI0_MOSI   (0x9UL<<SYS_GPA_MFP_PA2MFP_Pos)     /*!< GPA_MFP PA2 setting for SPI0_MOSI  */
#define SYS_GPA_MFP_PA2_SPI1_MISO   (0xAUL<<SYS_GPA_MFP_PA2MFP_Pos)     /*!< GPA_MFP PA2 setting for SPI1_MISO  */
#define SYS_GPA_MFP_PA2_UART0_RXD   (0xBUL<<SYS_GPA_MFP_PA2MFP_Pos)     /*!< GPA_MFP PA2 setting for UART0_RXD  */

/* PA.3 */
#define SYS_GPA_MFP_PA3_GPIO        (0x0UL<<SYS_GPA_MFP_PA3MFP_Pos)     /*!< GPA_MFP PA3 setting for GPIO       */
#define SYS_GPA_MFP_PA3_EPWM_CH3    (0x3UL<<SYS_GPA_MFP_PA3MFP_Pos)     /*!< GPA_MFP PA3 setting for EPWM_CH3   */
#define SYS_GPA_MFP_PA3_I2C0_SCL    (0x8UL<<SYS_GPA_MFP_PA3MFP_Pos)     /*!< GPA_MFP PA3 setting for I2C0_SCL   */
#define SYS_GPA_MFP_PA3_SPI0_CLK    (0x9UL<<SYS_GPA_MFP_PA3MFP_Pos)     /*!< GPA_MFP PA3 setting for SPI0_CLK   */
#define SYS_GPA_MFP_PA3_SPI1_SS     (0xAUL<<SYS_GPA_MFP_PA3MFP_Pos)     /*!< GPA_MFP PA3 setting for SPI1_SS    */
#define SYS_GPA_MFP_PA3_UART0_TXD   (0xBUL<<SYS_GPA_MFP_PA3MFP_Pos)     /*!< GPA_MFP PA3 setting for UART0_TXD  */

/* PA.4 */
#define SYS_GPA_MFP_PA4_GPIO        (0x0UL<<SYS_GPA_MFP_PA4MFP_Pos)     /*!< GPA_MFP PA4 setting for GPIO       */
#define SYS_GPA_MFP_PA4_XT_IN       (0x1UL<<SYS_GPA_MFP_PA4MFP_Pos)     /*!< GPA_MFP PA4 setting for XT_IN      */
#define SYS_GPA_MFP_PA4_EPWM_CH4    (0x3UL<<SYS_GPA_MFP_PA4MFP_Pos)     /*!< GPA_MFP PA4 setting for EPWM_CH4   */

/* PA.5 */
#define SYS_GPA_MFP_PA5_GPIO        (0x0UL<<SYS_GPA_MFP_PA5MFP_Pos)     /*!< GPA_MFP PA5 setting for GPIO       */
#define SYS_GPA_MFP_PA5_XT_OUT      (0x1UL<<SYS_GPA_MFP_PA5MFP_Pos)     /*!< GPA_MFP PA5 setting for XT_OUT     */
#define SYS_GPA_MFP_PA5_EPWM_CH5    (0x3UL<<SYS_GPA_MFP_PA5MFP_Pos)     /*!< GPA_MFP PA5 setting for EPWM_CH5   */
#define SYS_GPA_MFP_PA5_ACMP0_O     (0x4UL<<SYS_GPA_MFP_PA5MFP_Pos)     /*!< GPA_MFP PA5 setting for ACMP0_O    */

/********************* Bit definition of GPB_MFP register **********************/
/* PB.0 */
#define SYS_GPB_MFP_PB0_GPIO        (0x0UL<<SYS_GPB_MFP_PB0MFP_Pos)     /*!< GPB_MFP PB0 setting for GPIO       */
#define SYS_GPB_MFP_PB0_ADC0_CH0    (0x2UL<<SYS_GPB_MFP_PB0MFP_Pos)     /*!< GPB_MFP PB0 setting for ADC0_CH0   */
#define SYS_GPB_MFP_PB0_ACMP0_P0    (0x4UL<<SYS_GPB_MFP_PB0MFP_Pos)     /*!< GPB_MFP PB0 setting for ACMP0_P0   */
#define SYS_GPB_MFP_PB0_ECAP_P0     (0x7UL<<SYS_GPB_MFP_PB0MFP_Pos)     /*!< GPB_MFP PB0 setting for ECAP_P0    */

/* PB.1 */
#define SYS_GPB_MFP_PB1_GPIO        (0x0UL<<SYS_GPB_MFP_PB1MFP_Pos)     /*!< GPB_MFP PB1 setting for GPIO       */
#define SYS_GPB_MFP_PB1_ADC0_CH1    (0x2UL<<SYS_GPB_MFP_PB1MFP_Pos)     /*!< GPB_MFP PB1 setting for ADC0_CH1   */
#define SYS_GPB_MFP_PB1_ACMP0_P1    (0x4UL<<SYS_GPB_MFP_PB1MFP_Pos)     /*!< GPB_MFP PB1 setting for ACMP0_P1   */
#define SYS_GPB_MFP_PB1_ECAP_P1     (0x7UL<<SYS_GPB_MFP_PB1MFP_Pos)     /*!< GPB_MFP PB1 setting for ECAP_P1    */

/* PB.2 */
#define SYS_GPB_MFP_PB2_GPIO        (0x0UL<<SYS_GPB_MFP_PB2MFP_Pos)     /*!< GPB_MFP PB2 setting for GPIO       */
#define SYS_GPB_MFP_PB2_ADC0_CH2    (0x2UL<<SYS_GPB_MFP_PB2MFP_Pos)     /*!< GPB_MFP PB2 setting for ADC0_CH2   */
#define SYS_GPB_MFP_PB2_BPWM_CH1    (0x3UL<<SYS_GPB_MFP_PB2MFP_Pos)     /*!< GPB_MFP PB2 setting for BPWM_CH1   */
#define SYS_GPB_MFP_PB2_ACMP0_P2    (0x4UL<<SYS_GPB_MFP_PB2MFP_Pos)     /*!< GPB_MFP PB2 setting for ACMP0_P2   */
#define SYS_GPB_MFP_PB2_ECAP_P2     (0x7UL<<SYS_GPB_MFP_PB2MFP_Pos)     /*!< GPB_MFP PB2 setting for ECAP_P2    */

/* PB.3 */
#define SYS_GPB_MFP_PB3_GPIO        (0x0UL<<SYS_GPB_MFP_PB3MFP_Pos)     /*!< GPB_MFP PB3 setting for GPIO       */
#define SYS_GPB_MFP_PB3_ACMP1_N     (0x5UL<<SYS_GPB_MFP_PB3MFP_Pos)     /*!< GPB_MFP PB3 setting for ACMP1_N    */
#define SYS_GPB_MFP_PB3_PGA_I       (0x6UL<<SYS_GPB_MFP_PB3MFP_Pos)     /*!< GPB_MFP PB3 setting for PGA_I      */
#define SYS_GPB_MFP_PB3_TM0         (0x7UL<<SYS_GPB_MFP_PB3MFP_Pos)     /*!< GPB_MFP PB3 setting for TM0        */

/* PB.4 */
#define SYS_GPB_MFP_PB4_GPIO        (0x0UL<<SYS_GPB_MFP_PB4MFP_Pos)     /*!< GPB_MFP PB4 setting for GPIO       */
#define SYS_GPB_MFP_PB4_ADC1_CH0    (0x2UL<<SYS_GPB_MFP_PB4MFP_Pos)     /*!< GPB_MFP PB4 setting for ADC1_CH0   */
#define SYS_GPB_MFP_PB4_ACMP0_N     (0x4UL<<SYS_GPB_MFP_PB4MFP_Pos)     /*!< GPB_MFP PB4 setting for ACMP0_N    */
#define SYS_GPB_MFP_PB4_TM1         (0x7UL<<SYS_GPB_MFP_PB4MFP_Pos)     /*!< GPB_MFP PB4 setting for TM1        */

/********************* Bit definition of GPC_MFP register **********************/
/* PC.0 */
#define SYS_GPC_MFP_PC0_GPIO        (0x0UL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for GPIO       */
#define SYS_GPC_MFP_PC0_ADC0_CH3    (0x2UL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for ADC0_CH3   */
#define SYS_GPC_MFP_PC0_BPWM_CH0    (0x3UL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for BPWM_CH0   */
#define SYS_GPC_MFP_PC0_ACMP1_P0    (0x5UL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for ACMP1_P0   */
#define SYS_GPC_MFP_PC0_I2C1_SCL    (0x8UL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for I2C1_SCL   */
#define SYS_GPC_MFP_PC0_SPI0_SS     (0x9UL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for SPI0_SS    */
#define SYS_GPC_MFP_PC0_SPI1_CLK    (0xAUL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PC0 setting for SPI1_CLK   */
#define SYS_GPC_MFP_PC0_UART1_TXD   (0xBUL<<SYS_GPC_MFP_PC0MFP_Pos)     /*!< GPC_MFP PA0 setting for UART1_TXD  */

/* PC.1 */
#define SYS_GPC_MFP_PC1_GPIO        (0x0UL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for GPIO       */
#define SYS_GPC_MFP_PC1_ADC0_CH4    (0x2UL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for ADC0_CH4   */
#define SYS_GPC_MFP_PC1_STADC       (0x3UL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for STADC      */
#define SYS_GPC_MFP_PC1_ACMP0_P3    (0x4UL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for ACMP0_P3   */
#define SYS_GPC_MFP_PC1_ACMP1_P1    (0x5UL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for ACMP1_P1   */
#define SYS_GPC_MFP_PC1_SPI0_MOSI   (0x9UL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for SPI0_MOSI  */
#define SYS_GPC_MFP_PC1_SPI1_MISO   (0xAUL<<SYS_GPC_MFP_PC1MFP_Pos)     /*!< GPC_MFP PC1 setting for SPI1_MISO  */

/* PC.2 */
#define SYS_GPC_MFP_PC2_GPIO        (0x0UL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for GPIO       */
#define SYS_GPC_MFP_PC2_ADC1_CH2    (0x2UL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for ADC1_CH2   */
#define SYS_GPC_MFP_PC2_BRAKE       (0x3UL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for BRAKE      */
#define SYS_GPC_MFP_PC2_CCAP_P1     (0x7UL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for CCAP_P1    */
#define SYS_GPC_MFP_PC2_I2C1_SDA    (0x8UL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for I2C1_SDA   */
#define SYS_GPC_MFP_PC2_SPI0_MISO   (0x9UL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for SPI0_MISO  */
#define SYS_GPC_MFP_PC2_SPI1_MOSI   (0xAUL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for SPI1_MOSI  */
#define SYS_GPC_MFP_PC2_UART1_RXD   (0xBUL<<SYS_GPC_MFP_PC2MFP_Pos)     /*!< GPC_MFP PC2 setting for UART1_RXD  */

/* PC.3 */
#define SYS_GPC_MFP_PC3_GPIO        (0x0UL<<SYS_GPC_MFP_PC3MFP_Pos)     /*!< GPC_MFP PC3 setting for GPIO       */
#define SYS_GPC_MFP_PC3_ACMP1_O     (0x5UL<<SYS_GPC_MFP_PC3MFP_Pos)     /*!< GPC_MFP PC3 setting for ACMP1_O    */
#define SYS_GPC_MFP_PC3_PGA_O       (0x6UL<<SYS_GPC_MFP_PC3MFP_Pos)     /*!< GPC_MFP PC3 setting for PGA_O      */
#define SYS_GPC_MFP_PC3_SPI0_CLK    (0x9UL<<SYS_GPC_MFP_PC3MFP_Pos)     /*!< GPC_MFP PC3 setting for SPI0_CLK   */
#define SYS_GPC_MFP_PC3_SPI1_SS     (0xAUL<<SYS_GPC_MFP_PC3MFP_Pos)     /*!< GPC_MFP PC3 setting for SPI1_SS    */

/* PC.4 */
#define SYS_GPC_MFP_PC4_GPIO        (0x0UL<<SYS_GPC_MFP_PC4MFP_Pos)     /*!< GPC_MFP PC4 setting for GPIO   */
#define SYS_GPC_MFP_PC4_ECAP_P3     (0x7UL<<SYS_GPC_MFP_PC4MFP_Pos)     /*!< GPC_MFP PC4 setting for ECAP_P3 */

/********************* Bit definition of GPD_MFP register **********************/
/* PD.0 */
#define SYS_GPD_MFP_PD0_nRESET      (0x0UL<<SYS_GPD_MFP_PD0MFP_Pos)     /*!< GPD_MFP PD0 setting for nRESET  */

/* PD.1 */
#define SYS_GPD_MFP_PD1_GPIO        (0x0UL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for GPIO       */
#define SYS_GPD_MFP_PD1_ICE_CLK     (0x1UL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for ICE_CLK    */
#define SYS_GPD_MFP_PD1_ACMP1_P2    (0x5UL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for ACMP1_P2   */
#define SYS_GPD_MFP_PD1_I2C0_SCL    (0x8UL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for I2C0_SCL   */
#define SYS_GPD_MFP_PD1_SPI0_CLK    (0x9UL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for SPI0_CLK   */
#define SYS_GPD_MFP_PD1_SPI1_SS     (0xAUL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for SPI1_SS    */
#define SYS_GPD_MFP_PD1_UART0_TXD   (0xBUL<<SYS_GPD_MFP_PD1MFP_Pos)     /*!< GPD_MFP PD1 setting for UART0_TXD  */

/* PD.2 */
#define SYS_GPD_MFP_PD2_GPIO        (0x0UL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for GPIO       */
#define SYS_GPD_MFP_PD2_ICE_DAT     (0x1UL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for ICE_DAT    */
#define SYS_GPD_MFP_PD2_ADC1_CH1    (0x2UL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for ADC1_CH1   */
#define SYS_GPD_MFP_PD2_CCAP_P0     (0x7UL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for CCAP_P0    */
#define SYS_GPD_MFP_PD2_I2C0_SDA    (0x8UL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for I2C0_SDA   */
#define SYS_GPD_MFP_PD2_SPI0_MOSI   (0x9UL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for SPI0_MOSI  */
#define SYS_GPD_MFP_PD2_SPI1_MISO   (0xAUL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for SPI1_MISO  */
#define SYS_GPD_MFP_PD2_UART0_RXD   (0xBUL<<SYS_GPD_MFP_PD2MFP_Pos)     /*!< GPD_MFP PD2 setting for UART0_RXD  */

/* PD.3 */
#define SYS_GPD_MFP_PD3_GPIO        (0x0UL<<SYS_GPD_MFP_PD3MFP_Pos)     /*!< GPD_MFP PD3 setting for GPIO       */
#define SYS_GPD_MFP_PD3_BPWM_CH1    (0x3UL<<SYS_GPD_MFP_PD3MFP_Pos)     /*!< GPD_MFP PD3 setting for BPWM_CH1   */
#define SYS_GPD_MFP_PD3_UART1_TXD   (0xBUL<<SYS_GPD_MFP_PD3MFP_Pos)     /*!< GPD_MFP PD3 setting for UART1_TXD  */

/*  PD.4*/
#define SYS_GPD_MFP_PD4_GPIO        (0x0UL<<SYS_GPD_MFP_PD4MFP_Pos)     /*!< GPD_MFP PD4 setting for GPIO       */
#define SYS_GPD_MFP_PD4_BPWM_CH0    (0x3UL<<SYS_GPD_MFP_PD4MFP_Pos)     /*!< GPD_MFP PD4 setting for BPWM_CH0   */
#define SYS_GPD_MFP_PD4_UART1_RXD   (0xBUL<<SYS_GPD_MFP_PD4MFP_Pos)     /*!< GPD_MFP PD4 setting for UART1_RXD  */

/* PD.5 */
#define SYS_GPD_MFP_PD5_GPIO        (0x0UL<<SYS_GPD_MFP_PD5MFP_Pos)     /*!< GPD_MFP PD5 setting for GPIO       */
#define SYS_GPD_MFP_PD5_UART0_TXD   (0xBUL<<SYS_GPD_MFP_PD5MFP_Pos)     /*!< GPD_MFP PD5 setting for UART0_TXD  */

/* PD.6 */
#define SYS_GPD_MFP_PD6_GPIO        (0x0UL<<SYS_GPD_MFP_PD6MFP_Pos)     /*!< GPD_MFP PD6 setting for GPIO       */
#define SYS_GPD_MFP_PD6_UART0_RXD   (0xBUL<<SYS_GPD_MFP_PD6MFP_Pos)     /*!< GPD_MFP PD6 setting for UART0_RXD  */


/*@}*/ /* end of group Mini57_SYS_EXPORTED_CONSTANTS */

/** @addtogroup Mini57_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/


/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          ((SYS->BODCTL & SYS_BODCTL_BODIF_Msk) >> SYS_BODCTL_BODIF_Pos)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Low Power Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Low Power Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_LPBOD()              (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Low Power Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Low Power Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_LPBOD()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Disable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BOD_VL_2_0V
  *             - \ref SYS_BODCTL_BOD_VL_2_2V
  *             - \ref SYS_BODCTL_BOD_VL_2_4V
  *             - \ref SYS_BODCTL_BOD_VL_2_7V
  *             - \ref SYS_BODCTL_BOD_VL_3_0V
  *             - \ref SYS_BODCTL_BOD_VL_3_7V
  *             - \ref SYS_BODCTL_BOD_VL_4_0V
  *             - \ref SYS_BODCTL_BOD_VL_4_3V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from Low-voltage reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-voltage reset
  * @retval     >=1 Previous reset source is from Low-voltage reset
  * @details    This macro get previous reset source is from Low-voltage reset or not.
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_LVR()               (SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_LVR()                (SYS->BODCTL |= SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including:
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_PORF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) (SYS->RSTSTS |= (u32RstSrc))

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    UNLOCKREG();
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    LOCKREG();
}

void     SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void     SYS_LockReg(void);
void     SYS_UnlockReg(void);
uint32_t SYS_ReadPDID(void);
void     SYS_ResetChip(void);
void     SYS_ResetCPU(void);
void     SYS_ResetModule(uint32_t u32ModuleIndex);
void     SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void     SYS_DisableBOD(void);


/*@}*/ /* end of group Mini57_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_SYS_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif /* __SYS_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
