/**************************************************************************//**
 * @file     SYS.h
 * @version  V3
 * $Revision: 14 $
 * $Date: 16/10/25 4:25p $
 * @brief    NUC126 series System Manager (SYS) driver header file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
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
#define PDMA_RST    ((0x0<<24)|SYS_IPRST0_PDMARST_Pos)      /*!< PDMA reset is one of the SYS_ResetModule parameter */
#define EBI_RST     ((0x0<<24)|SYS_IPRST0_EBIRST_Pos)       /*!< EBI reset is one of the SYS_ResetModule parameter */
#define HDIV_RST    ((0x0<<24)|SYS_IPRST0_HDIVRST_Pos)      /*!< HDIV reset is one of the SYS_ResetModule parameter */
#define CRC_RST     ((0x0<<24)|SYS_IPRST0_CRCRST_Pos)       /*!< CRC reset is one of the SYS_ResetModule parameter */

#define GPIO_RST    ((0x4<<24)|SYS_IPRST1_GPIORST_Pos)      /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24)|SYS_IPRST1_TMR0RST_Pos)      /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24)|SYS_IPRST1_TMR1RST_Pos)      /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24)|SYS_IPRST1_TMR2RST_Pos)      /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24)|SYS_IPRST1_TMR3RST_Pos)      /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4<<24)|SYS_IPRST1_I2C0RST_Pos)      /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x4<<24)|SYS_IPRST1_I2C1RST_Pos)      /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24)|SYS_IPRST1_SPI0RST_Pos)      /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define SPI1_RST    ((0x4<<24)|SYS_IPRST1_SPI1RST_Pos)      /*!< SPI1 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24)|SYS_IPRST1_UART0RST_Pos)     /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4<<24)|SYS_IPRST1_UART1RST_Pos)     /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART2_RST   ((0x4<<24)|SYS_IPRST1_UART2RST_Pos)     /*!< UART2 reset is one of the SYS_ResetModule parameter */
#define PWM0_RST    ((0x4<<24)|SYS_IPRST1_PWM0RST_Pos)      /*!< PWM0 reset is one of the SYS_ResetModule parameter */
#define PWM1_RST    ((0x4<<24)|SYS_IPRST1_PWM1RST_Pos)      /*!< PWM1 reset is one of the SYS_ResetModule parameter */
#define ACMP01_RST  ((0x4<<24)|SYS_IPRST1_ACMP01RST_Pos)    /*!< ACMP01 reset is one of the SYS_ResetModule parameter */
#define USBD_RST    ((0x4<<24)|SYS_IPRST1_USBDRST_Pos)      /*!< USBD reset is one of the SYS_ResetModule parameter */
#define ADC_RST     ((0x4<<24)|SYS_IPRST1_ADCRST_Pos)       /*!< ADC reset is one of the SYS_ResetModule parameter */

#define SC0_RST     ((0x8<<24)|SYS_IPRST2_SC0RST_Pos)       /*!< SC0 reset is one of the SYS_ResetModule parameter */
#define SC1_RST     ((0x8<<24)|SYS_IPRST2_SC1RST_Pos)       /*!< SC1 reset is one of the SYS_ResetModule parameter */
#define USCI0_RST   ((0x8<<24)|SYS_IPRST2_USCI0RST_Pos)     /*!< USCI0 reset is one of the SYS_ResetModule parameter */
#define USCI1_RST   ((0x8<<24)|SYS_IPRST2_USCI1RST_Pos)     /*!< USCI1 reset is one of the SYS_ResetModule parameter */
#define USCI2_RST   ((0x8<<24)|SYS_IPRST2_USCI2RST_Pos)     /*!< USCI2 reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BODVL_4_5V           (3UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.5V */
#define SYS_BODCTL_BODVL_3_7V           (2UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_2_7V           (1UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_2V           (0UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  VREFCTL constant definitions. (Write-Protection Register)                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_VREFCTL_VREF_PIN        (0x0UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = Vref pin */
#define SYS_VREFCTL_VREF_2_56V      (0x3UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 2.56V */
#define SYS_VREFCTL_VREF_2_048V     (0x7UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 2.048V */
#define SYS_VREFCTL_VREF_3_072V     (0xBUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 3.072V */
#define SYS_VREFCTL_VREF_4_096V     (0xFUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 4.096V */
#define SYS_VREFCTL_VREF_AVDD       (0x10UL<<SYS_VREFCTL_VREFCTL_Pos)   /*!< Vref = AVDD */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?

Example: If user want to set PA.2 as UART0_TXD and PA.3 as UART0_RXD in initial function,
         user can issue following command to achieve it.

         SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART0_TXD;
         SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART0_RXD;
*/

//PA.0 MFP
#define SYS_GPA_MFPL_PA0MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for GPIO           */
#define SYS_GPA_MFPL_PA0MFP_UART1_nCTS      (0x1UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART1_nCTS     */
#define SYS_GPA_MFPL_PA0MFP_UART1_TXD       (0x3UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for UART1_TXD      */
#define SYS_GPA_MFPL_PA0MFP_USCI1_CTL0      (0x4UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for USCI1_CTL0     */
#define SYS_GPA_MFPL_PA0MFP_SC0_CLK         (0x5UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for SC0_CLK        */
#define SYS_GPA_MFPL_PA0MFP_PWM1_CH5        (0x6UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for PWM1_CH5       */
#define SYS_GPA_MFPL_PA0MFP_EBI_AD0         (0x7UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for EBI_AD0        */
#define SYS_GPA_MFPL_PA0MFP_INT0            (0x8UL<<SYS_GPA_MFPL_PA0MFP_Pos)    /*!< GPA_MFPL PA0 setting for INT0           */

//PA.1 MFP
#define SYS_GPA_MFPL_PA1MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for GPIO           */
#define SYS_GPA_MFPL_PA1MFP_UART1_nRTS      (0x1UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART1_nRTS     */
#define SYS_GPA_MFPL_PA1MFP_UART1_RXD       (0x3UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for UART1_RXD      */
#define SYS_GPA_MFPL_PA1MFP_USCI1_CTL1      (0x4UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for USCI1_CTL1     */
#define SYS_GPA_MFPL_PA1MFP_SC0_DAT         (0x5UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for SC0_DAT        */
#define SYS_GPA_MFPL_PA1MFP_PWM1_CH4        (0x6UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for PWM1_CH4       */
#define SYS_GPA_MFPL_PA1MFP_EBI_AD1         (0x7UL<<SYS_GPA_MFPL_PA1MFP_Pos)    /*!< GPA_MFPL PA1 setting for EBI_AD1        */

//PA.2 MFP
#define SYS_GPA_MFPL_PA2MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for GPIO           */
#define SYS_GPA_MFPL_PA2MFP_UART0_TXD       (0x2UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for UART0_TXD      */
#define SYS_GPA_MFPL_PA2MFP_UART0_nCTS      (0x3UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for UART0_nCTS     */
#define SYS_GPA_MFPL_PA2MFP_I2C0_SDA        (0x4UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for I2C0_SDA       */
#define SYS_GPA_MFPL_PA2MFP_SC0_RST         (0x5UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for SC0_RST        */
#define SYS_GPA_MFPL_PA2MFP_PWM1_CH3        (0x6UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for PWM1_CH3       */
#define SYS_GPA_MFPL_PA2MFP_EBI_AD2         (0x7UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for EBI_AD2        */
#define SYS_GPA_MFPL_PA2MFP_USCI1_CTL0      (0x8UL<<SYS_GPA_MFPL_PA2MFP_Pos)    /*!< GPA_MFPL PA2 setting for USCI1_CTL0     */

//PA.3 MFP
#define SYS_GPA_MFPL_PA3MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for GPIO           */
#define SYS_GPA_MFPL_PA3MFP_UART0_RXD       (0x2UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for UART0_RXD      */
#define SYS_GPA_MFPL_PA3MFP_UART0_nRTS      (0x3UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for UART0_nRTS     */
#define SYS_GPA_MFPL_PA3MFP_I2C0_SCL        (0x4UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for I2C0_SCL       */
#define SYS_GPA_MFPL_PA3MFP_SC0_PWR         (0x5UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for SC0_PWR        */
#define SYS_GPA_MFPL_PA3MFP_PWM1_CH2        (0x6UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for PWM1_CH2       */
#define SYS_GPA_MFPL_PA3MFP_EBI_AD3         (0x7UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for EBI_AD3        */
#define SYS_GPA_MFPL_PA3MFP_USCI1_CLK       (0x8UL<<SYS_GPA_MFPL_PA3MFP_Pos)    /*!< GPA_MFPL PA3 setting for USCI1_CLK      */

//PA.4 MFP
#define SYS_GPA_MFPL_PA4MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for GPIO           */
#define SYS_GPA_MFPL_PA4MFP_SPI1_SS         (0x2UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for SPI1_SS        */
#define SYS_GPA_MFPL_PA4MFP_T3_EXT          (0x3UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for T3_EXT         */
#define SYS_GPA_MFPL_PA4MFP_EBI_AD4         (0x7UL<<SYS_GPA_MFPL_PA4MFP_Pos)    /*!< GPA_MFPL PA4 setting for EBI_AD4        */

//PA.5 MFP
#define SYS_GPA_MFPL_PA5MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for GPIO           */
#define SYS_GPA_MFPL_PA5MFP_SPI1_MOSI       (0x2UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for SPI1_MOSI      */
#define SYS_GPA_MFPL_PA5MFP_T2_EXT          (0x3UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for T2_EXT         */
#define SYS_GPA_MFPL_PA5MFP_TM_BRAKE3       (0x6UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for TM_BRAKE3      */
#define SYS_GPA_MFPL_PA5MFP_EBI_AD5         (0x7UL<<SYS_GPA_MFPL_PA5MFP_Pos)    /*!< GPA_MFPL PA5 setting for EBI_AD5        */

//PA.6 MFP
#define SYS_GPA_MFPL_PA6MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for GPIO           */
#define SYS_GPA_MFPL_PA6MFP_SPI1_MISO       (0x2UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for SPI1_MISO      */
#define SYS_GPA_MFPL_PA6MFP_T1_EXT          (0x3UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for T1_EXT         */
#define SYS_GPA_MFPL_PA6MFP_TM_BRAKE2       (0x6UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for TM_BRAKE2      */
#define SYS_GPA_MFPL_PA6MFP_EBI_AD6         (0x7UL<<SYS_GPA_MFPL_PA6MFP_Pos)    /*!< GPA_MFPL PA6 setting for EBI_AD6        */

//PA.7 MFP
#define SYS_GPA_MFPL_PA7MFP_GPIO            (0x0UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for GPIO           */
#define SYS_GPA_MFPL_PA7MFP_SPI1_CLK        (0x2UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for SPI1_CLK       */
#define SYS_GPA_MFPL_PA7MFP_T0_EXT          (0x3UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for T0_EXT         */
#define SYS_GPA_MFPL_PA7MFP_TM_BRAKE1       (0x6UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for TM_BRAKE1      */
#define SYS_GPA_MFPL_PA7MFP_EBI_AD7         (0x7UL<<SYS_GPA_MFPL_PA7MFP_Pos)    /*!< GPA_MFPL PA7 setting for EBI_AD7        */

//PA.8 MFP
#define SYS_GPA_MFPH_PA8MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for GPIO           */
#define SYS_GPA_MFPH_PA8MFP_CLKO            (0x1UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for CLKO           */
#define SYS_GPA_MFPH_PA8MFP_I2C1_SCL        (0x2UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for I2C1_SCL       */
#define SYS_GPA_MFPH_PA8MFP_UART1_TXD       (0x3UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for UART1_TXD      */
#define SYS_GPA_MFPH_PA8MFP_SC0_PWR         (0x4UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for SC0_PWR        */
#define SYS_GPA_MFPH_PA8MFP_SC1_RST         (0x5UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for SC1_RST        */
#define SYS_GPA_MFPH_PA8MFP_TM_BRAKE0       (0x6UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for TM_BRAKE0      */
#define SYS_GPA_MFPH_PA8MFP_PWM0_BRAKE0     (0x7UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for PWM0_BRAKE0    */
#define SYS_GPA_MFPH_PA8MFP_T1              (0x8UL<<SYS_GPA_MFPH_PA8MFP_Pos)    /*!< GPA_MFPH PA8 setting for T1             */

//PA.9 MFP
#define SYS_GPA_MFPH_PA9MFP_GPIO            (0x0UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for GPIO           */
#define SYS_GPA_MFPH_PA9MFP_SPI1_I2SMCLK    (0x1UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for SPI1_I2SMCLK   */
#define SYS_GPA_MFPH_PA9MFP_I2C1_SDA        (0x2UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for I2C1_SDA       */
#define SYS_GPA_MFPH_PA9MFP_UART1_RXD       (0x3UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for UART1_RXD      */
#define SYS_GPA_MFPH_PA9MFP_SC0_RST         (0x4UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for SC0_RST        */
#define SYS_GPA_MFPH_PA9MFP_SC1_PWR         (0x5UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for SC1_PWR        */
#define SYS_GPA_MFPH_PA9MFP_TM_BRAKE1       (0x6UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for TM_BRAKE1      */
#define SYS_GPA_MFPH_PA9MFP_PWM1_BRAKE1     (0x7UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for PWM1_BRAKE1    */
#define SYS_GPA_MFPH_PA9MFP_T2              (0x8UL<<SYS_GPA_MFPH_PA9MFP_Pos)    /*!< GPA_MFPH PA9 setting for T2             */

//PA.10 MFP
#define SYS_GPA_MFPH_PA10MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for GPIO          */
#define SYS_GPA_MFPH_PA10MFP_UART1_nCTS     (0x3UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for UART1_nCTS    */
#define SYS_GPA_MFPH_PA10MFP_SC1_DAT        (0x5UL<<SYS_GPA_MFPH_PA10MFP_Pos)   /*!< GPA_MFPH PA10 setting for SC1_DAT       */

//PA.11 MFP
#define SYS_GPA_MFPH_PA11MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for GPIO          */
#define SYS_GPA_MFPH_PA11MFP_UART1_nRTS     (0x3UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for UART1_nRTS    */
#define SYS_GPA_MFPH_PA11MFP_SC1_CLK        (0x5UL<<SYS_GPA_MFPH_PA11MFP_Pos)   /*!< GPA_MFPH PA11 setting for SC1_CLK       */

//PA.12 MFP
#define SYS_GPA_MFPH_PA12MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for GPIO          */
#define SYS_GPA_MFPH_PA12MFP_SPI1_I2SMCLK   (0x2UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for SPI1_I2SMCLK  */
#define SYS_GPA_MFPH_PA12MFP_UART2_RXD      (0x3UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for UART2_RXD     */
#define SYS_GPA_MFPH_PA12MFP_UART1_RXD      (0x4UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for UART1_RXD     */
#define SYS_GPA_MFPH_PA12MFP_TM_BRAKE2      (0x6UL<<SYS_GPA_MFPH_PA12MFP_Pos)   /*!< GPA_MFPH PA12 setting for TM_BRAKE2     */

//PA.13 MFP
#define SYS_GPA_MFPH_PA13MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for GPIO          */
#define SYS_GPA_MFPH_PA13MFP_UART2_TXD      (0x3UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for UART2_TXD     */
#define SYS_GPA_MFPH_PA13MFP_UART1_TXD      (0x4UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA13 setting for UART1_TXD     */
#define SYS_GPA_MFPH_PA13MFP_TM_BRAKE3      (0x6UL<<SYS_GPA_MFPH_PA13MFP_Pos)   /*!< GPA_MFPH PA12 setting for TM_BRAKE3     */

//PA.14 MFP
#define SYS_GPA_MFPH_PA14MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for GPIO          */
#define SYS_GPA_MFPH_PA14MFP_UART2_nCTS     (0x3UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for UART2_nCTS    */
#define SYS_GPA_MFPH_PA14MFP_USCI1_CTL1     (0x4UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for USCI1_CTL1    */
#define SYS_GPA_MFPH_PA14MFP_T2    (0x6UL<<SYS_GPA_MFPH_PA14MFP_Pos)   /*!< GPA_MFPH PA14 setting for T2   */

//PA.15 MFP
#define SYS_GPA_MFPH_PA15MFP_GPIO           (0x0UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for GPIO          */
#define SYS_GPA_MFPH_PA15MFP_UART2_nRTS     (0x3UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for UART2_nRTS    */
#define SYS_GPA_MFPH_PA15MFP_USCI1_CLK      (0x4UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for USCI1_CLK     */
#define SYS_GPA_MFPH_PA15MFP_T3    (0x6UL<<SYS_GPA_MFPH_PA15MFP_Pos)   /*!< GPA_MFPH PA15 setting for T3   */

//PB.0 MFP
#define SYS_GPB_MFPL_PB0MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for GPIO           */
#define SYS_GPB_MFPL_PB0MFP_ADC_CH0         (0x1UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for ADC_CH0        */
#define SYS_GPB_MFPL_PB0MFP_BOD_SYS         (0x2UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for BOD_SYS        */
#define SYS_GPB_MFPL_PB0MFP_UART2_RXD       (0x3UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for UART2_RXD      */
#define SYS_GPB_MFPL_PB0MFP_T2              (0x4UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for T2             */
#define SYS_GPB_MFPL_PB0MFP_USCI1_DAT0      (0x6UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for USCI1_DAT0     */
#define SYS_GPB_MFPL_PB0MFP_EBI_nWRL        (0x7UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for EBI_nWRL       */
#define SYS_GPB_MFPL_PB0MFP_INT1            (0x8UL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for INT1           */
#define SYS_GPB_MFPL_PB0MFP_T1_EXT          (0xAUL<<SYS_GPB_MFPL_PB0MFP_Pos)    /*!< GPB_MFPL PB0 setting for T1_EXT         */

//PB.1 MFP
#define SYS_GPB_MFPL_PB1MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for GPIO           */
#define SYS_GPB_MFPL_PB1MFP_ADC_CH1         (0x1UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for ADC_CH1        */
#define SYS_GPB_MFPL_PB1MFP_BOD_SYS         (0x2UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for BOD_SYS        */
#define SYS_GPB_MFPL_PB1MFP_UART2_TXD       (0x3UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for UART2_TXD      */
#define SYS_GPB_MFPL_PB1MFP_T3              (0x4UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for T3             */
#define SYS_GPB_MFPL_PB1MFP_SC0_RST         (0x5UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for SC0_RST        */
#define SYS_GPB_MFPL_PB1MFP_PWM0_SYNC_OUT   (0x6UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for PWM0_SYNC_OUT  */
#define SYS_GPB_MFPL_PB1MFP_EBI_nWRH        (0x7UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for EBI_nWRH       */
#define SYS_GPB_MFPL_PB1MFP_USCI1_DAT1      (0x8UL<<SYS_GPB_MFPL_PB1MFP_Pos)    /*!< GPB_MFPL PB1 setting for USCI1_DAT1     */

//PB.2 MFP
#define SYS_GPB_MFPL_PB2MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for GPIO           */
#define SYS_GPB_MFPL_PB2MFP_ADC_CH2         (0x1UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for ADC_CH2        */
#define SYS_GPB_MFPL_PB2MFP_SPI0_CLK        (0x2UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for SPI0_CLK       */
#define SYS_GPB_MFPL_PB2MFP_SPI1_CLK        (0x3UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for SPI1_CLK       */
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD       (0x4UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for UART1_RXD      */
#define SYS_GPB_MFPL_PB2MFP_SC0_CD          (0x5UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for SC0_CD         */
#define SYS_GPB_MFPL_PB2MFP_TM_BRAKE0       (0x6UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for TM_BRAKE0      */
#define SYS_GPB_MFPL_PB2MFP_EBI_nCS0        (0x7UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for EBI_nCS0       */
#define SYS_GPB_MFPL_PB2MFP_USCI0_DAT0      (0x8UL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for USCI0_DAT0     */
#define SYS_GPB_MFPL_PB2MFP_T2_EXT          (0xAUL<<SYS_GPB_MFPL_PB2MFP_Pos)    /*!< GPB_MFPL PB2 setting for T2_EXT         */

//PB.3 MFP
#define SYS_GPB_MFPL_PB3MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for GPIO           */
#define SYS_GPB_MFPL_PB3MFP_ADC_CH3         (0x1UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for ADC_CH3        */
#define SYS_GPB_MFPL_PB3MFP_SPI0_MISO0      (0x2UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for SPI0_MISO0     */
#define SYS_GPB_MFPL_PB3MFP_SPI1_MISO       (0x3UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for SPI1_MISO      */
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD       (0x4UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for UART1_TXD      */
#define SYS_GPB_MFPL_PB3MFP_TM_BRAKE1       (0x6UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for TM_BRAKE1      */
#define SYS_GPB_MFPL_PB3MFP_EBI_ALE         (0x7UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for EBI_ALE        */
#define SYS_GPB_MFPL_PB3MFP_USCI0_DAT1      (0x8UL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for USCI0_DAT1     */
#define SYS_GPB_MFPL_PB3MFP_T0_EXT          (0xAUL<<SYS_GPB_MFPL_PB3MFP_Pos)    /*!< GPB_MFPL PB3 setting for T0_EXT         */

//PB.4 MFP
#define SYS_GPB_MFPL_PB4MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for GPIO           */
#define SYS_GPB_MFPL_PB4MFP_ADC_CH4         (0x1UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for ADC_CH4        */
#define SYS_GPB_MFPL_PB4MFP_SPI0_SS         (0x2UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for SPI0_SS        */
#define SYS_GPB_MFPL_PB4MFP_SPI1_SS         (0x3UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for SPI1_SS        */
#define SYS_GPB_MFPL_PB4MFP_UART1_nCTS      (0x4UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for UART1_nCTS     */
#define SYS_GPB_MFPL_PB4MFP_ACMP0_N         (0x5UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for ACMP0_N        */
#define SYS_GPB_MFPL_PB4MFP_SC1_CD          (0x6UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for SC1_CD         */
#define SYS_GPB_MFPL_PB4MFP_EBI_AD7         (0x7UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for EBI_AD7        */
#define SYS_GPB_MFPL_PB4MFP_USCI0_CTL1      (0x8UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for USCI0_CTL1     */
#define SYS_GPB_MFPL_PB4MFP_UART2_RXD       (0x9UL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for UART2_RXD      */
#define SYS_GPB_MFPL_PB4MFP_T1_EXT          (0xAUL<<SYS_GPB_MFPL_PB4MFP_Pos)    /*!< GPB_MFPL PB4 setting for T1_EXT         */

//PB.5 MFP
#define SYS_GPB_MFPL_PB5MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for GPIO           */
#define SYS_GPB_MFPL_PB5MFP_ADC_CH13        (0x1UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for ADC_CH13       */
#define SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0      (0x2UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for SPI0_MOSI0     */
#define SYS_GPB_MFPL_PB5MFP_SPI1_MOSI       (0x3UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for SPI1_MOSI      */
#define SYS_GPB_MFPL_PB5MFP_ACMP0_P2        (0x5UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for ACMP0_P2       */
#define SYS_GPB_MFPL_PB5MFP_SC1_RST         (0x6UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for SC1_RST        */
#define SYS_GPB_MFPL_PB5MFP_EBI_AD6         (0x7UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for EBI_AD6        */
#define SYS_GPB_MFPL_PB5MFP_UART2_RXD       (0x9UL<<SYS_GPB_MFPL_PB5MFP_Pos)    /*!< GPB_MFPL PB5 setting for UART2_RXD      */

//PB.6 MFP
#define SYS_GPB_MFPL_PB6MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for GPIO           */
#define SYS_GPB_MFPL_PB6MFP_ADC_CH14        (0x1UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for ADC_CH14       */
#define SYS_GPB_MFPL_PB6MFP_SPI0_MISO0      (0x2UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for SPI0_MISO0     */
#define SYS_GPB_MFPL_PB6MFP_SPI1_MISO       (0x3UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for SPI1_MISO      */
#define SYS_GPB_MFPL_PB6MFP_ACMP0_P1        (0x5UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for ACMP0_P1       */
#define SYS_GPB_MFPL_PB6MFP_SC1_PWR         (0x6UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for SC1_PWR        */
#define SYS_GPB_MFPL_PB6MFP_EBI_AD5         (0x7UL<<SYS_GPB_MFPL_PB6MFP_Pos)    /*!< GPB_MFPL PB6 setting for EBI_AD5        */

//PB.7 MFP
#define SYS_GPB_MFPL_PB7MFP_GPIO            (0x0UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for GPIO           */
#define SYS_GPB_MFPL_PB7MFP_ADC_CH15        (0x1UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for ADC_CH15       */
#define SYS_GPB_MFPL_PB7MFP_SPI0_CLK        (0x2UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for SPI0_CLK       */
#define SYS_GPB_MFPL_PB7MFP_SPI1_CLK        (0x3UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for SPI1_CLK       */
#define SYS_GPB_MFPL_PB7MFP_USCI2_CTL1      (0x4UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for USCI2_CTL1     */
#define SYS_GPB_MFPL_PB7MFP_ACMP0_P0        (0x5UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for ACMP0_P0       */
#define SYS_GPB_MFPL_PB7MFP_SC1_DAT         (0x6UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for SC1_DAT        */
#define SYS_GPB_MFPL_PB7MFP_EBI_AD4         (0x7UL<<SYS_GPB_MFPL_PB7MFP_Pos)    /*!< GPB_MFPL PB7 setting for EBI_AD4        */

//PB.8 MFP
#define SYS_GPB_MFPH_PB8MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for GPIO           */
#define SYS_GPB_MFPH_PB8MFP_ADC_CH5         (0x1UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for ADC_CH5        */
#define SYS_GPB_MFPH_PB8MFP_UART1_nRTS      (0x4UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for UART1_nRTS     */
#define SYS_GPB_MFPH_PB8MFP_TM_BRAKE2       (0x5UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for TM_BRAKE2      */
#define SYS_GPB_MFPH_PB8MFP_PWM0_CH2        (0x6UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for PWM0_CH2       */
#define SYS_GPB_MFPH_PB8MFP_USCI0_CTL0      (0x8UL<<SYS_GPB_MFPH_PB8MFP_Pos)    /*!< GPB_MFPH PB8 setting for USCI0_CTL0     */

//PB.9 MFP
#define SYS_GPB_MFPH_PB9MFP_GPIO            (0x0UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for GPIO           */
#define SYS_GPB_MFPH_PB9MFP_ADC_CH6         (0x1UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for ADC_CH6        */
#define SYS_GPB_MFPH_PB9MFP_USCI0_CLK       (0x8UL<<SYS_GPB_MFPH_PB9MFP_Pos)    /*!< GPB_MFPH PB9 setting for USCI0_CLK      */

//PB.10 MFP
#define SYS_GPB_MFPH_PB10MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for GPIO          */
#define SYS_GPB_MFPH_PB10MFP_ADC_CH7        (0x1UL<<SYS_GPB_MFPH_PB10MFP_Pos)   /*!< GPB_MFPH PB10 setting for ADC_CH7       */

//PB.11 MFP
#define SYS_GPB_MFPH_PB11MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for GPIO          */
#define SYS_GPB_MFPH_PB11MFP_ADC_CH8        (0x1UL<<SYS_GPB_MFPH_PB11MFP_Pos)   /*!< GPB_MFPH PB11 setting for ADC_CH8       */

//PB.12 MFP
#define SYS_GPB_MFPH_PB12MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for GPIO          */
#define SYS_GPB_MFPH_PB12MFP_PWM1_CH1       (0x6UL<<SYS_GPB_MFPH_PB12MFP_Pos)   /*!< GPB_MFPH PB12 setting for PWM1_CH1      */

//PB.13MFP
#define SYS_GPB_MFPH_PB13MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for GPIO          */
#define SYS_GPB_MFPH_PB13MFP_ADC_CH10       (0x1UL<<SYS_GPB_MFPH_PB13MFP_Pos)   /*!< GPB_MFPH PB13 setting for ADC_CH10      */

//PB.14 MFP
#define SYS_GPB_MFPH_PB14MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for GPIO          */
#define SYS_GPB_MFPH_PB14MFP_ADC_CH11       (0x1UL<<SYS_GPB_MFPH_PB14MFP_Pos)   /*!< GPB_MFPH PB14 setting for ADC_CH11      */

//PB.15 MFP
#define SYS_GPB_MFPH_PB15MFP_GPIO           (0x0UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for GPIO          */
#define SYS_GPB_MFPH_PB15MFP_ADC_CH12       (0x1UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for ADC_CH12      */
#define SYS_GPB_MFPH_PB15MFP_ACMP0_P3       (0x5UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for ACMP0_P3      */
#define SYS_GPB_MFPH_PB15MFP_EBI_nCS1       (0x7UL<<SYS_GPB_MFPH_PB15MFP_Pos)   /*!< GPB_MFPH PB15 setting for EBI_nCS1      */

//PC.0 MFP
#define SYS_GPC_MFPL_PC0MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for GPIO           */
#define SYS_GPC_MFPL_PC0MFP_SC0_DAT         (0x1UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for SC0_DAT        */
#define SYS_GPC_MFPL_PC0MFP_SPI0_CLK        (0x2UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for SPI0_CLK       */
#define SYS_GPC_MFPL_PC0MFP_UART2_nCTS      (0x3UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for UART2_nCTS     */
#define SYS_GPC_MFPL_PC0MFP_USCI0_DAT0      (0x4UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for USCI0_DAT0     */
#define SYS_GPC_MFPL_PC0MFP_ACMP0_WLAT      (0x5UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for ACMP0_WLAT     */
#define SYS_GPC_MFPL_PC0MFP_PWM0_CH0        (0x6UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for PWM0_CH0       */
#define SYS_GPC_MFPL_PC0MFP_EBI_AD8         (0x7UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for EBI_AD8        */
#define SYS_GPC_MFPL_PC0MFP_INT2            (0x8UL<<SYS_GPC_MFPL_PC0MFP_Pos)    /*!< GPC_MFPL PC0 setting for INT2           */

//PC.1 MFP
#define SYS_GPC_MFPL_PC1MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for GPIO           */
#define SYS_GPC_MFPL_PC1MFP_CLKO            (0x1UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for CLKO           */
#define SYS_GPC_MFPL_PC1MFP_SC0_CLK         (0x2UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for SC0_CLK        */
#define SYS_GPC_MFPL_PC1MFP_UART2_nRTS      (0x3UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for UART2_nRTS     */
#define SYS_GPC_MFPL_PC1MFP_USCI0_DAT1      (0x4UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for USCI0_DAT1     */
#define SYS_GPC_MFPL_PC1MFP_ACMP1_WLAT      (0x5UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for ACMP1_WLAT     */
#define SYS_GPC_MFPL_PC1MFP_PWM0_CH1        (0x6UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for PWM0_CH1       */
#define SYS_GPC_MFPL_PC1MFP_EBI_AD9         (0x7UL<<SYS_GPC_MFPL_PC1MFP_Pos)    /*!< GPC_MFPL PC1 setting for EBI_AD9        */

//PC.2 MFP
#define SYS_GPC_MFPL_PC2MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for GPIO           */
#define SYS_GPC_MFPL_PC2MFP_SC0_RST         (0x1UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for SC0_RST        */
#define SYS_GPC_MFPL_PC2MFP_SPI0_SS         (0x2UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for SPI0_SS        */
#define SYS_GPC_MFPL_PC2MFP_UART2_TXD       (0x3UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for UART2_TXD      */
#define SYS_GPC_MFPL_PC2MFP_USCI0_CTL1      (0x4UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for USCI0_CTL1     */
#define SYS_GPC_MFPL_PC2MFP_ACMP1_O         (0x5UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for ACMP1_O        */
#define SYS_GPC_MFPL_PC2MFP_PWM0_CH2        (0x6UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for PWM0_CH2       */
#define SYS_GPC_MFPL_PC2MFP_EBI_AD10        (0x7UL<<SYS_GPC_MFPL_PC2MFP_Pos)    /*!< GPC_MFPL PC2 setting for EBI_AD10       */

//PC.3 MFP
#define SYS_GPC_MFPL_PC3MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for GPIO           */
#define SYS_GPC_MFPL_PC3MFP_SC0_PWR         (0x1UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for SC0_PWR        */
#define SYS_GPC_MFPL_PC3MFP_SPI0_MOSI       (0x2UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for SPI0_MOSI      */
#define SYS_GPC_MFPL_PC3MFP_UART2_RXD       (0x3UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for UART2_RXD      */
#define SYS_GPC_MFPL_PC3MFP_USCI0_CTL0      (0x5UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for USCI0_CTL0     */
#define SYS_GPC_MFPL_PC3MFP_PWM0_CH3        (0x6UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for PWM0_CH3       */
#define SYS_GPC_MFPL_PC3MFP_EBI_AD11        (0x7UL<<SYS_GPC_MFPL_PC3MFP_Pos)    /*!< GPC_MFPL PC3 setting for EBI_AD11       */

//PC.4 MFP
#define SYS_GPC_MFPL_PC4MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for GPIO           */
#define SYS_GPC_MFPL_PC4MFP_SC0_CD          (0x1UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for SC0_CD         */
#define SYS_GPC_MFPL_PC4MFP_SPI0_MISO       (0x2UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for SPI0_MISO      */
#define SYS_GPC_MFPL_PC4MFP_I2C1_SCL        (0x3UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for I2C1_SCL       */
#define SYS_GPC_MFPL_PC4MFP_USCI0_CLK       (0x5UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for USCI0_CLK      */
#define SYS_GPC_MFPL_PC4MFP_PWM0_CH4        (0x6UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for PWM0_CH4       */
#define SYS_GPC_MFPL_PC4MFP_EBI_AD12        (0x7UL<<SYS_GPC_MFPL_PC4MFP_Pos)    /*!< GPC_MFPL PC4 setting for EBI_AD12       */

//PC.5 MFP
#define SYS_GPC_MFPL_PC5MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for GPIO           */
#define SYS_GPC_MFPL_PC5MFP_SPI0_I2SMCLK    (0x2UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for SPI0_I2SMCLK   */
#define SYS_GPC_MFPL_PC5MFP_I2C1_SDA        (0x3UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for I2C1_SDA       */
#define SYS_GPC_MFPL_PC5MFP_USCI0_DAT0      (0x4UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for USCI0_DAT0     */
#define SYS_GPC_MFPL_PC5MFP_PWM0_CH5        (0x6UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for PWM0_CH5       */
#define SYS_GPC_MFPL_PC5MFP_EBI_AD13        (0x7UL<<SYS_GPC_MFPL_PC5MFP_Pos)    /*!< GPC_MFPL PC5 setting for EBI_AD13       */

//PC.6 MFP
#define SYS_GPC_MFPL_PC6MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for GPIO           */
#define SYS_GPC_MFPL_PC6MFP_USCI0_DAT1      (0x4UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for USCI0_DAT1     */
#define SYS_GPC_MFPL_PC6MFP_ACMP1_O         (0x5UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for ACMP1_O        */
#define SYS_GPC_MFPL_PC6MFP_PWM1_CH0        (0x6UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for PWM1_CH0       */
#define SYS_GPC_MFPL_PC6MFP_EBI_AD14        (0x7UL<<SYS_GPC_MFPL_PC6MFP_Pos)    /*!< GPC_MFPL PC6 setting for EBI_AD14       */

//PC.7 MFP
#define SYS_GPC_MFPL_PC7MFP_GPIO            (0x0UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for GPIO           */
#define SYS_GPC_MFPL_PC7MFP_USCI0_CTL1      (0x4UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for USCI0_CTL1     */
#define SYS_GPC_MFPL_PC7MFP_PWM1_CH1        (0x6UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for PWM1_CH1       */
#define SYS_GPC_MFPL_PC7MFP_EBI_AD15        (0x7UL<<SYS_GPC_MFPL_PC7MFP_Pos)    /*!< GPC_MFPL PC7 setting for EBI_AD15       */

//PC.8 MFP
#define SYS_GPC_MFPH_PC8MFP_GPIO            (0x0UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for GPIO           */
#define SYS_GPC_MFPH_PC8MFP_ADC_CH16        (0x1UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for ADC_CH16       */
#define SYS_GPC_MFPH_PC8MFP_UART0_nRTS      (0x3UL<<SYS_GPC_MFPH_PC8MFP_Pos)    /*!< GPC_MFPH PC8 setting for UART0_nRTS     */

//PC.9 MFP
#define SYS_GPC_MFPH_PC9MFP_GPIO            (0x0UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for GPIO           */
#define SYS_GPC_MFPH_PC9MFP_SPI0_I2SMCLK    (0x2UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for SPI0_I2SMCLK   */
#define SYS_GPC_MFPH_PC9MFP_I2C1_SCL        (0x3UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for I2C1_SCL       */
#define SYS_GPC_MFPH_PC9MFP_USCI2_CTL1      (0x4UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for USCI2_CTL1     */
#define SYS_GPC_MFPH_PC9MFP_PWM1_CH0        (0x6UL<<SYS_GPC_MFPH_PC9MFP_Pos)    /*!< GPC_MFPH PC9 setting for PWM1_CH0       */

//PC.10 MFP
#define SYS_GPC_MFPH_PC10MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for GPIO          */
#define SYS_GPC_MFPH_PC10MFP_SPI0_MOSI      (0x2UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for SPI0_MOSI     */
#define SYS_GPC_MFPH_PC10MFP_I2C1_SDA       (0x3UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for I2C1_SDA      */
#define SYS_GPC_MFPH_PC10MFP_USCI2_DAT1     (0x4UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for USCI2_DAT1    */
#define SYS_GPC_MFPH_PC10MFP_PWM1_CH1       (0x6UL<<SYS_GPC_MFPH_PC10MFP_Pos)   /*!< GPC_MFPH PC10 setting for PWM1_CH1      */

//PC.11 MFP
#define SYS_GPC_MFPH_PC11MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for GPIO          */
#define SYS_GPC_MFPH_PC11MFP_SPI0_MISO      (0x2UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for SPI0_MISO     */
#define SYS_GPC_MFPH_PC11MFP_USCI2_CLK      (0x4UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for USCI2_CLK     */
#define SYS_GPC_MFPH_PC11MFP_PWM1_CH2       (0x6UL<<SYS_GPC_MFPH_PC11MFP_Pos)   /*!< GPC_MFPH PC11 setting for PWM1_CH2      */

//PC.12 MFP
#define SYS_GPC_MFPH_PC12MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for GPIO          */
#define SYS_GPC_MFPH_PC12MFP_SPI0_CLK       (0x2UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for SPI0_CLK      */
#define SYS_GPC_MFPH_PC12MFP_USCI2_CTL0     (0x4UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for USCI2_CTL0    */
#define SYS_GPC_MFPH_PC12MFP_PWM1_CH3       (0x6UL<<SYS_GPC_MFPH_PC12MFP_Pos)   /*!< GPC_MFPH PC12 setting for PWM1_CH3      */

//PC.13 MFP
#define SYS_GPC_MFPH_PC13MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC13MFP_Pos)   /*!< GPC_MFPH PC13 setting for GPIO          */
#define SYS_GPC_MFPH_PC13MFP_SPI0_SS        (0x2UL<<SYS_GPC_MFPH_PC13MFP_Pos)   /*!< GPC_MFPH PC13 setting for SPI0_SS       */
#define SYS_GPC_MFPH_PC13MFP_USCI2_DAT0     (0x4UL<<SYS_GPC_MFPH_PC13MFP_Pos)   /*!< GPC_MFPH PC13 setting for USCI2_DAT0    */
#define SYS_GPC_MFPH_PC13MFP_PWM1_CH4       (0x6UL<<SYS_GPC_MFPH_PC13MFP_Pos)   /*!< GPC_MFPH PC13 setting for PWM1_CH4      */

//PC.14 MFP
#define SYS_GPC_MFPH_PC14MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for GPIO          */
#define SYS_GPC_MFPH_PC14MFP_PWM1_CH5       (0x6UL<<SYS_GPC_MFPH_PC14MFP_Pos)   /*!< GPC_MFPH PC14 setting for PWM1_CH5      */

//PC.15 MFP
#define SYS_GPC_MFPH_PC15MFP_GPIO           (0x0UL<<SYS_GPC_MFPH_PC15MFP_Pos)   /*!< GPC_MFPH PC15 setting for GPIO          */
#define SYS_GPC_MFPH_PC15MFP_PWM1_CH0       (0x6UL<<SYS_GPC_MFPH_PC15MFP_Pos)   /*!< GPC_MFPH PC15 setting for PWM1_CH0      */

//PD.0 MFP
#define SYS_GPD_MFPL_PD0MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for GPIO           */
#define SYS_GPD_MFPL_PD0MFP_SPI0_I2SMCLK    (0x1UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for SPI0_I2SMCLK   */
#define SYS_GPD_MFPL_PD0MFP_SPI1_I2SMCLK    (0x2UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for SPI1_I2SMCLK   */
#define SYS_GPD_MFPL_PD0MFP_UART0_RXD       (0x3UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for UART0_RXD      */
#define SYS_GPD_MFPL_PD0MFP_USCI2_CTL0      (0x4UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for USCI2_CTL0     */
#define SYS_GPD_MFPL_PD0MFP_ACMP1_N         (0x5UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for ACMP1_N        */
#define SYS_GPD_MFPL_PD0MFP_SC1_CLK         (0x6UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for SC1_CLK        */
#define SYS_GPD_MFPL_PD0MFP_INT3            (0x8UL<<SYS_GPD_MFPL_PD0MFP_Pos)    /*!< GPD_MFPL PD0 setting for INT3           */

//PD.1 MFP
#define SYS_GPD_MFPL_PD1MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for GPIO           */
#define SYS_GPD_MFPL_PD1MFP_ADC_CH19        (0x1UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for ADC_CH19       */
#define SYS_GPD_MFPL_PD1MFP_PWM0_SYNC_IN    (0x2UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for PWM0_SYNC_IN   */
#define SYS_GPD_MFPL_PD1MFP_UART0_TXD       (0x3UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for UART0_TXD      */
#define SYS_GPD_MFPL_PD1MFP_USCI2_CLK       (0x4UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for USCI2_CLK      */
#define SYS_GPD_MFPL_PD1MFP_ACMP1_P2        (0x5UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for ACMP1_P2       */
#define SYS_GPD_MFPL_PD1MFP_T0              (0x6UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for T0             */
#define SYS_GPD_MFPL_PD1MFP_EBI_nRD         (0x7UL<<SYS_GPD_MFPL_PD1MFP_Pos)    /*!< GPD_MFPL PD1 setting for EBI_nRD        */

//PD.2 MFP
#define SYS_GPD_MFPL_PD2MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for GPIO           */
#define SYS_GPD_MFPL_PD2MFP_STADC           (0x1UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for STADC          */
#define SYS_GPD_MFPL_PD2MFP_T0_EXT          (0x3UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for T0_EXT         */
#define SYS_GPD_MFPL_PD2MFP_USCI2_DAT0      (0x4UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for USCI2_DAT0     */
#define SYS_GPD_MFPL_PD2MFP_ACMP1_P1        (0x5UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for ACMP1_P1       */
#define SYS_GPD_MFPL_PD2MFP_PWM0_BRAKE0     (0x6UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for PWM0_BRAKE0    */
#define SYS_GPD_MFPL_PD2MFP_EBI_nWR         (0x7UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for EBI_nWR        */
#define SYS_GPD_MFPL_PD2MFP_INT0            (0x8UL<<SYS_GPD_MFPL_PD2MFP_Pos)    /*!< GPD_MFPL PD2 setting for INT0           */

//PD.3 MFP
#define SYS_GPD_MFPL_PD3MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for GPIO           */
#define SYS_GPD_MFPL_PD3MFP_T2              (0x1UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for T2             */
#define SYS_GPD_MFPL_PD3MFP_SPI0_I2SMCLK    (0x2UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for SPI0_I2SMCLK   */
#define SYS_GPD_MFPL_PD3MFP_T1_EXT          (0x3UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for T1_EXT         */
#define SYS_GPD_MFPL_PD3MFP_USCI2_DAT1      (0x4UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for USCI2_DAT1     */
#define SYS_GPD_MFPL_PD3MFP_ACMP1_P0        (0x5UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for ACMP1_P0       */
#define SYS_GPD_MFPL_PD3MFP_PWM0_BRAKE1     (0x6UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for PWM0_BRAKE1    */
#define SYS_GPD_MFPL_PD3MFP_EBI_MCLK        (0x7UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for EBI_MCLK       */
#define SYS_GPD_MFPL_PD3MFP_INT1            (0x8UL<<SYS_GPD_MFPL_PD3MFP_Pos)    /*!< GPD_MFPL PD3 setting for INT1           */

//PD.4 MFP
#define SYS_GPD_MFPL_PD4MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for GPIO           */
#define SYS_GPD_MFPL_PD4MFP_SPI1_CLK        (0x2UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for SPI1_CLK       */
#define SYS_GPD_MFPL_PD4MFP_I2C0_SDA        (0x3UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for I2C0_SDA       */
#define SYS_GPD_MFPL_PD4MFP_UART2_nRTS      (0x4UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for UART2_nRTS     */
#define SYS_GPD_MFPL_PD4MFP_PWM0_BRAKE0     (0x5UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for PWM0_BRAKE0    */
#define SYS_GPD_MFPL_PD4MFP_T0              (0x6UL<<SYS_GPD_MFPL_PD4MFP_Pos)    /*!< GPD_MFPL PD4 setting for T0             */

//PD.5 MFP
#define SYS_GPD_MFPL_PD5MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for GPIO           */
#define SYS_GPD_MFPL_PD5MFP_CLKO            (0x1UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for CLKO           */
#define SYS_GPD_MFPL_PD5MFP_SPI1_MISO       (0x2UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for SPI1_MISO      */
#define SYS_GPD_MFPL_PD5MFP_I2C0_SCL        (0x3UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for I2C0_SCL       */
#define SYS_GPD_MFPL_PD5MFP_UART2_nCTS      (0x4UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for UART2_nCTS     */
#define SYS_GPD_MFPL_PD5MFP_PWM0_BRAKE1     (0x5UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for PWM0_BRAKE1    */
#define SYS_GPD_MFPL_PD5MFP_T1              (0x6UL<<SYS_GPD_MFPL_PD5MFP_Pos)    /*!< GPD_MFPL PD5 setting for T1             */

//PD.6 MFP
#define SYS_GPD_MFPL_PD6MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for GPIO           */
#define SYS_GPD_MFPL_PD6MFP_CLKO            (0x1UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for CLKO           */
#define SYS_GPD_MFPL_PD6MFP_SPI1_SS         (0x2UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for SPI1_SS        */
#define SYS_GPD_MFPL_PD6MFP_UART0_RXD       (0x3UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for UART0_RXD      */
#define SYS_GPD_MFPL_PD6MFP_UART2_TXD       (0x4UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for UART2_TXD      */
#define SYS_GPD_MFPL_PD6MFP_ACMP0_O         (0x5UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for ACMP0_O        */
#define SYS_GPD_MFPL_PD6MFP_PWM0_CH5        (0x6UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for PWM0_CH5       */
#define SYS_GPD_MFPL_PD6MFP_EBI_nWR         (0x7UL<<SYS_GPD_MFPL_PD6MFP_Pos)    /*!< GPD_MFPL PD6 setting for EBI_nWR        */

//PD.7 MFP
#define SYS_GPD_MFPL_PD7MFP_GPIO            (0x0UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for GPIO           */
#define SYS_GPD_MFPL_PD7MFP_USCI1_CTL1      (0x1UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for USCI1_CTL1     */
#define SYS_GPD_MFPL_PD7MFP_SPI0_I2SMCLK    (0x2UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for SPI0_I2SMCLK   */
#define SYS_GPD_MFPL_PD7MFP_PWM0_SYNC_IN    (0x3UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for PWM0_SYNC_IN   */
#define SYS_GPD_MFPL_PD7MFP_T1              (0x4UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for T1             */
#define SYS_GPD_MFPL_PD7MFP_ACMP0_O         (0x5UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for ACMP0_O        */
#define SYS_GPD_MFPL_PD7MFP_PWM0_CH5        (0x6UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for PWM0_CH5       */
#define SYS_GPD_MFPL_PD7MFP_EBI_nRD         (0x7UL<<SYS_GPD_MFPL_PD7MFP_Pos)    /*!< GPD_MFPL PD7 setting for EBI_nRD        */

//PD.8 MFP
#define SYS_GPD_MFPH_PD8MFP_GPIO            (0x0UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for GPIO           */
#define SYS_GPD_MFPH_PD8MFP_ADC_CH17        (0x1UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for ADC_CH17       */
#define SYS_GPD_MFPH_PD8MFP_UART0_nCTS      (0x3UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for UART0_nCTS     */
#define SYS_GPD_MFPH_PD8MFP_USCI2_CTL1      (0x4UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for USCI2_CTL1     */
#define SYS_GPD_MFPH_PD8MFP_T2              (0x6UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for T2             */
#define SYS_GPD_MFPH_PD8MFP_EBI_nCS0        (0x7UL<<SYS_GPD_MFPH_PD8MFP_Pos)    /*!< GPD_MFPH PD8 setting for EBI_nCS0       */

//PD.9 MFP
#define SYS_GPD_MFPH_PD9MFP_GPIO            (0x0UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for GPIO           */
#define SYS_GPD_MFPH_PD9MFP_ADC_CH18        (0x1UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for ADC_CH18       */
#define SYS_GPD_MFPH_PD9MFP_UART0_RXD       (0x3UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for UART0_RXD      */
#define SYS_GPD_MFPH_PD9MFP_USCI2_CTL0      (0x4UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for USCI2_CTL0     */
#define SYS_GPD_MFPH_PD9MFP_ACMP1_P3        (0x5UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for ACMP1_P3       */
#define SYS_GPD_MFPH_PD9MFP_T3              (0x6UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for T3             */
#define SYS_GPD_MFPH_PD9MFP_EBI_ALE         (0x7UL<<SYS_GPD_MFPH_PD9MFP_Pos)    /*!< GPD_MFPH PD9 setting for EBI_ALE        */

//PD.10 MFP
#define SYS_GPD_MFPH_PD10MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for GPIO          */
#define SYS_GPD_MFPH_PD10MFP_T2             (0x4UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for T2            */
#define SYS_GPD_MFPH_PD10MFP_USCI2_DAT0     (0x5UL<<SYS_GPD_MFPH_PD10MFP_Pos)   /*!< GPD_MFPH PD10 setting for USCI2_DAT0    */

//PD.11 MFP
#define SYS_GPD_MFPH_PD11MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for GPIO          */
#define SYS_GPD_MFPH_PD11MFP_T3             (0x4UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for T3            */
#define SYS_GPD_MFPH_PD11MFP_USCI2_DAT1     (0x5UL<<SYS_GPD_MFPH_PD11MFP_Pos)   /*!< GPD_MFPH PD11 setting for USCI2_DAT1    */

//PD.12 MFP
#define SYS_GPD_MFPH_PD12MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for GPIO          */
#define SYS_GPD_MFPH_PD12MFP_USCI1_CTL0     (0x1UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for USCI1_CTL0    */
#define SYS_GPD_MFPH_PD12MFP_SPI1_SS        (0x2UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for SPI1_SS       */
#define SYS_GPD_MFPH_PD12MFP_UART0_TXD      (0x3UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for UART0_TXD     */
#define SYS_GPD_MFPH_PD12MFP_PWM1_CH0       (0x6UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for PWM1_CH0      */
#define SYS_GPD_MFPH_PD12MFP_EBI_ADR16      (0x7UL<<SYS_GPD_MFPH_PD12MFP_Pos)   /*!< GPD_MFPH PD12 setting for EBI_ADR16     */

//PD.13 MFP
#define SYS_GPD_MFPH_PD13MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for GPIO          */
#define SYS_GPD_MFPH_PD13MFP_USCI1_DAT1     (0x1UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for USCI1_DAT1    */
#define SYS_GPD_MFPH_PD13MFP_SPI1_MOSI      (0x2UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for SPI1_MOSI     */
#define SYS_GPD_MFPH_PD13MFP_UART0_RXD      (0x3UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for UART0_RXD     */
#define SYS_GPD_MFPH_PD13MFP_PWM1_CH1       (0x6UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for PWM1_CH1      */
#define SYS_GPD_MFPH_PD13MFP_EBI_ADR17      (0x7UL<<SYS_GPD_MFPH_PD13MFP_Pos)   /*!< GPD_MFPH PD13 setting for EBI_ADR17     */

//PD.14 MFP
#define SYS_GPD_MFPH_PD14MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD14MFP_Pos)   /*!< GPD_MFPH PD14 setting for GPIO          */
#define SYS_GPD_MFPH_PD14MFP_USCI1_DAT0     (0x1UL<<SYS_GPD_MFPH_PD14MFP_Pos)   /*!< GPD_MFPH PD14 setting for USCI1_DAT0    */
#define SYS_GPD_MFPH_PD14MFP_SPI1_MISO      (0x2UL<<SYS_GPD_MFPH_PD14MFP_Pos)   /*!< GPD_MFPH PD14 setting for SPI1_MISO     */
#define SYS_GPD_MFPH_PD14MFP_UART0_nCTS     (0x3UL<<SYS_GPD_MFPH_PD14MFP_Pos)   /*!< GPD_MFPH PD14 setting for UART0_nCTS    */
#define SYS_GPD_MFPH_PD14MFP_PWM1_CH2       (0x6UL<<SYS_GPD_MFPH_PD14MFP_Pos)   /*!< GPD_MFPH PD14 setting for PWM1_CH2      */
#define SYS_GPD_MFPH_PD14MFP_EBI_ADR18      (0x7UL<<SYS_GPD_MFPH_PD14MFP_Pos)   /*!< GPD_MFPH PD14 setting for EBI_ADR18     */

//PD.15 MFP
#define SYS_GPD_MFPH_PD15MFP_GPIO           (0x0UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for GPIO          */
#define SYS_GPD_MFPH_PD15MFP_USCI1_CLK      (0x1UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for USCI1_CLK     */
#define SYS_GPD_MFPH_PD15MFP_SPI1_CLK       (0x2UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for SPI1_CLK      */
#define SYS_GPD_MFPH_PD15MFP_UART0_nRTS     (0x3UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for UART0_nRTS    */
#define SYS_GPD_MFPH_PD15MFP_PWM1_CH3       (0x6UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for PWM1_CH3      */
#define SYS_GPD_MFPH_PD15MFP_EBI_ADR19      (0x7UL<<SYS_GPD_MFPH_PD15MFP_Pos)   /*!< GPD_MFPH PD15 setting for EBI_ADR19     */

//PE.0 MFP
#define SYS_GPE_MFPL_PE0MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for GPIO           */
#define SYS_GPE_MFPL_PE0MFP_SPI0_CLK        (0x2UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for SPI0_CLK       */
#define SYS_GPE_MFPL_PE0MFP_I2C1_SDA        (0x3UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for I2C1_SDA       */
#define SYS_GPE_MFPL_PE0MFP_T2_EXT          (0x4UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for T2_EXT         */
#define SYS_GPE_MFPL_PE0MFP_SC0_CD          (0x5UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for SC0_CD         */
#define SYS_GPE_MFPL_PE0MFP_PWM0_CH0        (0x6UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for PWM0_CH0       */
#define SYS_GPE_MFPL_PE0MFP_EBI_nCS1        (0x7UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for EBI_nCS1       */
#define SYS_GPE_MFPL_PE0MFP_INT4            (0x8UL<<SYS_GPE_MFPL_PE0MFP_Pos)    /*!< GPE_MFPL PE0 setting for INT4           */

//PE.1 MFP
#define SYS_GPE_MFPL_PE1MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for GPIO           */
#define SYS_GPE_MFPL_PE1MFP_T3_EXT          (0x3UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for T3_EXT         */
#define SYS_GPE_MFPL_PE1MFP_SC0_CD          (0x5UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for SC0_CD         */
#define SYS_GPE_MFPL_PE1MFP_PWM0_CH1        (0x6UL<<SYS_GPE_MFPL_PE1MFP_Pos)    /*!< GPE_MFPL PE1 setting for PWM0_CH1       */

//PE.2 MFP
#define SYS_GPE_MFPL_PE2MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for GPIO           */
#define SYS_GPE_MFPL_PE2MFP_ADC_CH9         (0x1UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for ADC_CH9        */
#define SYS_GPE_MFPL_PE2MFP_UART1_nRTS      (0x4UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for UART1_nRTS     */
#define SYS_GPE_MFPL_PE2MFP_TM_BRAKE3       (0x5UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for TM_BRAKE3      */
#define SYS_GPE_MFPL_PE2MFP_PWM0_CH2        (0x6UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for PWM0_CH2       */
#define SYS_GPE_MFPL_PE2MFP_USCI0_CTL0      (0x8UL<<SYS_GPE_MFPL_PE2MFP_Pos)    /*!< GPE_MFPL PE2 setting for USCI0_CTL0     */

//PE.3 MFP
#define SYS_GPE_MFPL_PE3MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for GPIO           */
#define SYS_GPE_MFPL_PE3MFP_SPI1_MOSI       (0x2UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for SPI1_MOSI      */
#define SYS_GPE_MFPL_PE3MFP_UART2_RXD       (0x4UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for UART2_RXD      */
#define SYS_GPE_MFPL_PE3MFP_PWM0_CH3        (0x6UL<<SYS_GPE_MFPL_PE3MFP_Pos)    /*!< GPE_MFPL PE3 setting for PWM0_CH3       */

//PE.4 MFP
#define SYS_GPE_MFPL_PE4MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for GPIO           */
#define SYS_GPE_MFPL_PE4MFP_I2C0_SCL        (0x2UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for I2C0_SCL       */
#define SYS_GPE_MFPL_PE4MFP_I2C1_SCL        (0x3UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for I2C1_SCL       */
#define SYS_GPE_MFPL_PE4MFP_USCI0_CTL0      (0x4UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for USCI0_CTL0     */
#define SYS_GPE_MFPL_PE4MFP_SC0_PWR         (0x5UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for SC0_PWR        */
#define SYS_GPE_MFPL_PE4MFP_PWM1_BRAKE0     (0x6UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for PWM1_BRAKE0    */
#define SYS_GPE_MFPL_PE4MFP_EBI_nCS0        (0x7UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for EBI_nCS0       */
#define SYS_GPE_MFPL_PE4MFP_INT0            (0x8UL<<SYS_GPE_MFPL_PE4MFP_Pos)    /*!< GPE_MFPL PE4 setting for INT0           */

//PE.5 MFP
#define SYS_GPE_MFPL_PE5MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for GPIO           */
#define SYS_GPE_MFPL_PE5MFP_I2C0_SDA        (0x2UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for I2C0_SDA       */
#define SYS_GPE_MFPL_PE5MFP_I2C1_SDA        (0x3UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for I2C1_SDA       */
#define SYS_GPE_MFPL_PE5MFP_USCI0_CLK       (0x4UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for USCI0_CLK      */
#define SYS_GPE_MFPL_PE5MFP_SC0_RST         (0x5UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for SC0_RST        */
#define SYS_GPE_MFPL_PE5MFP_PWM1_BRAKE1     (0x6UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for PWM1_BRAKE1    */
#define SYS_GPE_MFPL_PE5MFP_EBI_ALE         (0x7UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for EBI_ALE        */
#define SYS_GPE_MFPL_PE5MFP_INT1            (0x8UL<<SYS_GPE_MFPL_PE5MFP_Pos)    /*!< GPE_MFPL PE5 setting for INT1           */

//PE.6 MFP
#define SYS_GPE_MFPL_PE6MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for GPIO           */
#define SYS_GPE_MFPL_PE6MFP_ICE_CLK         (0x1UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for ICE_CLK        */
#define SYS_GPE_MFPL_PE6MFP_I2C0_SCL        (0x2UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for I2C0_SCL       */
#define SYS_GPE_MFPL_PE6MFP_UART0_RXD       (0x3UL<<SYS_GPE_MFPL_PE6MFP_Pos)    /*!< GPE_MFPL PE6 setting for UART0_RXD      */

//PE.7 MFP
#define SYS_GPE_MFPL_PE7MFP_GPIO            (0x0UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for GPIO           */
#define SYS_GPE_MFPL_PE7MFP_ICE_DAT         (0x1UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for ICE_DAT        */
#define SYS_GPE_MFPL_PE7MFP_I2C0_SDA        (0x2UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for I2C0_SDA       */
#define SYS_GPE_MFPL_PE7MFP_UART0_TXD       (0x3UL<<SYS_GPE_MFPL_PE7MFP_Pos)    /*!< GPE_MFPL PE7 setting for UART0_TXD      */

//PE.8 MFP
#define SYS_GPE_MFPH_PE8MFP_GPIO            (0x0UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for GPIO           */
#define SYS_GPE_MFPH_PE8MFP_UART1_TXD       (0x1UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for UART1_TXD      */
#define SYS_GPE_MFPH_PE8MFP_T0              (0x3UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for T0             */
#define SYS_GPE_MFPH_PE8MFP_I2C1_SCL        (0x4UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for I2C1_SCL       */
#define SYS_GPE_MFPH_PE8MFP_SC0_PWR         (0x5UL<<SYS_GPE_MFPH_PE8MFP_Pos)    /*!< GPE_MFPH PE8 setting for SC0_PWR        */

//PE.9 MFP
#define SYS_GPE_MFPH_PE9MFP_GPIO            (0x0UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for GPIO           */
#define SYS_GPE_MFPH_PE9MFP_UART1_RXD       (0x1UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for UART1_RXD      */
#define SYS_GPE_MFPH_PE9MFP_T1              (0x3UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for T1             */
#define SYS_GPE_MFPH_PE9MFP_I2C1_SDA        (0x4UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for I2C1_SDA       */
#define SYS_GPE_MFPH_PE9MFP_SC0_RST         (0x5UL<<SYS_GPE_MFPH_PE9MFP_Pos)    /*!< GPE_MFPH PE9 setting for SC0_RST        */

//PE.10 MFP
#define SYS_GPE_MFPH_PE10MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for GPIO          */
#define SYS_GPE_MFPH_PE10MFP_SPI1_MISO      (0x1UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for SPI1_MISO     */
#define SYS_GPE_MFPH_PE10MFP_SPI0_MISO0     (0x2UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for SPI0_MISO0    */
#define SYS_GPE_MFPH_PE10MFP_UART1_nCTS     (0x3UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for UART1_nCTS    */
#define SYS_GPE_MFPH_PE10MFP_SC0_DAT        (0x5UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for SC0_DAT       */
#define SYS_GPE_MFPH_PE10MFP_SPI1_CLK       (0x6UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for SPI1_CLK      */
#define SYS_GPE_MFPH_PE10MFP_EBI_AD7        (0x7UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for EBI_AD7       */
#define SYS_GPE_MFPH_PE10MFP_T0_EXT         (0x8UL<<SYS_GPE_MFPH_PE10MFP_Pos)   /*!< GPE_MFPH PE10 setting for T0_EXT        */

//PE.11 MFP
#define SYS_GPE_MFPH_PE11MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for GPIO          */
#define SYS_GPE_MFPH_PE11MFP_SPI1_MOSI      (0x1UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for SPI1_MOSI     */
#define SYS_GPE_MFPH_PE11MFP_SPI0_MOSI0     (0x2UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for SPI0_MOSI0    */
#define SYS_GPE_MFPH_PE11MFP_UART1_nRTS     (0x3UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for UART1_nRTS    */
#define SYS_GPE_MFPH_PE11MFP_SC0_CLK        (0x5UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for SC0_CLK       */
#define SYS_GPE_MFPH_PE11MFP_SPI1_MISO      (0x6UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for SPI1_MISO     */
#define SYS_GPE_MFPH_PE11MFP_EBI_AD6        (0x7UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for EBI_AD6       */
#define SYS_GPE_MFPH_PE11MFP_T1_EXT         (0x8UL<<SYS_GPE_MFPH_PE11MFP_Pos)   /*!< GPE_MFPH PE11 setting for T1_EXT        */

//PE.12 MFP
#define SYS_GPE_MFPH_PE12MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for GPIO          */
#define SYS_GPE_MFPH_PE12MFP_SPI1_SS        (0x1UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for SPI1_SS       */
#define SYS_GPE_MFPH_PE12MFP_SPI0_SS        (0x2UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for SPI0_SS       */
#define SYS_GPE_MFPH_PE12MFP_UART1_TXD      (0x3UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for UART1_TXD     */
#define SYS_GPE_MFPH_PE12MFP_I2C0_SCL       (0x4UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for I2C0_SCL      */
#define SYS_GPE_MFPH_PE12MFP_SPI1_MOSI      (0x6UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for SPI1_MOSI     */
#define SYS_GPE_MFPH_PE12MFP_EBI_AD5        (0x7UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for EBI_AD5       */
#define SYS_GPE_MFPH_PE12MFP_T2_EXT         (0x8UL<<SYS_GPE_MFPH_PE12MFP_Pos)   /*!< GPE_MFPH PE12 setting for T2_EXT        */

//PE.13 MFP
#define SYS_GPE_MFPH_PE13MFP_GPIO           (0x0UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for GPIO          */
#define SYS_GPE_MFPH_PE13MFP_SPI1_CLK       (0x1UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for SPI1_CLK      */
#define SYS_GPE_MFPH_PE13MFP_SPI0_CLK       (0x2UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for SPI0_CLK      */
#define SYS_GPE_MFPH_PE13MFP_UART1_RXD      (0x3UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for UART1_RXD     */
#define SYS_GPE_MFPH_PE13MFP_I2C0_SDA       (0x4UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for I2C0_SDA      */
#define SYS_GPE_MFPH_PE13MFP_SPI1_SS        (0x6UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for SPI1_SS       */
#define SYS_GPE_MFPH_PE13MFP_EBI_AD4        (0x7UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for EBI_AD4       */
#define SYS_GPE_MFPH_PE13MFP_T3_EXT         (0x8UL<<SYS_GPE_MFPH_PE13MFP_Pos)   /*!< GPE_MFPH PE13 setting for T3_EXT        */

//PF.0 MFP
#define SYS_GPF_MFPL_PF0MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for GPIO           */
#define SYS_GPF_MFPL_PF0MFP_X32_OUT         (0x1UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for X32_OUT        */
#define SYS_GPF_MFPL_PF0MFP_USCI2_CTL1      (0x5UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for USCI2_CTL1     */
#define SYS_GPF_MFPL_PF0MFP_INT5            (0x8UL<<SYS_GPF_MFPL_PF0MFP_Pos)    /*!< GPF_MFPL PF0 setting for INT5           */

//PF.1 MFP
#define SYS_GPF_MFPL_PF1MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for GPIO           */
#define SYS_GPF_MFPL_PF1MFP_X32_IN          (0x1UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for X32_IN         */
#define SYS_GPF_MFPL_PF1MFP_USCI2_CTL0      (0x5UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for USCI2_CTL0     */
#define SYS_GPF_MFPL_PF1MFP_PWM1_BRAKE0     (0x6UL<<SYS_GPF_MFPL_PF1MFP_Pos)    /*!< GPF_MFPL PF1 setting for PWM1_BRAKE0    */

//PF.2 MFP
#define SYS_GPF_MFPL_PF2MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for GPIO           */
#define SYS_GPF_MFPL_PF2MFP_USCI2_CLK       (0x5UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for USCI2_CLK      */
#define SYS_GPF_MFPL_PF2MFP_PWM1_BRAKE1     (0x6UL<<SYS_GPF_MFPL_PF2MFP_Pos)    /*!< GPF_MFPL PF2 setting for PWM1_BRAKE1    */

//PF.3 MFP
#define SYS_GPF_MFPL_PF3MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for GPIO           */
#define SYS_GPF_MFPL_PF3MFP_XT1_OUT         (0x1UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for XT1_OUT        */
#define SYS_GPF_MFPL_PF3MFP_I2C1_SCL        (0x3UL<<SYS_GPF_MFPL_PF3MFP_Pos)    /*!< GPF_MFPL PF3 setting for I2C1_SCL       */

//PF.4 MFP
#define SYS_GPF_MFPL_PF4MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for GPIO           */
#define SYS_GPF_MFPL_PF4MFP_XT1_IN          (0x1UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for XT1_IN         */
#define SYS_GPF_MFPL_PF4MFP_I2C1_SDA        (0x3UL<<SYS_GPF_MFPL_PF4MFP_Pos)    /*!< GPF_MFPL PF4 setting for I2C1_SDA       */

//PF.5 MFP
#define SYS_GPF_MFPL_PF5MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for GPIO           */
#define SYS_GPF_MFPL_PF5MFP_T3_EXT          (0x3UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for T3_EXT         */
#define SYS_GPF_MFPL_PF5MFP_SC1_CD          (0x5UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for SC1_CD         */
#define SYS_GPF_MFPL_PF5MFP_TM_BRAKE0       (0x6UL<<SYS_GPF_MFPL_PF5MFP_Pos)    /*!< GPF_MFPL PF5 setting for TM_BRAKE0      */

//PF.6 MFP
#define SYS_GPF_MFPL_PF6MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF6MFP_Pos)    /*!< GPF_MFPL PF6 setting for GPIO           */

//PF.7 MFP
#define SYS_GPF_MFPL_PF7MFP_GPIO            (0x0UL<<SYS_GPF_MFPL_PF7MFP_Pos)    /*!< GPF_MFPL PF7 setting for GPIO           */



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
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

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
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
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
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_5V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_2V
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
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

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
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_MCURF_Msk)

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
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_MCURF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )




/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    do
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }
    while(SYS->REGLCTL == 0);
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
    SYS->REGLCTL = 0;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
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

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
