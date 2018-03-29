/**************************************************************************//**
* @file     sys.h
* @version  V1.00
* $Revision: 17 $
* $Date: 16/03/30 1:41p $
* @brief    NANO103 Series system control header file.
*
* @note
* Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup NANO103_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CHIP_RST  ((0x0<<24) | SYS_IPRST1_CHIPRST_Pos  ) /*!< CPU reset is one of the SYS_ResetModule parameter */
#define CPU_RST   ((0x0<<24) | SYS_IPRST1_CPURST_Pos   ) /*!< CHIP reset is one of the SYS_ResetModule parameter */
#define DMA_RST   ((0x0<<24) | SYS_IPRST1_PDMARST_Pos  ) /*!< DMA reset is one of the SYS_ResetModule parameter */
#define SC1_RST   ((0x4<<24) | SYS_IPRST2_SC1RST_Pos   ) /*!< SmartCard1 reset is one of the SYS_ResetModule parameter */
#define SC0_RST   ((0x4<<24) | SYS_IPRST2_SC0RST_Pos   ) /*!< SmartCard0 reset is one of the SYS_ResetModule parameter */
#define ADC_RST   ((0x4<<24) | SYS_IPRST2_ADCRST_Pos   ) /*!< ADC reset is one of the SYS_ResetModule parameter */
#define ACMP0_RST ((0x4<<24) | SYS_IPRST2_ACMP0RST_Pos ) /*!< ACMP0 reset is one of the SYS_ResetModule parameter */
#define PWM0_RST  ((0x4<<24) | SYS_IPRST2_PWM0RST_Pos  ) /*!< PWM0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST ((0x4<<24) | SYS_IPRST2_UART1RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART0_RST ((0x4<<24) | SYS_IPRST2_UART0RST_Pos ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define SPI3_RST  ((0x4<<24) | SYS_IPRST2_SPI3RST_Pos  ) /*!< SPI3 reset is one of the SYS_ResetModule parameter */
#define SPI2_RST  ((0x4<<24) | SYS_IPRST2_SPI2RST_Pos  ) /*!< SPI2 reset is one of the SYS_ResetModule parameter */
#define SPI1_RST  ((0x4<<24) | SYS_IPRST2_SPI1RST_Pos  ) /*!< SPI1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST  ((0x4<<24) | SYS_IPRST2_SPI0RST_Pos  ) /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST  ((0x4<<24) | SYS_IPRST2_I2C1RST_Pos  ) /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST  ((0x4<<24) | SYS_IPRST2_I2C0RST_Pos  ) /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST  ((0x4<<24) | SYS_IPRST2_TMR3RST_Pos  ) /*!< Timer3 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST  ((0x4<<24) | SYS_IPRST2_TMR2RST_Pos  ) /*!< Timer2 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST  ((0x4<<24) | SYS_IPRST2_TMR1RST_Pos  ) /*!< Timer1 reset is one of the SYS_ResetModule parameter */
#define TMR0_RST  ((0x4<<24) | SYS_IPRST2_TMR0RST_Pos  ) /*!< Timer0 reset is one of the SYS_ResetModule parameter */
#define GPIO_RST  ((0x4<<24) | SYS_IPRST2_GPIORST_Pos  ) /*!< GPIO reset is one of the SYS_ResetModule parameter */

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/


/********************** Brown Out Detector Threshold Voltage Selection constant definitions **********************/
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODREN_Pos)        /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (1UL<<SYS_BODCTL_BODIE_Pos)         /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BODVL_1_7V           (0UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.7V */
#define SYS_BODCTL_BODVL_1_8V           (1UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.8V */
#define SYS_BODCTL_BODVL_1_9V           (2UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.9V */
#define SYS_BODCTL_BODVL_2_0V           (3UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.0V */
#define SYS_BODCTL_BODVL_2_1V           (4UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.1V */
#define SYS_BODCTL_BODVL_2_2V           (5UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */
#define SYS_BODCTL_BODVL_2_3V           (6UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.3V */
#define SYS_BODCTL_BODVL_2_4V           (7UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.4V */
#define SYS_BODCTL_BODVL_2_5V           (8UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.5V */
#define SYS_BODCTL_BODVL_2_6V           (9UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.6V */
#define SYS_BODCTL_BODVL_2_7V           (0xAUL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_8V           (0xBUL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.8V */
#define SYS_BODCTL_BODVL_2_9V           (0xCUL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.9V */
#define SYS_BODCTL_BODVL_3_0V           (0xDUL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 3.0V */
#define SYS_BODCTL_BODVL_3_1V           (0xEUL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 3.1V */

/********************** Low Power Brown Out Detector Threshold Voltage Selection constant definitions **********************/
#define SYS_BODCTL_LPBOD_RST_EN           (1UL<<SYS_BODCTL_LPBODREN_Pos)        /*!< Low Power Brown-out Reset Enable */
#define SYS_BODCTL_LPBOD_INTERRUPT_EN     (1UL<<SYS_BODCTL_LPBODIE_Pos)         /*!< Low Power Brown-out Interrupt Enable */
#define SYS_BODCTL_LPBODVL_2_0V           (0UL<<SYS_BODCTL_LPBODVL_Pos)       /*!< Setting Low Power Brown Out Detector Threshold Voltage as 2.0V */
#define SYS_BODCTL_LPBODVL_2_5V           (1UL<<SYS_BODCTL_LPBODVL_Pos)       /*!< Setting Low Power Brown Out Detector Threshold Voltage as 2.5V */

/********************* Bit definition of IVREFCTL register **********************/
#define SYS_IVREFCTL_BGP_EN      ((uint32_t)0x00000001)      /*!<Band-gap Enable */
#define SYS_IVREFCTL_REG_EN      ((uint32_t)0x00000002)      /*!<Regulator Enable */
#define SYS_IVREFCTL_SEL25       ((uint32_t)0x00000008)      /*!<Regulator Output Voltage 2.5V */
#define SYS_IVREFCTL_SEL18       ((uint32_t)0x00000004)      /*!<Regulator Output Voltage 1.8V */
#define SYS_IVREFCTL_SEL15       ((uint32_t)0x00000000)      /*!<Regulator Output Voltage 1.5V */
#define SYS_IVREFCTL_EXTMODE     ((uint32_t)0x00000010)      /*!<Regulator External Mode */


/********************* Bit definition of LDOCTL register **********************/
#define SYS_LDOCTL_LDO_LEVEL12  ((uint32_t)0x00000000)      /*!< LDO  Output 1.2  Voltage  */
#define SYS_LDOCTL_LDO_LEVEL16  ((uint32_t)0x00000004)      /*!< LDO  Output 1.6  Voltage  */
#define SYS_LDOCTL_LDO_LEVEL18  ((uint32_t)0x00000008)      /*!< LDO  Output 1.8  Voltage  */


/********************* Bit definition of IRC0TCTL/IRC1TCTL/MIRCTCTL register **********************/
#define SYS_IRC0TCTL_TRIM_11_0592M ((uint32_t)0x00000001)      /*!<Trim IRC to 11.0592 MHz */
#define SYS_IRC0TCTL_TRIM_12M      ((uint32_t)0x00000002)      /*!<Trim IRC to 12 MHz */
#define SYS_IRC0TCTL_TRIM_12_288M  ((uint32_t)0x00000003)      /*!<Trim IRC to 12.288 MHz */
#define SYS_IRC0TCTL_TRIM_16M      ((uint32_t)0x00000004)      /*!<Trim IRC to 16 MHz */

#define SYS_IRC1TCTL_TRIM_36M      ((uint32_t)0x00000002)      /*!<Trim IRC to 36 MHz */
#define SYS_MIRCTCTL_TRIM_4M       ((uint32_t)0x00000002)      /*!<Trim IRC to 4 MHz */

#define SYS_IRCTCTL_LOOP_4CLK      ((uint32_t)0x00000000)      /*!<Based on average difference in 4 x 32.768 kHz clock */
#define SYS_IRCTCTL_LOOP_8CLK      ((uint32_t)0x00000010)      /*!<Based on average difference in 8 x 32.768 kHz clock */
#define SYS_IRCTCTL_LOOP_16CLK     ((uint32_t)0x00000020)      /*!<Based on average difference in 16 x 32.768 kHz clock */
#define SYS_IRCTCTL_LOOP_32CLK     ((uint32_t)0x00000030)      /*!<Based on average difference in 32 x 32.768 kHz clock */

#define SYS_IRCTCTL_RETRY_64       ((uint32_t)0x00000000)      /*!<Trim retry count limitation is 64 */
#define SYS_IRCTCTL_RETRY_128      ((uint32_t)0x00000040)      /*!<Trim retry count limitation is 128 */
#define SYS_IRCTCTL_RETRY_256      ((uint32_t)0x00000080)      /*!<Trim retry count limitation is 256 */
#define SYS_IRCTCTL_RETRY_512      ((uint32_t)0x000000C0)      /*!<Trim retry count limitation is 512 */

#define SYS_IRCTCTL_CLKERR_STOP    ((uint32_t)0x00000100)      /*!<Clock error stop enable */

/********************* Bit definition of IRC0TIEN/IRC1TIEN/MIRCTIEN register **********************/
#define SYS_IRCTIEN_DISABLE      ((uint32_t)0x00000000)      /*!<Trim failure interrupt disable */
#define SYS_IRCTIEN_FAIL_EN      ((uint32_t)0x00000002)      /*!<Trim failure interrupt enable */
#define SYS_IRCTIEN_32KERR_EN    ((uint32_t)0x00000004)      /*!<32.768 kHz Clock Error Interrupt Enable */

/********************* Bit definition of IRC0TISTS/IRC1TISTS/MIRCTISTS register **********************/
#define SYS_IRCTISTS_FREQLOCK     ((uint32_t)0x00000001)      /*!<HIRC frequency lock status */
#define SYS_IRCTISTS_FAIL_INT     ((uint32_t)0x00000002)      /*!<Trim failure interrupt status */
#define SYS_IRCTISTS_32KERR_INT   ((uint32_t)0x00000004)      /*!<32.768 kHz Clock Error Interrupt Status */

/********************* Bit definition of GPA_MFPL register **********************/
#define SYS_GPA_MFPL_PA0MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA0MFP_ADC_CH0           (0x01UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< ADC channel0 analog input. */
#define SYS_GPA_MFPL_PA0MFP_ACMP0_P           (0x02UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< Analog comparator0 positive input pin. */
#define SYS_GPA_MFPL_PA0MFP_TM2_EXT           (0x03UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< Timer2 external capture input. */
#define SYS_GPA_MFPL_PA0MFP_PWM0_CH2          (0x05UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< PWM0 channel2 output/capture input. */
#define SYS_GPA_MFPL_PA0MFP_SPI3_MOSI1        (0x06UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< 2nd SPI3 MOSI (Master Out, Slave In) pin. */
#define SYS_GPA_MFPL_PA1MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA1MFP_ADC_CH1           (0x01UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< ADC channel1 analog input. */
#define SYS_GPA_MFPL_PA1MFP_ACMP0_N           (0x02UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< Analog comparator0 negative input pin. */
#define SYS_GPA_MFPL_PA1MFP_SPI3_MISO1        (0x06UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< 2nd SPI3 MISO (Master In, Slave Out) pin. */
#define SYS_GPA_MFPL_PA2MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA2MFP_ADC_CH2           (0x01UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< ADC channel2 analog input. */
#define SYS_GPA_MFPL_PA2MFP_UART1_RXD         (0x05UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< Data receiver input pin for UART1. */
#define SYS_GPA_MFPL_PA3MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA3MFP_ADC_CH3           (0x01UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< ADC channel3 analog input. */
#define SYS_GPA_MFPL_PA3MFP_UART1_TXD         (0x05UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< Data transmitter output pin for UART1. */
#define SYS_GPA_MFPL_PA3MFP_SPI3_MOSI0        (0x06UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< 1st SPI3 MOSI (Master Out, Slave In) pin. */
#define SYS_GPA_MFPL_PA4MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA4MFP_ADC_CH4           (0x01UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< ADC channel4 analog input. */
#define SYS_GPA_MFPL_PA4MFP_I2C0_SDA          (0x05UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< I2C0 data input/output pin. */
#define SYS_GPA_MFPL_PA4MFP_SPI3_MISO0        (0x06UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< 1st SPI3 MISO (Master In, Slave Out) pin. */
#define SYS_GPA_MFPL_PA5MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA5MFP_ADC_CH5           (0x01UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< ADC channel5 analog input. */
#define SYS_GPA_MFPL_PA5MFP_I2C0_SCL          (0x05UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< I2C0 clock pin. */
#define SYS_GPA_MFPL_PA5MFP_SPI3_CLK          (0x06UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< SPI3 serial clock pin. */
#define SYS_GPA_MFPL_PA6MFP_GPIO              (0x00UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPL_PA6MFP_ADC_CH6           (0x01UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< ADC channel6 analog input. */
#define SYS_GPA_MFPL_PA6MFP_ACMP0_O           (0x02UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< Analog comparator0 output. */
#define SYS_GPA_MFPL_PA6MFP_TM3_EXT           (0x03UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< Timer3 external capture input. */
#define SYS_GPA_MFPL_PA6MFP_TM3_CNT           (0x04UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< Timer3 event counter input. */
#define SYS_GPA_MFPL_PA6MFP_PWM0_CH3          (0x05UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< PWM0 channel3 output/capture input. */
#define SYS_GPA_MFPL_PA6MFP_SPI3_SS0          (0x06UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< 1st SPI3 slave select pin. */
#define SYS_GPA_MFPL_PA6MFP_TM3_OUT           (0x07UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< Timer3 toggle output. */
/********************* Bit definition of GPA_MFPH register **********************/
#define SYS_GPA_MFPH_PA8MFP_GPIO              (0x00UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA8MFP_I2C0_SDA          (0x01UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< I2C0 data input/output pin. */
#define SYS_GPA_MFPH_PA8MFP_TM0_CNT           (0x02UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< Timer0 event counter input. */
#define SYS_GPA_MFPH_PA8MFP_SC0_CLK           (0x03UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< SmartCard0 clock pin. */
#define SYS_GPA_MFPH_PA8MFP_SPI2_SS0          (0x04UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< 1st SPI2 slave select pin. */
#define SYS_GPA_MFPH_PA8MFP_TM0_OUT           (0x05UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< Timer0 toggle output. */
#define SYS_GPA_MFPH_PA8MFP_UART1_nCTS        (0x06UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< Clear to Send input pin for UART1. */
#define SYS_GPA_MFPH_PA9MFP_GPIO              (0x00UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA9MFP_I2C0_SCL          (0x01UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< I2C0 clock pin. */
#define SYS_GPA_MFPH_PA9MFP_TM1_CNT           (0x02UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< Timer1 event counter input. */
#define SYS_GPA_MFPH_PA9MFP_SC0_DAT           (0x03UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< SmartCard0 data pin. */
#define SYS_GPA_MFPH_PA9MFP_SPI2_CLK          (0x04UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< SPI2 serial clock pin. */
#define SYS_GPA_MFPH_PA9MFP_TM1_OUT           (0x05UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< Timer1 toggle output. */
#define SYS_GPA_MFPH_PA9MFP_UART1_nRTS        (0x06UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< Request to Send output pin for UART1. */
#define SYS_GPA_MFPH_PA9MFP_SNOOPER           (0x07UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< Snooper pin. */
#define SYS_GPA_MFPH_PA10MFP_GPIO             (0x00UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA10MFP_I2C1_SDA         (0x01UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*!< I2C1 data input/output pin. */
#define SYS_GPA_MFPH_PA10MFP_TM2_CNT          (0x02UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*!< Timer2 event counter input. */
#define SYS_GPA_MFPH_PA10MFP_SC0_PWR          (0x03UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*!< SmartCard0 power pin. */
#define SYS_GPA_MFPH_PA10MFP_SPI2_MISO0       (0x04UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*!< 1st SPI2 MISO (Master In, Slave Out) pin. */
#define SYS_GPA_MFPH_PA10MFP_TM2_OUT          (0x05UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*!< Timer2 toggle output. */
#define SYS_GPA_MFPH_PA11MFP_GPIO             (0x00UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA11MFP_I2C1_SCL         (0x01UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*!< I2C1 clock pin. */
#define SYS_GPA_MFPH_PA11MFP_TM3_CNT          (0x02UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*!< Timer3 event counter input. */
#define SYS_GPA_MFPH_PA11MFP_SC0_RST          (0x03UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*!< SmartCard0 reset pin. */
#define SYS_GPA_MFPH_PA11MFP_SPI2_MOSI0       (0x04UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*!< 1st SPI2 MOSI (Master Out, Slave In) pin. */
#define SYS_GPA_MFPH_PA11MFP_TM3_OUT          (0x05UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*!< Timer3 toggle output. */
#define SYS_GPA_MFPH_PA12MFP_GPIO             (0x00UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA12MFP_PWM0_CH0         (0x01UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*!< PWM0 channel0 output/capture input. */
#define SYS_GPA_MFPH_PA12MFP_TM0_EXT          (0x03UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*!< Timer0 external capture input. */
#define SYS_GPA_MFPH_PA12MFP_I2C0_SDA         (0x05UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*!< I2C0 data input/output pin. */
#define SYS_GPA_MFPH_PA13MFP_GPIO             (0x00UL<<SYS_GPA_MFPH_PA13MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA13MFP_PWM0_CH1         (0x01UL<<SYS_GPA_MFPH_PA13MFP_Pos) /*!< PWM0 channel1 output/capture input. */
#define SYS_GPA_MFPH_PA13MFP_TM1_EXT          (0x03UL<<SYS_GPA_MFPH_PA13MFP_Pos) /*!< Timer1 external capture input. */
#define SYS_GPA_MFPH_PA13MFP_I2C0_SCL         (0x05UL<<SYS_GPA_MFPH_PA13MFP_Pos) /*!< I2C0 clock pin. */
#define SYS_GPA_MFPH_PA14MFP_GPIO             (0x00UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA14MFP_PWM0_CH2         (0x01UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< PWM0 channel2 output/capture input. */
#define SYS_GPA_MFPH_PA14MFP_I2C1_SDA         (0x02UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< I2C1 data input/output pin. */
#define SYS_GPA_MFPH_PA14MFP_TM2_EXT          (0x03UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< Timer2 external capture input. */
#define SYS_GPA_MFPH_PA14MFP_TM2_CNT          (0x05UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< Timer2 event counter input. */
#define SYS_GPA_MFPH_PA14MFP_UART0_RXD        (0x06UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< Data receiver input pin for UART0. */
#define SYS_GPA_MFPH_PA14MFP_TM2_OUT          (0x07UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*!< Timer2 toggle output. */
#define SYS_GPA_MFPH_PA15MFP_GPIO             (0x00UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPA_MFPH_PA15MFP_PWM0_CH3         (0x01UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< PWM0 channel3 output/capture input. */
#define SYS_GPA_MFPH_PA15MFP_I2C1_SCL         (0x02UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< I2C1 clock pin. */
#define SYS_GPA_MFPH_PA15MFP_TM3_EXT          (0x03UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< Timer3 external capture input. */
#define SYS_GPA_MFPH_PA15MFP_SC0_PWR          (0x04UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< SmartCard0 power pin. */
#define SYS_GPA_MFPH_PA15MFP_TM3_CNT          (0x05UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< Timer3 event counter input. */
#define SYS_GPA_MFPH_PA15MFP_UART0_TXD        (0x06UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< Data transmitter output pin for UART0. */
#define SYS_GPA_MFPH_PA15MFP_TM3_OUT          (0x07UL<<SYS_GPA_MFPH_PA15MFP_Pos) /*!< Timer3 toggle output. */
/********************* Bit definition of GPB_MFPL register **********************/
#define SYS_GPB_MFPL_PB0MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB0MFP_UART0_RXD         (0x01UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< Data receiver input pin for UART0. */
#define SYS_GPB_MFPL_PB0MFP_SPI1_MOSI0        (0x03UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< 1st SPI1 MOSI (Master Out, Slave In) pin. */
#define SYS_GPB_MFPL_PB1MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB1MFP_UART0_TXD         (0x01UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< Data transmitter output pin for UART0. */
#define SYS_GPB_MFPL_PB1MFP_SPI1_MISO0        (0x03UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< 1st SPI1 MISO (Master In, Slave Out) pin. */
#define SYS_GPB_MFPL_PB2MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB2MFP_UART0_nRTS        (0x01UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< Request to Send output pin for UART0. */
#define SYS_GPB_MFPL_PB2MFP_SPI1_CLK          (0x03UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< SPI1 serial clock pin. */
#define SYS_GPB_MFPL_PB2MFP_CLKO              (0x04UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< Output selected clock. */
#define SYS_GPB_MFPL_PB3MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB3MFP_UART0_nCTS        (0x01UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< Clear to Send input pin for UART0. */
#define SYS_GPB_MFPL_PB3MFP_SPI1_SS0          (0x03UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< 1st SPI1 slave select pin. */
#define SYS_GPB_MFPL_PB3MFP_SC1_CD            (0x04UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< SmartCard1 card detect pin. */
#define SYS_GPB_MFPL_PB4MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB4MFP_UART1_RXD         (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< Data receiver input pin for UART1. */
#define SYS_GPB_MFPL_PB4MFP_SC0_CD            (0x03UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< SmartCard0 card detect pin. */
#define SYS_GPB_MFPL_PB4MFP_SPI2_SS0          (0x04UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< 1st SPI2 slave select pin. */
#define SYS_GPB_MFPL_PB4MFP_RTC_HZ            (0x06UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< RTC output 1Hz pin. */
#define SYS_GPB_MFPL_PB5MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB5MFP_UART1_TXD         (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< Data transmitter output pin for UART1. */
#define SYS_GPB_MFPL_PB5MFP_SC0_RST           (0x03UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< SmartCard0 reset pin. */
#define SYS_GPB_MFPL_PB5MFP_SPI2_CLK          (0x04UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< SPI2 serial clock pin. */
#define SYS_GPB_MFPL_PB6MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB6MFP_UART1_nRTS        (0x01UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< Request to Send output pin for UART1. */
#define SYS_GPB_MFPL_PB6MFP_SPI2_MISO0        (0x04UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< 1st SPI2 MISO (Master In, Slave Out) pin. */
#define SYS_GPB_MFPL_PB7MFP_GPIO              (0x00UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPL_PB7MFP_UART1_nCTS        (0x01UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< Clear to Send input pin for UART1. */
#define SYS_GPB_MFPL_PB7MFP_SPI2_MOSI0        (0x04UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< 1st SPI2 MOSI (Master Out, Slave In) pin. */
/********************* Bit definition of GPB_MFPH register **********************/
#define SYS_GPB_MFPH_PB8MFP_GPIO              (0x00UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB8MFP_STADC             (0x01UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< ADC external trigger input. */
#define SYS_GPB_MFPH_PB8MFP_TM0_CNT           (0x02UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< Timer0 event counter input. */
#define SYS_GPB_MFPH_PB8MFP_INT0              (0x03UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< External interrupt0 input pin. */
#define SYS_GPB_MFPH_PB8MFP_TM0_OUT           (0x04UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< Timer0 toggle output. */
#define SYS_GPB_MFPH_PB8MFP_SNOOPER           (0x07UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< Snooper pin. */
#define SYS_GPB_MFPH_PB9MFP_GPIO              (0x00UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB9MFP_SPI1_SS1          (0x01UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< 1st SPI1 slave select pin. */
#define SYS_GPB_MFPH_PB9MFP_TM1_CNT           (0x02UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< Timer1 event counter input. */
#define SYS_GPB_MFPH_PB9MFP_TM1_OUT           (0x04UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< Timer1 toggle output. */
#define SYS_GPB_MFPH_PB9MFP_INT0              (0x05UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< External interrupt0 input pin. */
#define SYS_GPB_MFPH_PB10MFP_GPIO             (0x00UL<<SYS_GPB_MFPH_PB10MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB10MFP_SPI0_MOSI0       (0x01UL<<SYS_GPB_MFPH_PB10MFP_Pos) /*!< 1st SPI0 MOSI (Master Out, Slave In) pin. */
#define SYS_GPB_MFPH_PB10MFP_TM2_CNT          (0x02UL<<SYS_GPB_MFPH_PB10MFP_Pos) /*!< Timer2 event counter input. */
#define SYS_GPB_MFPH_PB10MFP_TM2_OUT          (0x04UL<<SYS_GPB_MFPH_PB10MFP_Pos) /*!< Timer2 toggle output. */
#define SYS_GPB_MFPH_PB10MFP_SPI0_SS1         (0x05UL<<SYS_GPB_MFPH_PB10MFP_Pos) /*!< 1st SPI0 slave select pin. */
#define SYS_GPB_MFPH_PB11MFP_GPIO             (0x00UL<<SYS_GPB_MFPH_PB11MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB11MFP_PWM0_CH4         (0x01UL<<SYS_GPB_MFPH_PB11MFP_Pos) /*!< PWM0 channel4 output/capture input. */
#define SYS_GPB_MFPH_PB11MFP_TM3_CNT          (0x02UL<<SYS_GPB_MFPH_PB11MFP_Pos) /*!< Timer3 event counter input. */
#define SYS_GPB_MFPH_PB11MFP_TM3_OUT          (0x04UL<<SYS_GPB_MFPH_PB11MFP_Pos) /*!< Timer3 toggle output. */
#define SYS_GPB_MFPH_PB11MFP_SPI0_MISO0       (0x05UL<<SYS_GPB_MFPH_PB11MFP_Pos) /*!< 1st SPI0 MISO (Master In, Slave Out) pin. */
#define SYS_GPB_MFPH_PB13MFP_GPIO             (0x00UL<<SYS_GPB_MFPH_PB13MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB13MFP_SPI2_MISO1       (0x03UL<<SYS_GPB_MFPH_PB13MFP_Pos) /*!< 2nd SPI2 MISO (Master In, Slave Out) pin. */
#define SYS_GPB_MFPH_PB13MFP_SNOOPER          (0x07UL<<SYS_GPB_MFPH_PB13MFP_Pos) /*!< Snooper pin. */
#define SYS_GPB_MFPH_PB14MFP_GPIO             (0x00UL<<SYS_GPB_MFPH_PB14MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB14MFP_INT0             (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos) /*!< External interrupt0 input pin. */
#define SYS_GPB_MFPH_PB14MFP_SPI2_MOSI1       (0x03UL<<SYS_GPB_MFPH_PB14MFP_Pos) /*!< 2nd SPI2 MOSI (Master Out, Slave In) pin. */
#define SYS_GPB_MFPH_PB14MFP_SPI2_SS1         (0x04UL<<SYS_GPB_MFPH_PB14MFP_Pos) /*!< 1st SPI2 slave select pin. */
#define SYS_GPB_MFPH_PB15MFP_GPIO             (0x00UL<<SYS_GPB_MFPH_PB15MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPB_MFPH_PB15MFP_INT1             (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos) /*!< External interrupt1 input pin. */
#define SYS_GPB_MFPH_PB15MFP_SNOOPER          (0x03UL<<SYS_GPB_MFPH_PB15MFP_Pos) /*!< Snooper pin. */
#define SYS_GPB_MFPH_PB15MFP_SC1_CD           (0x04UL<<SYS_GPB_MFPH_PB15MFP_Pos) /*!< SmartCard1 card detect pin. */
/********************* Bit definition of GPC_MFPL register **********************/
#define SYS_GPC_MFPL_PC0MFP_GPIO              (0x00UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPL_PC0MFP_SPI0_SS0          (0x01UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< 1st SPI0 slave select pin. */
#define SYS_GPC_MFPL_PC0MFP_SC1_CLK           (0x04UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< SmartCard1 clock pin. */
#define SYS_GPC_MFPL_PC0MFP_PWM0_BRAKE1       (0x05UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< Brake input pin 1 of PWM0. */
#define SYS_GPC_MFPL_PC1MFP_GPIO              (0x00UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPL_PC1MFP_SPI0_CLK          (0x01UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< SPI0 serial clock pin. */
#define SYS_GPC_MFPL_PC1MFP_SC1_DAT           (0x04UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< SmartCard1 data pin. */
#define SYS_GPC_MFPL_PC1MFP_PWM0_BRAKE1       (0x05UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< Brake input pin 1 of PWM0. */
#define SYS_GPC_MFPL_PC2MFP_GPIO              (0x00UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPL_PC2MFP_SPI0_MISO0        (0x01UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< 1st SPI0 MISO (Master In, Slave Out) pin. */
#define SYS_GPC_MFPL_PC2MFP_SC1_PWR           (0x04UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< SmartCard1 power pin. */
#define SYS_GPC_MFPL_PC2MFP_PWM0_BRAKE0       (0x05UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< Brake input pin 0 of PWM0. */
#define SYS_GPC_MFPL_PC3MFP_GPIO              (0x00UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPL_PC3MFP_SPI0_MOSI0        (0x01UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< 1st SPI0 MOSI (Master Out, Slave In) pin. */
#define SYS_GPC_MFPL_PC3MFP_SC1_RST           (0x04UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< SmartCard1 reset pin. */
#define SYS_GPC_MFPL_PC3MFP_PWM0_BRAKE0       (0x05UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< Brake input pin 0 of PWM0. */
#define SYS_GPC_MFPL_PC6MFP_GPIO              (0x00UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPL_PC6MFP_UART1_RXD         (0x01UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< Data receiver input pin for UART1. */
#define SYS_GPC_MFPL_PC6MFP_TM0_EXT           (0x03UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< Timer0 external capture input. */
#define SYS_GPC_MFPL_PC6MFP_SC1_CD            (0x04UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< SmartCard1 card detect pin. */
#define SYS_GPC_MFPL_PC6MFP_PWM0_CH0          (0x05UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< PWM0 channel0 output/capture input. */
#define SYS_GPC_MFPL_PC7MFP_GPIO              (0x00UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPL_PC7MFP_UART1_TXD         (0x01UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< Data transmitter output pin for UART1. */
#define SYS_GPC_MFPL_PC7MFP_ADC_CH7           (0x02UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< ADC channel7 analog input. */
#define SYS_GPC_MFPL_PC7MFP_TM1_EXT           (0x03UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< Timer1 external capture input. */
#define SYS_GPC_MFPL_PC7MFP_PWM0_CH1          (0x05UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< PWM0 channel1 output/capture input. */
/********************* Bit definition of GPC_MFPH register **********************/
#define SYS_GPC_MFPH_PC8MFP_GPIO              (0x00UL<<SYS_GPC_MFPH_PC8MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPH_PC8MFP_SPI1_SS0          (0x01UL<<SYS_GPC_MFPH_PC8MFP_Pos) /*!< 1st SPI1 slave select pin. */
#define SYS_GPC_MFPH_PC8MFP_I2C1_SDA          (0x05UL<<SYS_GPC_MFPH_PC8MFP_Pos) /*!< I2C1 data input/output pin. */
#define SYS_GPC_MFPH_PC9MFP_GPIO              (0x00UL<<SYS_GPC_MFPH_PC9MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPH_PC9MFP_SPI1_CLK          (0x01UL<<SYS_GPC_MFPH_PC9MFP_Pos) /*!< SPI1 serial clock pin. */
#define SYS_GPC_MFPH_PC9MFP_I2C1_SCL          (0x05UL<<SYS_GPC_MFPH_PC9MFP_Pos) /*!< I2C1 clock pin. */
#define SYS_GPC_MFPH_PC10MFP_GPIO             (0x00UL<<SYS_GPC_MFPH_PC10MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPH_PC10MFP_SPI1_MISO0       (0x01UL<<SYS_GPC_MFPH_PC10MFP_Pos) /*!< 1st SPI1 MISO (Master In, Slave Out) pin. */
#define SYS_GPC_MFPH_PC10MFP_UART1_RXD        (0x05UL<<SYS_GPC_MFPH_PC10MFP_Pos) /*!< Data receiver input pin for UART1. */
#define SYS_GPC_MFPH_PC11MFP_GPIO             (0x00UL<<SYS_GPC_MFPH_PC11MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPH_PC11MFP_SPI1_MOSI0       (0x01UL<<SYS_GPC_MFPH_PC11MFP_Pos) /*!< 1st SPI1 MOSI (Master Out, Slave In) pin. */
#define SYS_GPC_MFPH_PC11MFP_UART1_TXD        (0x05UL<<SYS_GPC_MFPH_PC11MFP_Pos) /*!< Data transmitter output pin for UART1. */
#define SYS_GPC_MFPH_PC14MFP_GPIO             (0x00UL<<SYS_GPC_MFPH_PC14MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPH_PC14MFP_UART1_nCTS       (0x01UL<<SYS_GPC_MFPH_PC14MFP_Pos) /*!< Clear to Send input pin for UART1. */
#define SYS_GPC_MFPH_PC15MFP_GPIO             (0x00UL<<SYS_GPC_MFPH_PC15MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPC_MFPH_PC15MFP_UART1_nRTS       (0x01UL<<SYS_GPC_MFPH_PC15MFP_Pos) /*!< Request to Send output pin for UART1. */
#define SYS_GPC_MFPH_PC15MFP_TM0_EXT          (0x03UL<<SYS_GPC_MFPH_PC15MFP_Pos) /*!< Timer0 external capture input. */
/********************* Bit definition of GPD_MFPL register **********************/
#define SYS_GPD_MFPL_PD6MFP_GPIO              (0x00UL<<SYS_GPD_MFPL_PD6MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPD_MFPL_PD6MFP_SPI1_MOSI1        (0x03UL<<SYS_GPD_MFPL_PD6MFP_Pos) /*!< 2nd SPI1 MOSI (Master Out, Slave In) pin. */
#define SYS_GPD_MFPL_PD6MFP_SC1_RST           (0x04UL<<SYS_GPD_MFPL_PD6MFP_Pos) /*!< SmartCard1 reset pin. */
#define SYS_GPD_MFPL_PD7MFP_GPIO              (0x00UL<<SYS_GPD_MFPL_PD7MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPD_MFPL_PD7MFP_SPI1_MISO1        (0x03UL<<SYS_GPD_MFPL_PD7MFP_Pos) /*!< 2nd SPI1 MISO (Master In, Slave Out) pin. */
#define SYS_GPD_MFPL_PD7MFP_SC1_PWR           (0x04UL<<SYS_GPD_MFPL_PD7MFP_Pos) /*!< SmartCard1 power pin. */
/********************* Bit definition of GPD_MFPH register **********************/
#define SYS_GPD_MFPH_PD14MFP_GPIO             (0x00UL<<SYS_GPD_MFPH_PD14MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPD_MFPH_PD14MFP_SPI0_MOSI1       (0x01UL<<SYS_GPD_MFPH_PD14MFP_Pos) /*!< 2nd SPI0 MOSI (Master Out, Slave In) pin. */
#define SYS_GPD_MFPH_PD14MFP_SC1_DAT          (0x04UL<<SYS_GPD_MFPH_PD14MFP_Pos) /*!< SmartCard1 data pin. */
#define SYS_GPD_MFPH_PD15MFP_GPIO             (0x00UL<<SYS_GPD_MFPH_PD15MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPD_MFPH_PD15MFP_SPI0_MISO1       (0x01UL<<SYS_GPD_MFPH_PD15MFP_Pos) /*!< 2nd SPI0 MISO (Master In, Slave Out) pin. */
#define SYS_GPD_MFPH_PD15MFP_SC1_CLK          (0x04UL<<SYS_GPD_MFPH_PD15MFP_Pos) /*!< SmartCard1 clock pin. */
/********************* Bit definition of GPE_MFPL register **********************/
#define SYS_GPE_MFPL_PE5MFP_GPIO              (0x00UL<<SYS_GPE_MFPL_PE5MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPE_MFPL_PE5MFP_PWM0_CH5          (0x01UL<<SYS_GPE_MFPL_PE5MFP_Pos) /*!< PWM0 channel5 output/capture input. */
#define SYS_GPE_MFPL_PE5MFP_RTC_HZ            (0x06UL<<SYS_GPE_MFPL_PE5MFP_Pos) /*!< RTC output 1Hz pin. */
/********************* Bit definition of GPF_MFPL register **********************/
#define SYS_GPF_MFPL_PF0MFP_GPIO              (0x00UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPF_MFPL_PF0MFP_INT0              (0x05UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< External interrupt0 input pin. */
#define SYS_GPF_MFPL_PF0MFP_ICE_DAT           (0x07UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< Serial wired debugger data pin. */
#define SYS_GPF_MFPL_PF1MFP_GPIO              (0x00UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPF_MFPL_PF1MFP_CLKO              (0x04UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< Output selected clock. */
#define SYS_GPF_MFPL_PF1MFP_INT1              (0x05UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< External interrupt1 input pin. */
#define SYS_GPF_MFPL_PF1MFP_ICE_CLK           (0x07UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< Serial wired debugger clock pin. */
#define SYS_GPF_MFPL_PF2MFP_GPIO              (0x00UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPF_MFPL_PF2MFP_XT1_OUT           (0x07UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< External 4~24 MHz (high speed) crystal output pin. */
#define SYS_GPF_MFPL_PF3MFP_GPIO              (0x00UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPF_MFPL_PF3MFP_XT1_IN            (0x07UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< External 4~24 MHz (high speed) crystal input pin. */
#define SYS_GPF_MFPL_PF6MFP_GPIO              (0x00UL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPF_MFPL_PF6MFP_I2C1_SDA          (0x01UL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< I2C1 data input/output pin. */
#define SYS_GPF_MFPL_PF6MFP_X32_OUT           (0x07UL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< External 32.768 kHz (low speed) crystal output pin. */
#define SYS_GPF_MFPL_PF7MFP_GPIO              (0x00UL<<SYS_GPF_MFPL_PF7MFP_Pos) /*!< General purpose digital I/O pin. */
#define SYS_GPF_MFPL_PF7MFP_I2C1_SCL          (0x01UL<<SYS_GPF_MFPL_PF7MFP_Pos) /*!< I2C1 clock pin. */
#define SYS_GPF_MFPL_PF7MFP_SC0_CD            (0x03UL<<SYS_GPF_MFPL_PF7MFP_Pos) /*!< SmartCard0 card detect pin. */
#define SYS_GPF_MFPL_PF7MFP_X32_IN            (0x07UL<<SYS_GPF_MFPL_PF7MFP_Pos) /*!< External 32.768 kHz (low speed) crystal input pin. */

/*@}*/ /* end of group NANO103_SYS_EXPORTED_CONSTANTS */

/** @addtogroup NANO103_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
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
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  */
#define SYS_ENABLE_BOD()              (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  */
#define SYS_DISABLE_BOD()             (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Low Power Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Low Power Brown-out detector function.
  */
#define SYS_ENABLE_LPBOD()              (SYS->BODCTL |= SYS_BODCTL_LPBODEN_Msk)

/**
  * @brief      Disable Low Power Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Low Power Brown-out detector function.
  */
#define SYS_DISABLE_LPBOD()             (SYS->BODCTL &= ~SYS_BODCTL_LPBODEN_Msk)

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
  */
#define SYS_DISABLE_BOD_RST()         (SYS->BODCTL &= ~SYS_BODCTL_BODREN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  */
#define SYS_ENABLE_BOD_RST()          (SYS->BODCTL |= SYS_BODCTL_BODREN_Msk)

/**
  * @brief      Disable Low Power Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low Power Brown-out detector reset function.
  */
#define SYS_DISABLE_LPBOD_RST()         (SYS->BODCTL &= ~SYS_BODCTL_LPBODREN_Msk)

/**
  * @brief      Enable Low Power Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low Power Brown-out detect reset function.
  */
#define SYS_ENABLE_LPBOD_RST()          (SYS->BODCTL |= SYS_BODCTL_LPBODREN_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_1_7V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  *             - \ref SYS_BODCTL_BODVL_1_9V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_2_1V
  *             - \ref SYS_BODCTL_BODVL_2_2V
  *             - \ref SYS_BODCTL_BODVL_2_3V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_5V
  *             - \ref SYS_BODCTL_BODVL_2_6V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_8V
  *             - \ref SYS_BODCTL_BODVL_2_9V
  *             - \ref SYS_BODCTL_BODVL_3_0V
  *             - \ref SYS_BODCTL_BODVL_3_1V
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
  * @brief      Get reset source is from Cortex-M0 lockup event.
  * @param      None
  * @retval     0   Previous reset source is not from Cortex-M0 lockup event
  * @retval     >=1 Previous reset source is from Cortex-M0 lockup event
  * @details    This macro get previous reset source is from Cortex-M0 lockup event.
  */
#define SYS_IS_LOCKUP_RST()             (SYS->RSTSTS & SYS_RSTSTS_LOCKRF_Msk)

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
  */
#define SYS_DISABLE_POR()               do{SYS->PORCTL = 0x5AA5;SYS->MISCCTL = SYS_MISCCTL_POR33DIS_Msk | SYS_MISCCTL_POR18DIS_Msk;}while(0)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  */
#define SYS_ENABLE_POR()                do{SYS->PORCTL = 0;SYS->MISCCTL = 0;}while(0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including:
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
    *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_LOCKRF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) (SYS->RSTSTS |= u32RstSrc)

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    while(SYS->REGLCTL != SYS_REGLCTL_REGLCTL_Msk)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }
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

/**
  * @brief      Get HIRC0 trim status
  * @param      None
  * @retval     BIT0 HIRC0 Frequency Lock
  * @retval     BIT1 Trim Failure Interrupt
  * @retval     BIT2 LXT Clock error
  * @details    This macro get HIRC0 trim interrupt status register.
  */
#define SYS_GET_IRC0TRIM_INT_FLAG()          (SYS->IRC0TISTS)

/**
  * @brief      Clear HIRC0 trim flag
  * @param[in]  u32IRCTrimFlg is HIRC0 trim flags. Including:
  *             - \ref SYS_IRCTISTS_FAIL_INT
  *             - \ref SYS_IRCTISTS_32KERR_INT
  * @return     None
  * @details    This macro clear HIRC0 trim flag.
  */
#define SYS_CLEAR_IRC0TRIM_INT_FLAG(u32IRCTrimFlg)          (SYS->IRC0TISTS = u32IRCTrimFlg)

/**
  * @brief      Get HIRC1 trim status
  * @param      None
  * @retval     BIT0 HIRC1 Frequency Lock
  * @retval     BIT1 Trim Failure Interrupt
  * @retval     BIT2 LXT Clock error
  * @details    This macro get HIRC1 trim interrupt status register.
  */
#define SYS_GET_IRC1TRIM_INT_FLAG()          (SYS->IRC1TISTS)

/**
  * @brief      Clear HIRC1 trim flag
  * @param[in]  u32IRCTrimFlg is HIRC1 trim flags. Including:
  *             - \ref SYS_IRCTISTS_FAIL_INT
  *             - \ref SYS_IRCTISTS_32KERR_INT
  * @return     None
  * @details    This macro clear HIRC1 trim flag.
  */
#define SYS_CLEAR_IRC1TRIM_INT_FLAG(u32IRCTrimFlg)          (SYS->IRC1TISTS = u32IRCTrimFlg)

/**
  * @brief      Get MIRC trim status
  * @param      None
  * @retval     BIT0 MIRC Frequency Lock
  * @retval     BIT1 Trim Failure Interrupt
  * @retval     BIT2 LXT Clock error
  * @details    This macro get MIRC trim interrupt status register.
  */
#define SYS_GET_MIRCTRIM_INT_FLAG()          (SYS->MIRCTISTS)

/**
  * @brief      Clear MIRC trim flag
  * @param[in]  u32IRCTrimFlg is MIRC trim flags. Including:
  *             - \ref SYS_IRCTISTS_FAIL_INT
  *             - \ref SYS_IRCTISTS_32KERR_INT
  * @return     None
  * @details    This macro clear MIRC trim flag.
  */
#define SYS_CLEAR_MIRCTRIM_INT_FLAG(u32IRCTrimFlg)          (SYS->MIRCTISTS = u32IRCTrimFlg)

void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_UnlockReg(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_EnableLPBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableLPBOD(void);
void SYS_EnableHIRC0Trim(uint32_t u32TrimSel,uint32_t u32TrimEnInt);
void SYS_DisableHIRC0Trim(void);
void SYS_EnableHIRC1Trim(uint32_t u32TrimSel,uint32_t u32TrimEnInt);
void SYS_DisableHIRC1Trim(void);
void SYS_EnableMIRCTrim(uint32_t u32TrimSel,uint32_t u32TrimEnInt);
void SYS_DisableMIRCTrim(void);
/*@}*/ /* end of group NANO103_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_SYS_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SYS_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
