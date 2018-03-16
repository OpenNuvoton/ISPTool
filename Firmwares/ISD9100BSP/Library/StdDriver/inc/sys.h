/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/17 11:40a $
 * @brief    ISD9100 Series SYS Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup ISD9100_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  RSTSTS constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_RSTSTS_CORERSTF      (0x1UL<<SYS_RSTSTS_CORERSTF_Pos)   /*!<Core reset flag  \hideinitializer */
#define  SYS_RSTSTS_WDTRF         (0x1UL<<SYS_RSTSTS_WDTRF_Pos)      /*!<Watch dog reset flag  \hideinitializer */
#define  SYS_RSTSTS_SYSRF         (0x1UL<<SYS_RSTSTS_SYSRF_Pos)      /*!<System reset flag  \hideinitializer */
#define  SYS_RSTSTS_PMURSTF       (0x1UL<<SYS_RSTSTS_PMURSTF_Pos)    /*!<Wake up reset flag  \hideinitializer */
#define  SYS_RSTSTS_CPURF         (0x1UL<<SYS_RSTSTS_CPURF_Pos)      /*!<CPU reset flag   \hideinitializer */
 	
/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CHIP_RST    ((0x0<<24) | SYS_IPRST0_CHIPRST_Pos  ) /*!<Reset CHIP  \hideinitializer */
#define CPU_RST     ((0x0<<24) | SYS_IPRST0_CPURST_Pos   ) /*!<Reset CPU  \hideinitializer  */
#define PDMA_RST    ((0x0<<24) | SYS_IPRST0_PDMARST_Pos  ) /*!<Reset PDMA  \hideinitializer  */
#define TMR0_RST    ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos  ) /*!<Reset TMR0  \hideinitializer  */
#define TMR1_RST    ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos  ) /*!<Reset TMR1  \hideinitializer  */
#define I2C0_RST    ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos  ) /*!<Reset I2C0  \hideinitializer  */
#define SPI0_RST    ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos  ) /*!<Reset SPI0  \hideinitializer  */
#define DPWM_RST    ((0x4<<24) | SYS_IPRST1_DPWMRST_Pos  ) /*!<Reset DPWM  \hideinitializer  */
#define UART0_RST   ((0x4<<24) | SYS_IPRST1_UART0RST_Pos ) /*!<Reset UART0  \hideinitializer  */
#define BIQ_RST     ((0x4<<24) | SYS_IPRST1_BIQRST_Pos   ) /*!<Reset BIQ  \hideinitializer  */
#define CRC_RST     ((0x4<<24) | SYS_IPRST1_CRCRST_Pos   ) /*!<Reset CRC  \hideinitializer  */
#define PWM0_RST    ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos  ) /*!<Reset PWM0  \hideinitializer  */
#define ACMP_RST    ((0x4<<24) | SYS_IPRST1_ACMPRST_Pos  ) /*!<Reset ACMP  \hideinitializer  */
#define EADC_RST    ((0x4<<24) | SYS_IPRST1_EADCRST_Pos  ) /*!<Reset EADC  \hideinitializer  */
#define I2S0_RST    ((0x4<<24) | SYS_IPRST1_I2S0RST_Pos  ) /*!<Reset I2S0  \hideinitializer  */
#define ANA_RST     ((0x4<<24) | SYS_IPRST1_ANARST_Pos   ) /*!<Reset ANA  \hideinitializer  */

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Input Type Control Resister constant definitions.                                                 */
/*---------------------------------------------------------------------------------------------------------*/

// GPIOA
#define SYS_PASMTEN_SMTEN16_GPIOA0     (0x1UL<<SYS_PASMTEN_SMTEN16_Pos)          /*!< PASMTEN SMTEN16 setting for GPIOA0 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN17_GPIOA1     (0x1UL<<SYS_PASMTEN_SMTEN17_Pos)          /*!< PASMTEN SMTEN17 setting for GPIOA1 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN18_GPIOA2     (0x1UL<<SYS_PASMTEN_SMTEN18_Pos)          /*!< PASMTEN SMTEN18 setting for GPIOA2 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN19_GPIOA3     (0x1UL<<SYS_PASMTEN_SMTEN19_Pos)          /*!< PASMTEN SMTEN19 setting for GPIOA3 input type   \hideinitializer */   
#define SYS_PASMTEN_SMTEN20_GPIOA4     (0x1UL<<SYS_PASMTEN_SMTEN20_Pos)          /*!< PASMTEN SMTEN20 setting for GPIOA4 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN21_GPIOA5     (0x1UL<<SYS_PASMTEN_SMTEN21_Pos)          /*!< PASMTEN SMTEN21 setting for GPIOA5 input type   \hideinitializer */   
#define SYS_PASMTEN_SMTEN22_GPIOA6     (0x1UL<<SYS_PASMTEN_SMTEN22_Pos)          /*!< PASMTEN SMTEN22 setting for GPIOA6 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN23_GPIOA7     (0x1UL<<SYS_PASMTEN_SMTEN23_Pos)          /*!< PASMTEN SMTEN23 setting for GPIOA7 input type   \hideinitializer */   
#define SYS_PASMTEN_SMTEN24_GPIOA8     (0x1UL<<SYS_PASMTEN_SMTEN24_Pos)          /*!< PASMTEN SMTEN24 setting for GPIOA8 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN25_GPIOA9     (0x1UL<<SYS_PASMTEN_SMTEN25_Pos)          /*!< PASMTEN SMTEN25 setting for GPIOA9 input type   \hideinitializer */   
#define SYS_PASMTEN_SMTEN26_GPIOA10    (0x1UL<<SYS_PASMTEN_SMTEN26_Pos)          /*!< PASMTEN SMTEN26 setting for GPIOA10 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN27_GPIOA11    (0x1UL<<SYS_PASMTEN_SMTEN27_Pos)          /*!< PASMTEN SMTEN27 setting for GPIOA11 input type   \hideinitializer */   
#define SYS_PASMTEN_SMTEN28_GPIOA12    (0x1UL<<SYS_PASMTEN_SMTEN28_Pos)          /*!< PASMTEN SMTEN28 setting for GPIOA12 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN29_GPIOA13    (0x1UL<<SYS_PASMTEN_SMTEN29_Pos)          /*!< PASMTEN SMTEN29 setting for GPIOA13 input type   \hideinitializer */   
#define SYS_PASMTEN_SMTEN30_GPIOA14    (0x1UL<<SYS_PASMTEN_SMTEN30_Pos)          /*!< PASMTEN SMTEN30 setting for GPIOA14 input type   \hideinitializer */     
#define SYS_PASMTEN_SMTEN31_GPIOA15    (0x1UL<<SYS_PASMTEN_SMTEN31_Pos)          /*!< PASMTEN SMTEN31 setting for GPIOA15 input type   \hideinitializer */   

// GPIOB
#define SYS_PBSMTEN_SMTEN16_GPIOB0     (0x1UL<<SYS_PBSMTEN_SMTEN16_Pos)          /*!< PBSMTEN SMTEN16 setting for GPIOB0 input type   \hideinitializer */     
#define SYS_PBSMTEN_SMTEN17_GPIOB1     (0x1UL<<SYS_PBSMTEN_SMTEN17_Pos)          /*!< PBSMTEN SMTEN17 setting for GPIOB1 input type   \hideinitializer */     
#define SYS_PBSMTEN_SMTEN18_GPIOB2     (0x1UL<<SYS_PBSMTEN_SMTEN18_Pos)          /*!< PBSMTEN SMTEN18 setting for GPIOB2 input type   \hideinitializer */     
#define SYS_PBSMTEN_SMTEN19_GPIOB3     (0x1UL<<SYS_PBSMTEN_SMTEN19_Pos)          /*!< PBSMTEN SMTEN19 setting for GPIOB3 input type   \hideinitializer */   
#define SYS_PBSMTEN_SMTEN20_GPIOB4     (0x1UL<<SYS_PBSMTEN_SMTEN20_Pos)          /*!< PBSMTEN SMTEN20 setting for GPIOB4 input type   \hideinitializer */     
#define SYS_PBSMTEN_SMTEN21_GPIOB5     (0x1UL<<SYS_PBSMTEN_SMTEN21_Pos)          /*!< PBSMTEN SMTEN21 setting for GPIOB5 input type   \hideinitializer */   
#define SYS_PBSMTEN_SMTEN22_GPIOB6     (0x1UL<<SYS_PBSMTEN_SMTEN22_Pos)          /*!< PBSMTEN SMTEN22 setting for GPIOB6 input type   \hideinitializer */     
#define SYS_PBSMTEN_SMTEN23_GPIOB7     (0x1UL<<SYS_PBSMTEN_SMTEN23_Pos)          /*!< PBSMTEN SMTEN23 setting for GPIOB7 input type   \hideinitializer */   

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as SPI_MOSI0 in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_SPI_MOSI0  ;

*/
//GPA_MFP_PA0MFP
#define SYS_GPA_MFP_PA0MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_SPI_MOSI0   (0x1UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for SPI_MOSI0   \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_MCLK        (0x2UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for MCLK   \hideinitializer */

//GPA_MFP_PA1MFP
#define SYS_GPA_MFP_PA1MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_SPI_SCLK    (0x1UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for SPI_SCLK   \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_I2C_SCL     (0x2UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for I2C_SCL   \hideinitializer */

//GPA_MFP_PA2MFP
#define SYS_GPA_MFP_PA2MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_SPI_SSB0    (0x1UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for SPI_SSB0   \hideinitializer */

//GPA_MFP_PA3MFP
#define SYS_GPA_MFP_PA3MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_SPI_MISO0   (0x1UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for SPI_MISO0   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_I2C_SDA     (0x2UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for I2C_SDA   \hideinitializer */

//GPA_MFP_PA4MFP
#define SYS_GPA_MFP_PA4MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_I2S_FS      (0x1UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for I2S_FS   \hideinitializer */

//GPA_MFP_PA5MFP
#define SYS_GPA_MFP_PA5MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_I2S_BCLK    (0x1UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for I2S_BCLK   \hideinitializer */

//GPA_MFP_PA6MFP
#define SYS_GPA_MFP_PA6MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_I2S_SDI     (0x1UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for I2S_SDI   \hideinitializer */

//GPA_MFP_PA7MFP
#define SYS_GPA_MFP_PA7MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_I2S_SDO     (0x1UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for I2S_SDO   \hideinitializer */

//GPA_MFP_PA8MFP
#define SYS_GPA_MFP_PA8MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_UART_TX     (0x1UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for UART_TX   \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_I2S_FS      (0x2UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for I2S_FS   \hideinitializer */

//GPA_MFP_PA9MFP
#define SYS_GPA_MFP_PA9MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_UART_RX     (0x1UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for UART_RX   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_I2S_BCLK    (0x2UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for I2S_BCLK   \hideinitializer */

//GPA_MFP_PA10MFP
#define SYS_GPA_MFP_PA10MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_I2C_SDA    (0x1UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for I2C_SDA   \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_I2S_SDI    (0x2UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for I2S_SDI   \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_UART_RTSn  (0x3UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for UART_RTSn   \hideinitializer */

//GPA_MFP_PA11MFP
#define SYS_GPA_MFP_PA11MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_I2C_SCL    (0x1UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for I2C_SCL   \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_I2S_SDO    (0x2UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for I2S_SDO   \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_UART_CTSn  (0x3UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for UART_CTSn   \hideinitializer */

//GPA_MFP_PA12MFP
#define SYS_GPA_MFP_PA12MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_PWM0CH0    (0x1UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for PWM0CH0   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_SPKP       (0x2UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for SPKP   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_I2S_FS     (0x3UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for I2S_FS   \hideinitializer */

//GPA_MFP_PA13MFP
#define SYS_GPA_MFP_PA13MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_PWM0CH1    (0x1UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for PWM0CH1   \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_SPKM       (0x2UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for SPKM   \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_I2S_BCLK   (0x3UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for I2S_BCLK   \hideinitializer */

//GPA_MFP_PA14MFP
#define SYS_GPA_MFP_PA14MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_TM0        (0x1UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for TM0   \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_SDCLK      (0x2UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for SDCLK   \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_SDCLKn     (0x3UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for SDCLKn   \hideinitializer */

//GPA_MFP_PA15MFP
#define SYS_GPA_MFP_PA15MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_TM1        (0x1UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for TM1   \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_SDIN       (0x2UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for SDIN   \hideinitializer */

//GPB_MFP_PB0MFP
#define SYS_GPB_MFP_PB0MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_SPI_SSB1    (0x1UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for SPI_SSB1   \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_CMP0        (0x2UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for CMP0   \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_SPI_SSB0    (0x3UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for SPI_SSB0   \hideinitializer */

//GPB_MFP_PB1MFP
#define SYS_GPB_MFP_PB1MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_MCLK        (0x1UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for MCLK   \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_CMP1        (0x2UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for CMP1   \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_SPI_SSB1    (0x3UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for SPI_SSB1   \hideinitializer */

//GPB_MFP_PB2MFP
#define SYS_GPB_MFP_PB2MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_I2C_SCL     (0x1UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for I2C_SCL   \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_CMP2        (0x2UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for CMP2   \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_SPI_SCLK    (0x3UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for SPI_SCLK   \hideinitializer */

//GPB_MFP_PB3MFP
#define SYS_GPB_MFP_PB3MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_I2C_SDA     (0x1UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for I2C_SDA   \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_CMP3        (0x2UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for CMP3   \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_SPI_MISO0   (0x3UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for SPI_MISO0   \hideinitializer */

//GPB_MFP_PB4MFP
#define SYS_GPB_MFP_PB4MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_PWM0CH0_INV (0x1UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for PWM0CH0_INV   \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_CMP4        (0x2UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for CMP3   \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_SPI_MOSI0   (0x3UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for SPI_MOSI0   \hideinitializer */

//GPB_MFP_PB5MFP
#define SYS_GPB_MFP_PB5MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_PWM0CH1_INV (0x1UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for PWM0CH1_INV   \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_CMP5        (0x2UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for CMP5   \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_SPI_MISO1   (0x3UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for SPI_MISO1   \hideinitializer */

//GPB_MFP_PB6MFP
#define SYS_GPB_MFP_PB6MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB6MFP_Pos)           /*!< GPB_MFP PB6 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB6MFP_I2S_SDI     (0x1UL<<SYS_GPB_MFP_PB6MFP_Pos)           /*!< GPB_MFP PB6 setting for I2S_SDI   \hideinitializer */
#define SYS_GPB_MFP_PB6MFP_CMP6        (0x2UL<<SYS_GPB_MFP_PB6MFP_Pos)           /*!< GPB_MFP PB6 setting for CMP6   \hideinitializer */
#define SYS_GPB_MFP_PB6MFP_SPI_MOSI1   (0x3UL<<SYS_GPB_MFP_PB6MFP_Pos)           /*!< GPB_MFP PB6 setting for SPI_MOSI1   \hideinitializer */

//GPB_MFP_PB7MFP
#define SYS_GPB_MFP_PB7MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB7MFP_Pos)           /*!< GPB_MFP PB7 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB7MFP_I2S_SDO     (0x1UL<<SYS_GPB_MFP_PB7MFP_Pos)           /*!< GPB_MFP PB7 setting for I2S_SDO   \hideinitializer */
#define SYS_GPB_MFP_PB7MFP_CMP7        (0x2UL<<SYS_GPB_MFP_PB7MFP_Pos)           /*!< GPB_MFP PB7 setting for CMP7   \hideinitializer */

/*@}*/ /* end of group ISD9100_SYS_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

#define SYS_IS_CPU_RST()                   (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)   /*!< This macro get previous reset source is from CPU-Reset   \hideinitializer */
#define SYS_IS_SYSTEM_RST()                (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)   /*!< This macro get previous reset source is from system reset   \hideinitializer */
#define SYS_IS_WDT_RST()                   (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)   /*!< This macro get previous reset source is from window watch dog  reset   \hideinitializer */
#define SYS_CLEAR_RST_SOURCE(u32RSTSTS)    (SYS->RSTSTS = u32RSTSTS )             /*!< This macro clears reset source   \hideinitializer */

#define SYS_ENABLE_SCHMITT_GPIOA(u32Bit)   (SYS->PASMTEN | u32Bit)                /*!< This macro enable GPIOA(0-15) schmitt mode   \hideinitializer */
#define SYS_DISABLE_SCHMITT_GPIOA(u32Bit)  (SYS->PASMTEN & (~u32Bit))             /*!< This macro disable GPIOA(0-15) schmitt mode   \hideinitializer */
#define SYS_ENABLE_SCHMITT_GPIOB(u32Bit)   (SYS->PBSMTEN | u32Bit)                /*!< This macro enable GPIOB(0-7) schmitt mode   \hideinitializer */
#define SYS_DISABLE_SCHMITT_GPIOB(u32Bit)  (SYS->PBSMTEN & (~u32Bit))             /*!< This macro disable GPIOB(0-7) schmitt mode   \hideinitializer */

void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_Lock(uint8_t u8Lock);
void SYS_UnlockReg(void);
uint8_t SYS_Unlock(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);

/*@}*/ /* end of group ISD9100_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_SYS_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SYS_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
