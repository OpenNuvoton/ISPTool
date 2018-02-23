/**************************************************************************//**
 * @file     sys.h
 * @version  V1.01
 * $Revision: 2 $
 * $Date: 16/08/25 11:40a $
 * @brief    I91200 Series SYS Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup I91200_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  RSTSTS constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_RSTSTS_CORERSTF      (SYS_RSTSTS_CORERSTF_Msk)   /*!<Core reset flag  \hideinitializer */
#define  SYS_RSTSTS_PADRF         (SYS_RSTSTS_PADRF_Msk)      /*!</RESET Pin reset flag  \hideinitializer */
#define  SYS_RSTSTS_WDTRF         (SYS_RSTSTS_WDTRF_Msk)      /*!<Watch dog reset flag  \hideinitializer */
#define  SYS_RSTSTS_LVRF          (SYS_RSTSTS_LVRF_Msk)       /*!<Low Voltage Reset flag  \hideinitializer */
#define  SYS_RSTSTS_SYSRF         (SYS_RSTSTS_SYSRF_Msk)      /*!<System reset flag  \hideinitializer */
#define  SYS_RSTSTS_PMURSTF       (SYS_RSTSTS_PMURSTF_Msk)    /*!<Wake up reset flag  \hideinitializer */
#define  SYS_RSTSTS_CPURF         (SYS_RSTSTS_CPURF_Msk)      /*!<CPU reset flag   \hideinitializer */
#define  SYS_RSTSTS_WKRSTF        (SYS_RSTSTS_WKRSTF_Msk)     /*!<Wakeup Pin Reset Flag   \hideinitializer */
#define  SYS_RSTSTS_DPDRSTF       (SYS_RSTSTS_DPDRSTF_Msk)    /*!<Deep Power Down Reset Flag   \hideinitializer */
#define  SYS_RSTSTS_PORF          (SYS_RSTSTS_PORF_Msk)       /*!<Power on Reset Flag   \hideinitializer */
 	
/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CHIP_RST    ((0x0<<24) | SYS_IPRST0_CHIPRST_Pos  ) /*!<Reset CHIP  \hideinitializer */
#define CPU_RST     ((0x0<<24) | SYS_IPRST0_CPURST_Pos   ) /*!<Reset CPU  \hideinitializer  */
#define PDMA_RST    ((0x0<<24) | SYS_IPRST0_PDMARST_Pos  ) /*!<Reset PDMA  \hideinitializer  */

#define RTC_RST     ((0x4<<24) | SYS_IPRST1_RTCRST_Pos   ) /*!<Reset RTC  \hideinitializer  */
#define TMR0_RST    ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos  ) /*!<Reset TMR0  \hideinitializer  */
#define TMR1_RST    ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos  ) /*!<Reset TMR1  \hideinitializer  */
#define I2C0_RST    ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos  ) /*!<Reset I2C0  \hideinitializer  */
#define SPI0_RST    ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos  ) /*!<Reset SPI0  \hideinitializer  */
#define SPI1_RST    ((0x4<<24) | SYS_IPRST1_SPI1RST_Pos  ) /*!<Reset SPI1  \hideinitializer  */
#define DPWM_RST    ((0x4<<24) | SYS_IPRST1_DPWMRST_Pos  ) /*!<Reset DPWM  \hideinitializer  */
#define UART0_RST   ((0x4<<24) | SYS_IPRST1_UART0RST_Pos ) /*!<Reset UART0  \hideinitializer  */
#define UART1_RST   ((0x4<<24) | SYS_IPRST1_UART1RST_Pos ) /*!<Reset UART1  \hideinitializer  */
#define BIQ_RST     ((0x4<<24) | SYS_IPRST1_BIQRST_Pos   ) /*!<Reset BIQ  \hideinitializer  */
#define PWM0_RST    ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos  ) /*!<Reset PWM0  \hideinitializer  */
#define SARADC_RST  ((0x4<<24) | SYS_IPRST1_SARADCRST_Pos  ) /*!<Reset SARADC  \hideinitializer  */
#define SDADC_RST   ((0x4<<24) | SYS_IPRST1_SDADCRST_Pos  ) /*!<Reset SDADC \hideinitializer  */
#define I2S0_RST    ((0x4<<24) | SYS_IPRST1_I2S0RST_Pos  ) /*!<Reset I2S0  \hideinitializer  */
#define ANA_RST     ((0x4<<24) | SYS_IPRST1_ANARST_Pos   ) /*!<Reset ANA  \hideinitializer  */


/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Input Type Control Resister constant definitions.                                                 */
/*---------------------------------------------------------------------------------------------------------*/

// GPIOA
#define SYS_GPSMTEN_SSGPAG0     (0x1UL<<SYS_GPSMTEN_SSGPAG0_Pos)          /*!< GPIOA 3/2/1/0 input Schmitt Trigger enabled   \hideinitializer */     
#define SYS_GPSMTEN_SSGPAG1     (0x1UL<<SYS_GPSMTEN_SSGPAG1_Pos)          /*!< GPIOA 7/6/5/4 input Schmitt Trigger enabled   \hideinitializer */     
#define SYS_GPSMTEN_SSGPAG2     (0x1UL<<SYS_GPSMTEN_SSGPAG2_Pos)          /*!< GPIOA 11/10/9/8 input Schmitt Trigger enabled   \hideinitializer */     
#define SYS_GPSMTEN_SSGPAG3     (0x1UL<<SYS_GPSMTEN_SSGPAG3_Pos)          /*!< GPIOA 15/14/13/12 input Schmitt Trigger enabled   \hideinitializer */     

#define SYS_GPSMTEN_HSSGPAG0    (0x1UL<<SYS_GPSMTEN_HSSGPAG0_Pos)         /*!< GPIOA 3/2/1/0 high slew rate.  \hideinitializer */ 
#define SYS_GPSMTEN_HSSGPAG1    (0x1UL<<SYS_GPSMTEN_HSSGPAG1_Pos)         /*!< GPIOA 7/6/5/4 high slew rate.  \hideinitializer */ 
#define SYS_GPSMTEN_HSSGPAG2    (0x1UL<<SYS_GPSMTEN_HSSGPAG2_Pos)         /*!< GPIOA 11/10/9/8 high slew rate.  \hideinitializer */ 
#define SYS_GPSMTEN_HSSGPAG3    (0x1UL<<SYS_GPSMTEN_HSSGPAG3_Pos)         /*!< GPIOA 15/14/13/12 high slew rate.  \hideinitializer */ 

// GPIOB
#define SYS_GPSMTEN_SSGPBG0     (0x1UL<<SYS_GPSMTEN_SSGPBG0_Pos)          /*!< GPIOB 3/2/1/0 input Schmitt Trigger enabled   \hideinitializer */     
#define SYS_GPSMTEN_SSGPBG1     (0x1UL<<SYS_GPSMTEN_SSGPBG1_Pos)          /*!< GPIOB 7/6/5/4 input Schmitt Trigger enabled   \hideinitializer */     
#define SYS_GPSMTEN_SSGPBG2     (0x1UL<<SYS_GPSMTEN_SSGPBG2_Pos)          /*!< GPIOB 11/10/9/8 input Schmitt Trigger enabled   \hideinitializer */     
#define SYS_GPSMTEN_SSGPBG3     (0x1UL<<SYS_GPSMTEN_SSGPBG3_Pos)          /*!< GPIOB 15/14/13/12 input Schmitt Trigger enabled   \hideinitializer */     

#define SYS_GPSMTEN_HSSGPBG0    (0x1UL<<SYS_GPSMTEN_HSSGPBG0_Pos)         /*!< GPIOB 3/2/1/0 high slew rate.  \hideinitializer */ 
#define SYS_GPSMTEN_HSSGPBG1    (0x1UL<<SYS_GPSMTEN_HSSGPBG1_Pos)         /*!< GPIOB 7/6/5/4 high slew rate.  \hideinitializer */ 
#define SYS_GPSMTEN_HSSGPBG2    (0x1UL<<SYS_GPSMTEN_HSSGPBG2_Pos)         /*!< GPIOB 11/10/9/8 high slew rate.  \hideinitializer */ 
#define SYS_GPSMTEN_HSSGPBG3    (0x1UL<<SYS_GPSMTEN_H1SGPBG3_Pos)         /*!< GPIOB 15/14/13/12 high slew rate.  \hideinitializer */ 

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function (Digital) constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as SYS_GPA_MFP_PA0MFP_SPI0_MISO1 in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_SPI0_MISO1  ;

*/
//GPA_MFP_PA0MFP
#define SYS_GPA_MFP_PA0MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_SPI0_MISO1  (0x1UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for SPI0_MISO1   \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_I2S0_FS	   (0x3UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for I2S0_FS   \hideinitializer */

//GPA_MFP_PA1MFP
#define SYS_GPA_MFP_PA1MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_SPI0_MOSI0  (0x1UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for SPI0_MOSI0   \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_I2S0_BCLK   (0x3UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for I2S0_BCLK   \hideinitializer */

//GPA_MFP_PA2MFP
#define SYS_GPA_MFP_PA2MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_SPI0_SCLK   (0x1UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for SPI0_SCLK   \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_DMIC_DAT    (0x2UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for DMIC_DAT   \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_I2S0_SDI    (0x3UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA1 setting for I2S0_SDI   \hideinitializer */

//GPA_MFP_PA3MFP
#define SYS_GPA_MFP_PA3MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_SPI0_SSB    (0x1UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for SPI0_SSB   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_SARADC_TRIG (0x2UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for SARADC_TRIG   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_I2S0_SDO    (0x3UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for I2S0_SDO   \hideinitializer */

//GPA_MFP_PA4MFP
#define SYS_GPA_MFP_PA4MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_SPI0_MISO0  (0x1UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for SPI0_MISO0   \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_UART0_TX    (0x2UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for UART0_TX   \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_SPI1_MOSI   (0x3UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for SPI1_MOSI   \hideinitializer */

//GPA_MFP_PA5MFP
#define SYS_GPA_MFP_PA5MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_SPI0_MOSI1  (0x1UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for SPI0_MOSI1   \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_UART0_RX    (0x2UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for UART0_RX   \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_SPI1_SCLK   (0x3UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for SPI1_SCLK   \hideinitializer */

//GPA_MFP_PA6MFP
#define SYS_GPA_MFP_PA6MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_UART0_TX    (0x1UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for UART0_TX   \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_I2C0_SDA    (0x2UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for I2C0_SDA   \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_SPI1_SSB    (0x3UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for SPI1_SSB   \hideinitializer */

//GPA_MFP_PA7MFP
#define SYS_GPA_MFP_PA7MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_UART0_RX    (0x1UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for UART0_RX   \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_I2C0_SCL    (0x2UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for I2C0_SCL   \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_SPI1_MISO   (0x3UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for SPI1_MISO   \hideinitializer */

//GPA_MFP_PA8MFP
#define SYS_GPA_MFP_PA8MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_I2C0_SDA    (0x1UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for I2C0_SDA   \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_UART1_TX    (0x2UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for UART1_TX   \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_UART0_RTSn  (0x3UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for UART0_RTSn   \hideinitializer */

//GPA_MFP_PA9MFP
#define SYS_GPA_MFP_PA9MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_I2C0_SCL    (0x1UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for I2C0_SCL   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_UART1_RX    (0x2UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for UART1_RX   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_UART0_CTSn  (0x3UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for UART0_CTSn   \hideinitializer */

//GPA_MFP_PA10MFP
#define SYS_GPA_MFP_PA10MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_PWM0CH0    (0x1UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for PWM0CH0   \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_TM0        (0x2UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for TM0   \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_SPKP       (0x3UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for SPKP   \hideinitializer */

//GPA_MFP_PA11MFP
#define SYS_GPA_MFP_PA11MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_PWM0CH1    (0x1UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for PWM0CH1   \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_TM1        (0x2UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for TM1    \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_SPKM       (0x3UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for SPKM   \hideinitializer */

//GPA_MFP_PA12MFP
#define SYS_GPA_MFP_PA12MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_PWM0CH2    (0x1UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for PWM0CH2   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_X12MI      (0x2UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for X12MI   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_I2C0_SDA   (0x3UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for I2C0_SDA   \hideinitializer */

//GPA_MFP_PA13MFP
#define SYS_GPA_MFP_PA13MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_PWM0CH3    (0x1UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for PWM0CH3   \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_X12MO      (0x2UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for X12MO   \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_I2C0_SCL   (0x3UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for I2C0_SCL   \hideinitializer */

//GPA_MFP_PA14MFP
#define SYS_GPA_MFP_PA14MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_UART1_TX   (0x1UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for UART1_TX   \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_DMIC_CLK   (0x2UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for DMIC_CLK   \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_X32KI      (0x3UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for X32KI   \hideinitializer */

//GPA_MFP_PA15MFP
#define SYS_GPA_MFP_PA15MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for GPIO   \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_UART1_RX   (0x1UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for UART1_RX   \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_MCLK       (0x2UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for MCLK   \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_X32KO      (0x3UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for X32KO   \hideinitializer */

//GPB_MFP_PB0MFP
#define SYS_GPB_MFP_PB0MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_SPI1_MOSI   (0x1UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for SPI1_MOSI   \hideinitializer */

//GPB_MFP_PB1MFP
#define SYS_GPB_MFP_PB1MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_SPI1_SCLK   (0x1UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for SPI1_SCLK   \hideinitializer */

//GPB_MFP_PB2MFP
#define SYS_GPB_MFP_PB2MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_SPI1_SSB    (0x1UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for SPI1_SSB   \hideinitializer */

//GPB_MFP_PB3MFP
#define SYS_GPB_MFP_PB3MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_SPI1_MISO   (0x1UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for SPI1_MISO   \hideinitializer */

//GPB_MFP_PB4MFP
#define SYS_GPB_MFP_PB4MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_I2S0_FS     (0x1UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for I2S0_FS   \hideinitializer */

//GPB_MFP_PB5MFP
#define SYS_GPB_MFP_PB5MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_I2S0_BCLK   (0x1UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for I2S0_BCLK   \hideinitializer */

//GPB_MFP_PB6MFP
#define SYS_GPB_MFP_PB6MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB6MFP_Pos)           /*!< GPB_MFP PB6 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB6MFP_I2S0_SDI    (0x1UL<<SYS_GPB_MFP_PB6MFP_Pos)           /*!< GPB_MFP PB6 setting for I2S0_SDI   \hideinitializer */

//GPB_MFP_PB7MFP
#define SYS_GPB_MFP_PB7MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB7MFP_Pos)           /*!< GPB_MFP PB7 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB7MFP_I2S_SDO     (0x1UL<<SYS_GPB_MFP_PB7MFP_Pos)           /*!< GPB_MFP PB7 setting for I2S_SDO   \hideinitializer */

//GPB_MFP_PB8MFP
#define SYS_GPB_MFP_PB8MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB8MFP_Pos)           /*!< GPB_MFP PB8 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB8MFP_I2C0_SDA    (0x1UL<<SYS_GPB_MFP_PB8MFP_Pos)           /*!< GPB_MFP PB8 setting for I2C0_SDA   \hideinitializer */
#define SYS_GPB_MFP_PB8MFP_I2S0_FS     (0x2UL<<SYS_GPB_MFP_PB8MFP_Pos)           /*!< GPB_MFP PB8 setting for I2S0_FS   \hideinitializer */
#define SYS_GPB_MFP_PB8MFP_UART1_RSTn  (0x3UL<<SYS_GPB_MFP_PB8MFP_Pos)           /*!< GPB_MFP PB8 setting for UART1_RSTn   \hideinitializer */

//GPB_MFP_PB9MFP
#define SYS_GPB_MFP_PB9MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB9MFP_Pos)           /*!< GPB_MFP PB9 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB9MFP_I2C0_SCL    (0x1UL<<SYS_GPB_MFP_PB9MFP_Pos)           /*!< GPB_MFP PB9 setting for I2C0_SCL   \hideinitializer */
#define SYS_GPB_MFP_PB9MFP_I2S0_BCLK   (0x2UL<<SYS_GPB_MFP_PB9MFP_Pos)           /*!< GPB_MFP PB9 setting for I2S0_BCLK   \hideinitializer */
#define SYS_GPB_MFP_PB9MFP_UART1_CTsn  (0x3UL<<SYS_GPB_MFP_PB9MFP_Pos)           /*!< GPB_MFP PB9 setting for UART1_CTsn   \hideinitializer */

//GPB_MFP_PB10MFP
#define SYS_GPB_MFP_PB10MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB10MFP_Pos)           /*!< GPB_MFP PB10 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB10MFP_I2S0_SDI    (0x2UL<<SYS_GPB_MFP_PB10MFP_Pos)           /*!< GPB_MFP PB10 setting for I2S0_SDI   \hideinitializer */
#define SYS_GPB_MFP_PB10MFP_UART1_TX    (0x3UL<<SYS_GPB_MFP_PB10MFP_Pos)           /*!< GPB_MFP PB10 setting for UART1_TX   \hideinitializer */

//GPB_MFP_PB11MFP
#define SYS_GPB_MFP_PB11MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB11MFP_Pos)           /*!< GPB_MFP PB11 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB11MFP_I2S0_SDO    (0x2UL<<SYS_GPB_MFP_PB11MFP_Pos)           /*!< GPB_MFP PB11 setting for I2S0_SDO   \hideinitializer */
#define SYS_GPB_MFP_PB11MFP_UART1_RX    (0x3UL<<SYS_GPB_MFP_PB11MFP_Pos)           /*!< GPB_MFP PB11 setting for UART1_RX   \hideinitializer */

//GPB_MFP_PB12MFP
#define SYS_GPB_MFP_PB12MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB12MFP_Pos)           /*!< GPB_MFP PB12 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB12MFP_SP0_MISO1   (0x1UL<<SYS_GPB_MFP_PB12MFP_Pos)           /*!< GPB_MFP PB12 setting for SP0_MISO1   \hideinitializer */
#define SYS_GPB_MFP_PB12MFP_SPI1_MOSI   (0x2UL<<SYS_GPB_MFP_PB12MFP_Pos)           /*!< GPB_MFP PB12 setting for SPI1_MOSI   \hideinitializer */
#define SYS_GPB_MFP_PB12MFP_DMIC_DAT    (0x3UL<<SYS_GPB_MFP_PB12MFP_Pos)           /*!< GPB_MFP PB12 setting for DMIC_DAT   \hideinitializer */

//GPB_MFP_PB13MFP
#define SYS_GPB_MFP_PB13MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB13MFP_Pos)           /*!< GPB_MFP PB13 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB13MFP_SPI0_MOSI0  (0x1UL<<SYS_GPB_MFP_PB13MFP_Pos)           /*!< GPB_MFP PB13 setting for SPI0_MOSI0   \hideinitializer */
#define SYS_GPB_MFP_PB13MFP_SPI1_SCLK   (0x2UL<<SYS_GPB_MFP_PB13MFP_Pos)           /*!< GPB_MFP PB13 setting for SPI1_SCLK   \hideinitializer */
#define SYS_GPB_MFP_PB13MFP_SARADC_TRIG (0x3UL<<SYS_GPB_MFP_PB13MFP_Pos)           /*!< GPB_MFP PB13 setting for SARADC_TRIG   \hideinitializer */

//GPB_MFP_PB14MFP
#define SYS_GPB_MFP_PB14MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB14MFP_Pos)           /*!< GPB_MFP PB14 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB14MFP_SPI0_SCLK   (0x1UL<<SYS_GPB_MFP_PB14MFP_Pos)           /*!< GPB_MFP PB14 setting for SPI0_SCLK   \hideinitializer */
#define SYS_GPB_MFP_PB14MFP_SPI1_SSB    (0x2UL<<SYS_GPB_MFP_PB14MFP_Pos)           /*!< GPB_MFP PB14 setting for SPI1_SSB   \hideinitializer */
#define SYS_GPB_MFP_PB14MFP_DMIC_CLK    (0x3UL<<SYS_GPB_MFP_PB14MFP_Pos)           /*!< GPB_MFP PB14 setting for DMIC_CLK   \hideinitializer */

//GPB_MFP_PB15MFP
#define SYS_GPB_MFP_PB15MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB15MFP_Pos)           /*!< GPB_MFP PB15 setting for GPIO   \hideinitializer */
#define SYS_GPB_MFP_PB15MFP_SPI0_SSB    (0x1UL<<SYS_GPB_MFP_PB15MFP_Pos)           /*!< GPB_MFP PB15 setting for SPI0_SSB   \hideinitializer */
#define SYS_GPB_MFP_PB15MFP_SPI1_MISO   (0x2UL<<SYS_GPB_MFP_PB15MFP_Pos)           /*!< GPB_MFP PB15 setting for SPI1_MISO   \hideinitializer */
#define SYS_GPB_MFP_PB15MFP_MCLK        (0x3UL<<SYS_GPB_MFP_PB15MFP_Pos)           /*!< GPB_MFP PB15 setting for MCLK   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Analog-Function constant definitions.                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_GPB0_ANALOG_FUNCTION		(BIT0)				 /*!< GPB0 Analog pin for CS0 and A0P   \hideinitializer */
#define SYS_GPB1_ANALOG_FUNCTION		(BIT1)				 /*!< GPB1 Analog pin for CS1 and A0N   \hideinitializer */
#define SYS_GPB2_ANALOG_FUNCTION		(BIT2)				 /*!< GPB2 Analog pin for CS2 and A0E   \hideinitializer */
#define SYS_GPB3_ANALOG_FUNCTION		(BIT3)				 /*!< GPB3 Analog pin for CS3 and A1P   \hideinitializer */
#define SYS_GPB4_ANALOG_FUNCTION		(BIT4)				 /*!< GPB4 Analog pin for CS4 and A1N   \hideinitializer */
#define SYS_GPB5_ANALOG_FUNCTION		(BIT5)				 /*!< GPB5 Analog pin for CS5 A1E nd SAR10   \hideinitializer */
#define SYS_GPB6_ANALOG_FUNCTION		(BIT6)				 /*!< GPB6 Analog pin for CS6, CNP and SAR8   \hideinitializer */
#define SYS_GPB7_ANALOG_FUNCTION		(BIT7)				 /*!< GPB7 Analog pin for CS7, C1P and SAR9   \hideinitializer */
#define SYS_GPB8_ANALOG_FUNCTION		(BIT8)				 /*!< GPB8 Analog pin for CS8, C2P and SAR0   \hideinitializer */
#define SYS_GPB9_ANALOG_FUNCTION		(BIT9)				 /*!< GPB9 Analog pin for CS9 and SAR1   \hideinitializer */
#define SYS_GPB10_ANALOG_FUNCTION		(BIT10)				 /*!< GPB10 Analog pin for CS10 and SAR2   \hideinitializer */
#define SYS_GPB11_ANALOG_FUNCTION		(BIT11)				 /*!< GPB11 Analog pin for CS11 and SAR3   \hideinitializer */
#define SYS_GPB12_ANALOG_FUNCTION		(BIT12)				 /*!< GPB12 Analog pin for CS12 and SAR4   \hideinitializer */
#define SYS_GPB13_ANALOG_FUNCTION		(BIT13)				 /*!< GPB13 Analog pin for CS13 and SAR5   \hideinitializer */
#define SYS_GPB14_ANALOG_FUNCTION		(BIT14)				 /*!< GPB14 Analog pin for CS14 and SAR6   \hideinitializer */
#define SYS_GPB15_ANALOG_FUNCTION		(BIT15)				 /*!< GPB15 Analog pin for CS15 and SAR7   \hideinitializer */


/*@}*/ /* end of group I91200_SYS_EXPORTED_CONSTANTS */

/** @addtogroup I91200_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

#define SYS_IS_CPU_RST()                   (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)   /*!< This macro get previous reset source is from CPU-Reset   \hideinitializer */
#define SYS_IS_SYSTEM_RST()                (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)   /*!< This macro get previous reset source is from system reset   \hideinitializer */
#define SYS_IS_WDT_RST()                   (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)   /*!< This macro get previous reset source is from window watch dog  reset   \hideinitializer */
#define SYS_CLEAR_RST_SOURCE(u32RSTSTS)    (SYS->RSTSTS = u32RSTSTS )             /*!< This macro clears reset source   \hideinitializer */

#define SYS_ENABLE_SCHMITT_GPIOA(u32Group)   (SYS->GPSMTEN | u32Group)              /*!< This macro enable schmitt mode for SYS_GPSMTEN_SSGPAG0~3 groups    \hideinitializer */
#define SYS_DISABLE_SCHMITT_GPIOA(u32Group)  (SYS->GPSMTEN & (~u32Group))           /*!< This macro disable schmitt mode for SYS_GPSMTEN_SSGPAG0~3 groups    \hideinitializer */
#define SYS_ENABLE_SCHMITT_GPIOB(u32Group)   (SYS->GPSMTEN | u32Group)              /*!< This macro enable schmitt mode for SYS_GPSMTEN_SSGPBG0~3 groups    \hideinitializer */
#define SYS_DISABLE_SCHMITT_GPIOB(u32Group)  (SYS->GPSMTEN & (~u32Group))           /*!< This macro disable schmitt mode for SYS_GPSMTEN_SSGPBG0~3 groups    \hideinitializer */

#define SYS_ENABLE_SCHMITT_HIGH_SLEW(u32Group)   (SYS->GPSMTEN | u32Group)          /*!< This macro enable high slew rate in schmitt mode for SYS_GPSMTEN_HSSGPAG0~3 or SYS_GPSMTEN_HSSGPBG0~3 groups\hideinitializer */
#define SYS_DISABLE_SCHMITT_HIGH_SLEW(u32Group)   (SYS->GPSMTEN & (~u32Group))      /*!< This macro disable high slew rate in schmitt mode for SYS_GPSMTEN_HSSGPAG0~3 or SYS_GPSMTEN_HSSGPBG0~3 groups\hideinitializer */

/**
  * @brief Enable GPIO analog function.
  * @param[in] u32PinFuns corresponding GPIO pins
  *            - \ref SYS_GPB0_ANALOG_FUNCTION 
  *            - \ref SYS_GPB1_ANALOG_FUNCTION 
  *            - \ref SYS_GPB2_ANALOG_FUNCTION 
  *            - \ref SYS_GPB3_ANALOG_FUNCTION 
  *            - \ref SYS_GPB4_ANALOG_FUNCTION 
  *            - \ref SYS_GPB5_ANALOG_FUNCTION 
  *            - \ref SYS_GPB6_ANALOG_FUNCTION 
  *            - \ref SYS_GPB7_ANALOG_FUNCTION 
  *            - \ref SYS_GPB8_ANALOG_FUNCTION 
  *            - \ref SYS_GPB9_ANALOG_FUNCTION 
  *            - \ref SYS_GPB10_ANALOG_FUNCTION 
  *            - \ref SYS_GPB11_ANALOG_FUNCTION 
  *            - \ref SYS_GPB12_ANALOG_FUNCTION 
  *            - \ref SYS_GPB13_ANALOG_FUNCTION 
  *            - \ref SYS_GPB14_ANALOG_FUNCTION 
  *            - \ref SYS_GPB15_ANALOG_FUNCTION 
  * @return None
  */
__STATIC_INLINE void SYS_EnableAnalogPins(uint32_t u32PinFuns)
{
	PB->DINOFF |= ((u32PinFuns&0xffff)<<GPIO_DINOFF_DINOFF_Pos);
	CSCAN->AGPIO |= (u32PinFuns&CSCAN_AGPIO_AGPIO_Msk); 
}

/**
  * @brief Disable GPIO analog function.
  * @param[in] u32PinFuns corresponding GPIO pins
  *            - \ref SYS_GPB0_ANALOG_FUNCTION 
  *            - \ref SYS_GPB1_ANALOG_FUNCTION 
  *            - \ref SYS_GPB2_ANALOG_FUNCTION 
  *            - \ref SYS_GPB3_ANALOG_FUNCTION 
  *            - \ref SYS_GPB4_ANALOG_FUNCTION 
  *            - \ref SYS_GPB5_ANALOG_FUNCTION 
  *            - \ref SYS_GPB6_ANALOG_FUNCTION 
  *            - \ref SYS_GPB7_ANALOG_FUNCTION 
  *            - \ref SYS_GPB8_ANALOG_FUNCTION 
  *            - \ref SYS_GPB9_ANALOG_FUNCTION 
  *            - \ref SYS_GPB10_ANALOG_FUNCTION 
  *            - \ref SYS_GPB11_ANALOG_FUNCTION 
  *            - \ref SYS_GPB12_ANALOG_FUNCTION 
  *            - \ref SYS_GPB13_ANALOG_FUNCTION 
  *            - \ref SYS_GPB14_ANALOG_FUNCTION 
  *            - \ref SYS_GPB15_ANALOG_FUNCTION 
  * @return None
  */
__STATIC_INLINE void SYS_DisableAnalogPins(uint32_t u32PinFuns)
{
	PB->DINOFF &= ~((u32PinFuns&0xffff)<<GPIO_DINOFF_DINOFF_Pos);
	CSCAN->AGPIO &= ~(u32PinFuns&CSCAN_AGPIO_AGPIO_Msk); 
}

void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetResetSrc(void);

void SYS_LockReg(void);
void SYS_UnlockReg(void);
void SYS_Lock(uint8_t u8Lock);
uint8_t SYS_Unlock(void);
uint32_t SYS_IsRegLocked(void);

uint32_t SYS_ReadPDID(void);

/*@}*/ /* end of group I91200_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SYS_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SYS_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
