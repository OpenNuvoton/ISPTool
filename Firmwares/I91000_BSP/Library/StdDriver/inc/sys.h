/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/10/19 10:00a $
 * @brief    ISD9000 Series SYS Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup ISD9000_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  RSTSTS constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_RSTSTS_PORWK         (SYS_RSTSTS_PORWK_Msk)      /*!<Wakeup from DPD From POR  \hideinitializer */
#define  SYS_RSTSTS_TIMWK         (SYS_RSTSTS_TIMWK_Msk)      /*!<Wakeup from DPD From TIMER  \hideinitializer */
#define  SYS_RSTSTS_PINWK         (SYS_RSTSTS_PINWK_Msk)      /*!<Wakeup from DPD From PIN  \hideinitializer */
#define  SYS_RSTSTS_PMURSTF       (SYS_RSTSTS_PMURSTF_Msk)    /*!<Reset Source From PMU  \hideinitializer */
#define  SYS_RSTSTS_M0F           (SYS_RSTSTS_M0F_Msk)        /*!<M0 Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_BODF          (SYS_RSTSTS_BODF_Msk)       /*!<BOD Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_LVRF          (SYS_RSTSTS_LVRF_Msk)       /*!<LVR Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_WDTRF         (SYS_RSTSTS_WDTRF_Msk)      /*!<Reset Source From WDG  \hideinitializer */
#define  SYS_RSTSTS_PINRF         (SYS_RSTSTS_PINRF_Msk)      /*!<Reset Pin Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_PORF          (SYS_RSTSTS_PORF_Msk)       /*!<POR Reset Flag  \hideinitializer */
 	
/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CHIP_RST    ((0x0<<24) | SYS_IPRST0_CHIPRST_Pos  ) /*!<Reset CHIP  \hideinitializer */
#define CPU_RST     ((0x0<<24) | SYS_IPRST0_CPURST_Pos   ) /*!<Reset CPU  \hideinitializer  */
#define GPIO_RST    ((0x4<<24) | SYS_IPRST1_GPIORST_Pos  ) /*!<Reset GPIO  \hideinitializer  */
#define TMR0_RST    ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos  ) /*!<Reset TMR0  \hideinitializer  */
#define TMR1_RST    ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos  ) /*!<Reset TMR1  \hideinitializer  */
#define TMR2_RST    ((0x4<<24) | SYS_IPRST1_TMR2RST_Pos  ) /*!<Reset TMR2  \hideinitializer  */
#define TMRF_RST    ((0x4<<24) | SYS_IPRST1_TMRFRST_Pos  ) /*!<Reset TMRF  \hideinitializer  */
#define PDMA_RST    ((0x4<<24) | SYS_IPRST1_PDMARST_Pos  ) /*!<Reset PDMA  \hideinitializer  */
#define SPI0_RST    ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos  ) /*!<Reset SPI0  \hideinitializer  */
#define SPIM_RST    ((0x4<<24) | SYS_IPRST1_SPIMRST_Pos  ) /*!<Reset SPIM  \hideinitializer  */
#define PWM0_RST    ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos  ) /*!<Reset PWM0  \hideinitializer  */
#define PWM1_RST    ((0x4<<24) | SYS_IPRST1_PWM1RST_Pos  ) /*!<Reset PWM1  \hideinitializer  */
#define ADC_RST     ((0x4<<24) | SYS_IPRST1_ADCRST_Pos   ) /*!<Reset ADC  \hideinitializer  */
#define DPWM_RST    ((0x4<<24) | SYS_IPRST1_DPWMRST_Pos  ) /*!<Reset DPWM  \hideinitializer  */

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO_INTP constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_GPIO_INTP_GPA0_TO_GPA3     (0x1UL<<SYS_GPIO_INTP_GPA0_3_Pos)
#define SYS_GPIO_INTP_GPA4_TO_GPA7     (0x1UL<<SYS_GPIO_INTP_GPA4_7_Pos)
#define SYS_GPIO_INTP_GPA8_TO_GPA11    (0x1UL<<SYS_GPIO_INTP_GPA8_11_Pos)
#define SYS_GPIO_INTP_GPA12_TO_GPA15   (0x1UL<<SYS_GPIO_INTP_GPA12_15_Pos)
#define SYS_GPIO_INTP_GPB0_TO_GPB3     (0x1UL<<SYS_GPIO_INTP_GPB0_3_Pos)
#define SYS_GPIO_INTP_GPB4_TO_GPB5     (0x1UL<<SYS_GPIO_INTP_GPB4_5_Pos)

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as PWM00 in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_PWM00  ;

*/
//GPA_MFP_PA0MFP
#define SYS_GPA_MFP_PA0MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_PWM00       (0x1UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for PWM00      \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_SPI_SS0     (0x2UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for SPI_SS0    \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_CXA0        (0x3UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for CXA0       \hideinitializer */

//GPA_MFP_PA1MFP
#define SYS_GPA_MFP_PA1MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_PWM01       (0x1UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for PWM01      \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_SPI_CLK     (0x2UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for SPI_CLK    \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_CXA1        (0x3UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for CXA1       \hideinitializer */

//GPA_MFP_PA2MFP
#define SYS_GPA_MFP_PA2MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_PWM02       (0x1UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for PWM02      \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_SPI_MISO    (0x2UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for SPI_MISO   \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_CXA2        (0x3UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for CXA2       \hideinitializer */

//GPA_MFP_PA3MFP
#define SYS_GPA_MFP_PA3MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_PWM03       (0x1UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for PWM03      \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_SPI_MOSI    (0x2UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for SPI_MOSI   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_CXA3        (0x3UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for CXA3       \hideinitializer */

//GPA_MFP_PA4MFP
#define SYS_GPA_MFP_PA4MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_PWM10       (0x1UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for PWM10      \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_SPI_SS1     (0x2UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for SPI_SS1    \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_CXA4        (0x3UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for CXA4       \hideinitializer */

//GPA_MFP_PA5MFP
#define SYS_GPA_MFP_PA5MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_PWM11       (0x1UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for PWM11      \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_TM0         (0x2UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for TM0        \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_CXA5        (0x3UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for CXA5       \hideinitializer */

//GPA_MFP_PA6MFP
#define SYS_GPA_MFP_PA6MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_PWM12       (0x1UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for PWM12      \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_TM1         (0x2UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for TM1        \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_CXA6        (0x3UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for CXA6       \hideinitializer */

//GPA_MFP_PA7MFP
#define SYS_GPA_MFP_PA7MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_PWM13       (0x1UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for PWM13      \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_CPR1        (0x2UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for CPR1       \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_CXA7        (0x3UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for CXA7       \hideinitializer */

//GPA_MFP_PA8MFP
#define SYS_GPA_MFP_PA8MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_UARTRTS     (0x2UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for UARTRTS    \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_X32I        (0x3UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for X32I       \hideinitializer */

//GPA_MFP_PA9MFP
#define SYS_GPA_MFP_PA9MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_UARTCTS     (0x2UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for UARTCTS    \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_X32O        (0x3UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for X32O       \hideinitializer */

//GPA_MFP_PA10MFP
#define SYS_GPA_MFP_PA10MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_UARTTX     (0x2UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for UARTTx    \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_CXA10      (0x3UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for CXA10     \hideinitializer */

//GPA_MFP_PA11MFP
#define SYS_GPA_MFP_PA11MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_UARTRX     (0x2UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for UARTRx    \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_CXA11      (0x3UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for CXA11     \hideinitializer */

//GPA_MFP_PA12MFP
#define SYS_GPA_MFP_PA12MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_MICBIAS    (0x1UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for MICBIAS   \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_CXA12      (0x3UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for CXA12     \hideinitializer */

//GPA_MFP_PA13MFP
#define SYS_GPA_MFP_PA13MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_MICP       (0x1UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for MICP      \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_CXA13      (0x3UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for CXA13     \hideinitializer */

//GPA_MFP_PA14MFP
#define SYS_GPA_MFP_PA14MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_MICN       (0x1UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for MICN      \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_AIN5       (0x2UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for AIN5      \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_CXA14      (0x3UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for CXA14     \hideinitializer */

//GPA_MFP_PA15MFP
#define SYS_GPA_MFP_PA15MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_PGCVMID    (0x1UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for PGCVMID   \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_CXA15      (0x3UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for CXA15     \hideinitializer */

//GPB_MFP_PB0MFP
#define SYS_GPB_MFP_PB0MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_SPIM_CLK    (0x1UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for SPIM_CLK   \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_PWM00       (0x2UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for PWM00      \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_SPIM_MOSI   (0x3UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for SPIM_MOSI  \hideinitializer */

//GPB_MFP_PB1MFP
#define SYS_GPB_MFP_PB1MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_SPIM_MOSI   (0x1UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for SPIM_CLK   \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_PWM01       (0x2UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for PWM01      \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_SPIM_CLK    (0x3UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for CXB1       \hideinitializer */

//GPB_MFP_PB2MFP
#define SYS_GPB_MFP_PB2MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_SPIM_SIO3   (0x1UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for SPIM_MISO  \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_PWM02       (0x2UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for PWM02      \hideinitializer */
#define SYS_GPB_MFP_PB2MFP_SPIM_SS     (0x3UL<<SYS_GPB_MFP_PB2MFP_Pos)           /*!< GPB_MFP PB2 setting for CXB2       \hideinitializer */

//GPB_MFP_PB3MFP
#define SYS_GPB_MFP_PB3MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_SPIM_SIO2   (0x1UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for SPIM_MOSI  \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_PWM03       (0x2UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for PWM03      \hideinitializer */
#define SYS_GPB_MFP_PB3MFP_SPIM_MISO   (0x3UL<<SYS_GPB_MFP_PB3MFP_Pos)           /*!< GPB_MFP PB3 setting for CXB3       \hideinitializer */

//GPB_MFP_PB4MFP
#define SYS_GPB_MFP_PB4MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_SPIM_MISO   (0x1UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for SPI_SIO2   \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_CPR0        (0x2UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for CPR0       \hideinitializer */
#define SYS_GPB_MFP_PB4MFP_CXB4        (0x3UL<<SYS_GPB_MFP_PB4MFP_Pos)           /*!< GPB_MFP PB4 setting for CXB4       \hideinitializer */

//GPB_MFP_PB5MFP
#define SYS_GPB_MFP_PB5MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_SPIM_SS     (0x1UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for SPI_SIO3   \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_IR          (0x2UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for IR         \hideinitializer */
#define SYS_GPB_MFP_PB5MFP_CXB5        (0x3UL<<SYS_GPB_MFP_PB5MFP_Pos)           /*!< GPB_MFP PB5 setting for CXB5       \hideinitializer */

/*@}*/ /* end of group ISD9000_SYS_EXPORTED_CONSTANTS */

/** @addtogroup ISD9000_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

#define SYS_IS_POR_RST()                   (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)    /*!< This macro get previous reset source is from Power-on Reset   \hideinitializer */
#define SYS_IS_PIN_RST()                   (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)   /*!< This macro get previous reset source is from Pin reset   \hideinitializer */
#define SYS_IS_WDT_RST()                   (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)   /*!< This macro get previous reset source is from Watch dog reset   \hideinitializer */
#define SYS_IS_LV_RST()                    (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)    /*!< This macro get previous reset source is from Lower voltage reset   \hideinitializer */
#define SYS_IS_PMU_RST()                   (SYS->RSTSTS & SYS_RSTSTS_PMURSTF_Msk) /*!< This macro get previous reset source is from Power manager unit reset   \hideinitializer */

#define SYS_CLEAR_RST_SOURCE(u32RSTSTS)    (SYS->RSTSTS = u32RSTSTS)              /*!< This macro clears reset source   \hideinitializer */

#define SYS_DISABLE_POR()                  SYS_DisablePOR()                             /*!< This macro disable Power-on Reset function \hideinitializer */
#define SYS_ENABLE_POR()                   SYS_EnablePOR()                              /*!< This macro enable Power-on Reset function \hideinitializer */

#define SYS_SET_GPIO_INPUT_TYPE_SMITT(u32PinMask)       (SYS->GPIO_INTP |= u32PinMask)  /*!< This macro set GPIOA high speed transition to 50MHz   \hideinitializer */
#define SYS_SET_GPIO_INPUT_TYPE_CMOS(u32PinMask)        (SYS->GPIO_INTP &= ~u32PinMask) /*!< This macro set GPIOA normal speed transition less than 25MHz.   \hideinitializer */

#define SYS_SET_GPIO_OUTPUT_HIGH_SLEW_RATE(u32PinMask)  (SYS->GPIO_INTP |= (u32PinMask<<1))         /*!< This macro set GPIOB high speed transition to 50MHz   \hideinitializer */
#define SYS_SET_GPIO_OUTPUT_LOW_SLEW_RATE(u32PinMask)   (SYS->GPIO_INTP &= ~(u32PinMask<<1))        /*!< This macro set GPIOB normal speed transition less than 25MHz.   \hideinitializer */

#define SYS_ENABLE_GPIOA_PULL_UP(u32PinMask)  (SYS->GPA_PULL |= (u32PinMask&0xFFFF))       /*!< This macro enable GPIOA input mode pull-up   \hideinitializer */
#define SYS_DISABLE_GPIOA_PULL_UP(u32PinMask) (SYS->GPA_PULL &= ~(u32PinMask&0xFFFF))      /*!< This macro disable GPIOA input mode pull-up   \hideinitializer */
#define SYS_ENABLE_GPIOB_PULL_UP(u32PinMask)  (SYS->GPB_PULL |= (u32PinMask&0x3F))         /*!< This macro enable GPIOB input mode pull-up   \hideinitializer */
#define SYS_DISABLE_GPIOB_PULL_UP(u32PinMask) (SYS->GPB_PULL &= ~(u32PinMask&0x3F))        /*!< This macro disable GPIOB input mode pull-up   \hideinitializer */

#define SYS_ENABLE_GPIOA_DIGITAL_INPUT_BUF(u32PinMask)  (SYS->GPA_IEN &= ~(u32PinMask&0xFFFF))     /*!< This macro enable GPIOA digital input buffer   \hideinitializer */
#define SYS_DISABLE_GPIOA_DIGITAL_INPUT_BUF(u32PinMask) (SYS->GPA_IEN |= (u32PinMask&0xFFFF))      /*!< This macro disable GPIOA digital input buffer   \hideinitializer */
#define SYS_ENABLE_GPIOB_DIGITAL_INPUT_BUF(u32PinMask)  (SYS->GPB_IEN &= ~(u32PinMask&0x3F))       /*!< This macro enable GPIOB digital input buffer   \hideinitializer */
#define SYS_DISABLE_GPIOB_DIGITAL_INPUT_BUF(u32PinMask) (SYS->GPB_IEN |= (u32PinMask&0x3F))        /*!< This macro disable GPIOB digital input buffer   \hideinitializer */

#define SYS_READ_ROMMAP0()                 (*((uint32_t*)&SYS->IMGMAP0))                /*!< This macro read ROMMAP0 image data   \hideinitializer */
#define SYS_READ_ROMMAP1()                 (*((uint32_t*)&SYS->IMGMAP1))                /*!< This macro read ROMMAP1 image data   \hideinitializer */
#define SYS_READ_ROMMAP2()                 (*((uint32_t*)&SYS->IMGMAP2))                /*!< This macro read ROMMAP2 image data   \hideinitializer */
#define SYS_READ_ROMMAP3()                 (*((uint32_t*)&SYS->IMGMAP3))                /*!< This macro read ROMMAP3 image data   \hideinitializer */

#define SYS_ENABLE_ICE_PIN()               SYS_EnableICEPin()					        /*!< This macro set ICE_TCK and ICE_TDA to be ICE CLCOK/ ICE DIO,for debug purpose.   \hideinitializer */
#define SYS_DISABLE_ICE_PIN()              SYS_DisableICEPin()                          /*!< This macro set ICE_TCK and ICE_TDA to be GPB4 and GPB5,for general IO purpose.   \hideinitializer */

#define SYS_SET_PA_DC_OFFSET(u8AdjValue)   SYS_SetPADCOffset(u8AdjValue)			    /*!< This macro turn the DC offset voltage between SPK+/SPK-.   \hideinitializer */

void     SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetResetSrc(void);

uint32_t SYS_IsRegLocked(void);
void     SYS_LockReg(void);
void 	 SYS_Lock(uint8_t u8Lock);
void     SYS_UnlockReg(void);
uint8_t  SYS_Unlock(void);

uint32_t SYS_ReadPDID(void);
uint32_t SYS_ReadDeviceID(void);

void     SYS_ResetChip(void);
void     SYS_ResetCPU(void);
void     SYS_ResetModule(uint32_t u32ModuleIndex);

void 	 SYS_DisablePOR(void);
void 	 SYS_EnablePOR(void);

void	 SYS_SetPADCOffset(uint8_t u8AdjValue);

void	 SYS_DisableICEPin(void);
void 	 SYS_EnableICEPin(void);

/*@}*/ /* end of group ISD9000_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_SYS_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif 

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
