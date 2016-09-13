/**************************************************************************//**
 * @file     sys.h
 * @version  V3.00
 * $Revision: 20 $
 * $Date: 15/07/07 3:37p $
 * @brief    SYS Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
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
#define HDIV_RST    ((0x0<<24) | SYS_IPRSTC1_HDIV_RST_Pos )     /*!< HDIV reset is one of the SYS_ResetModule parameter */
#define GPIO_RST    ((0x4<<24) | SYS_IPRSTC2_GPIO_RST_Pos )     /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24) | SYS_IPRSTC2_TMR0_RST_Pos )     /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24) | SYS_IPRSTC2_TMR1_RST_Pos )     /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24) | SYS_IPRSTC2_TMR2_RST_Pos )     /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24) | SYS_IPRSTC2_TMR3_RST_Pos )     /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4<<24) | SYS_IPRSTC2_I2C0_RST_Pos )     /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24) | SYS_IPRSTC2_SPI0_RST_Pos )     /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define SPI1_RST    ((0x4<<24) | SYS_IPRSTC2_SPI1_RST_Pos )     /*!< SPI1 reset is one of the SYS_ResetModule parameter */
#define SPI2_RST    ((0x4<<24) | SYS_IPRSTC2_SPI2_RST_Pos )     /*!< SPI2 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24) | SYS_IPRSTC2_UART0_RST_Pos )    /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4<<24) | SYS_IPRSTC2_UART1_RST_Pos )    /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define BPWM0_RST   ((0x4<<24) | SYS_IPRSTC2_BPWM0_RST_Pos )    /*!< BPWM0 reset is one of the SYS_ResetModule parameter */
#define EPWM0_RST   ((0x4<<24) | SYS_IPRSTC2_EPWM0_RST_Pos )    /*!< EPWM0 reset is one of the SYS_ResetModule parameter */
#define EPWM1_RST   ((0x4<<24) | SYS_IPRSTC2_EPWM1_RST_Pos )    /*!< EPWM1 reset is one of the SYS_ResetModule parameter */
#define ACMP_RST    ((0x4<<24) | SYS_IPRSTC2_ACMP_RST_Pos  )    /*!< ACMP reset is one of the SYS_ResetModule parameter */
#define ECAP0_RST   ((0x4<<24) | SYS_IPRSTC2_ECAP0_RST_Pos )    /*!< ECAP0 reset is one of the SYS_ResetModule parameter */
#define ECAP1_RST   ((0x4<<24) | SYS_IPRSTC2_ECAP1_RST_Pos )    /*!< ECAP1 reset is one of the SYS_ResetModule parameter */
#define EADC_RST    ((0x4<<24) | SYS_IPRSTC2_EADC_RST_Pos )     /*!< EADC reset is one of the SYS_ResetModule parameter */
#define OPA_RST     ((0x4<<24) | SYS_IPRSTC2_OPA_RST_Pos )      /*!< OPA reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCR_BOD_RST_EN            (1UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCR_BOD_INTERRUPT_EN      (0UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCR_BOD_VL_4_4V           (3UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown-out Detector Threshold Voltage as 4.4V */
#define SYS_BODCR_BOD_VL_3_7V           (2UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown-out Detector Threshold Voltage as 3.7V */
#define SYS_BODCR_BOD_VL_2_7V           (1UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown-out Detector Threshold Voltage as 2.7V */
#define SYS_BODCR_BOD_VL_2_2V           (0UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown-out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?
Example: If user want to set P0.0 as EPWM0_CH0 and P0.1 as EPWM0_CH1 in initial function,
         user can issue following command to achieve it.

         SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P00_Msk);
         SYS->P0_MFP |= (SYS_MFP_P00_EPWM0_CH0 | SYS_MFP_P01_EPWM0_CH1);
*/

#define SYS_MFP_P00_GPIO            0x00000000UL    /*!< P0_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P00_EPWM0_CH0       0x00000001UL    /*!< P0_MFP pin 0 setting for EPWM0_CH0   */
#define SYS_MFP_P00_ECAP1_IC0       0x00000101UL    /*!< P0_MFP pin 0 setting for ECAP1_IC0   */
#define SYS_MFP_P00_Msk             0x00000101UL    /*!< P0_MFP pin 0 Msk                     */

#define SYS_MFP_P01_GPIO            0x00000000UL    /*!< P0_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P01_EPWM0_CH1       0x00000002UL    /*!< P0_MFP pin 1 setting for EPWM0_CH1   */
#define SYS_MFP_P01_ECAP1_IC1       0x00000202UL    /*!< P0_MFP pin 1 setting for ECAP1_IC1   */
#define SYS_MFP_P01_Msk             0x00000202UL    /*!< P0_MFP pin 1 Msk                     */

#define SYS_MFP_P02_GPIO            0x00000000UL    /*!< P0_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P02_EPWM0_CH2       0x00000004UL    /*!< P0_MFP pin 2 setting for EPWM0_CH2   */
#define SYS_MFP_P02_ECAP1_IC2       0x00000404UL    /*!< P0_MFP pin 2 setting for ECAP1_IC2   */
#define SYS_MFP_P02_Msk             0x00000404UL    /*!< P0_MFP pin 2 Msk                     */

#define SYS_MFP_P03_GPIO            0x00000000UL    /*!< P0_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P03_EPWM0_CH3       0x00000008UL    /*!< P0_MFP pin 3 setting for EPWM0_CH3   */
#define SYS_MFP_P03_STADC           0x00000808UL    /*!< P0_MFP pin 3 setting for STADC       */
#define SYS_MFP_P03_Msk             0x00000808UL    /*!< P0_MFP pin 3 Msk                     */

#define SYS_MFP_P04_GPIO            0x00000000UL    /*!< P0_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P04_EPWM0_CH4       0x00000010UL    /*!< P0_MFP pin 4 setting for EPWM0_CH4   */
#define SYS_MFP_P04_Msk             0x00000010UL    /*!< P0_MFP pin 4 Msk                     */

#define SYS_MFP_P05_GPIO            0x00000000UL    /*!< P0_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P05_EPWM0_CH5       0x00000020UL    /*!< P0_MFP pin 5 setting for EPWM0_CH5   */
#define SYS_MFP_P05_Msk             0x00000020UL    /*!< P0_MFP pin 5 Msk                     */

#define SYS_MFP_P06_GPIO            0x00000000UL    /*!< P0_MFP pin 6 setting for GPIO         */
#define SYS_MFP_P06_EPWM0_BRAKE1    0x00000040UL    /*!< P0_MFP pin 6 setting for EPWM0_BRAKE1 */
#define SYS_MFP_P06_Msk             0x00000040UL    /*!< P0_MFP pin 6 Msk                      */

#define SYS_MFP_P07_GPIO            0x00000000UL    /*!< P0_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P07_STADC           0x00000080UL    /*!< P0_MFP pin 7 setting for STADC       */
#define SYS_MFP_P07_Msk             0x00000080UL    /*!< P0_MFP pin 7 Msk                     */

#define SYS_MFP_P10_GPIO            0x00000000UL    /*!< P1_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P10_EPWM1_CH0       0x00000001UL    /*!< P1_MFP pin 0 setting for EPWM1_CH0   */
#define SYS_MFP_P10_Msk             0x00000001UL    /*!< P1_MFP pin 0 Msk                     */

#define SYS_MFP_P11_GPIO            0x00000000UL    /*!< P1_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P11_EPWM1_CH1       0x00000002UL    /*!< P1_MFP pin 1 setting for EPWM1_CH1   */
#define SYS_MFP_P11_Msk             0x00000002UL    /*!< P1_MFP pin 1 Msk                     */

#define SYS_MFP_P12_GPIO            0x00000000UL    /*!< P1_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P12_EPWM1_CH2       0x00000004UL    /*!< P1_MFP pin 2 setting for EPWM1_CH2   */
#define SYS_MFP_P12_Msk             0x00000004UL    /*!< P1_MFP pin 2 Msk                     */

#define SYS_MFP_P13_GPIO            0x00000000UL    /*!< P1_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P13_EPWM1_CH3       0x00000008UL    /*!< P1_MFP pin 3 setting for EPWM1_CH3   */
#define SYS_MFP_P13_Msk             0x00000008UL    /*!< P1_MFP pin 3 Msk                     */

#define SYS_MFP_P14_GPIO            0x00000000UL    /*!< P1_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P14_EPWM1_CH4       0x00000010UL    /*!< P1_MFP pin 4 setting for EPWM1_CH4   */
#define SYS_MFP_P14_Msk             0x00000010UL    /*!< P1_MFP pin 4 Msk                     */

#define SYS_MFP_P15_GPIO            0x00000000UL    /*!< P1_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P15_EPWM1_CH5       0x00000020UL    /*!< P1_MFP pin 5 setting for EPWM1_CH5   */
#define SYS_MFP_P15_Msk             0x00000020UL    /*!< P1_MFP pin 5 Msk                     */

#define SYS_MFP_P16_GPIO            0x00000000UL    /*!< P1_MFP pin 6 setting for GPIO         */
#define SYS_MFP_P16_EPWM0_BRAKE0    0x00000040UL    /*!< P1_MFP pin 6 setting for EPWM0_BRAKE0 */
#define SYS_MFP_P16_Msk             0x00000040UL    /*!< P1_MFP pin 6 Msk                      */

#define SYS_MFP_P17_GPIO            0x00000000UL    /*!< P1_MFP pin 7 setting for GPIO         */
#define SYS_MFP_P17_EPWM1_BRAKE0    0x00000080UL    /*!< P1_MFP pin 7 setting for EPWM1_BRAKE0 */
#define SYS_MFP_P17_Msk             0x00000080UL    /*!< P1_MFP pin 7 Msk                      */

#define SYS_MFP_P20_GPIO            0x00000000UL    /*!< P2_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P20_SPI2_MOSI       0x00000001UL    /*!< P2_MFP pin 0 setting for SPI2_MOSI   */
#define SYS_MFP_P20_ACMP2_O         0x00000100UL    /*!< P2_MFP pin 0 setting for ACMP2_O     */
#define SYS_MFP_P20_Msk             0x00000101UL    /*!< P2_MFP pin 0 Msk                     */

#define SYS_MFP_P21_GPIO            0x00000000UL    /*!< P2_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P21_ECAP0_IC2       0x00000002UL    /*!< P2_MFP pin 1 setting for ECAP0_IC2   */
#define SYS_MFP_P21_Msk             0x00000002UL    /*!< P2_MFP pin 1 Msk                     */

#define SYS_MFP_P22_GPIO            0x00000000UL    /*!< P2_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P22_ECAP0_IC1       0x00000004UL    /*!< P2_MFP pin 2 setting for ECAP0_IC1   */
#define SYS_MFP_P22_Msk             0x00000004UL    /*!< P2_MFP pin 2 Msk                     */

#define SYS_MFP_P23_GPIO            0x00000000UL    /*!< P2_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P23_ECAP0_IC0       0x00000008UL    /*!< P2_MFP pin 3 setting for ECAP0_IC0   */
#define SYS_MFP_P23_Msk             0x00000008UL    /*!< P2_MFP pin 3 Msk                     */

#define SYS_MFP_P24_GPIO            0x00000000UL    /*!< P2_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P24_Msk             0x00000010UL    /*!< P2_MFP pin 4 Msk                     */

#define SYS_MFP_P25_GPIO            0x00000000UL    /*!< P2_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P25_Msk             0x00000020UL    /*!< P2_MFP pin 5 Msk                     */

#define SYS_MFP_P26_GPIO            0x00000000UL    /*!< P2_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P26_SPI0_SS         0x00004000UL    /*!< P2_MFP pin 6 setting for SPI0_SS     */
#define SYS_MFP_P26_UART1_nCTS      0x00004040UL    /*!< P2_MFP pin 6 setting for UART1_nCTS  */
#define SYS_MFP_P26_Msk             0x00004040UL    /*!< P2_MFP pin 6 Msk                     */

#define SYS_MFP_P27_GPIO            0x00000000UL    /*!< P2_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P27_SPI0_CLK        0x00008000UL    /*!< P2_MFP pin 7 setting for SPI0_CLK    */
#define SYS_MFP_P27_UART1_nRTS      0x00008080UL    /*!< P2_MFP pin 7 setting for UART1_nRTS  */
#define SYS_MFP_P27_Msk             0x00008080UL    /*!< P2_MFP pin 7 Msk                     */

#define SYS_MFP_P30_GPIO            0x00000000UL    /*!< P3_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P30_UART0_RXD       0x00000001UL    /*!< P3_MFP pin 0 setting for UART0_RXD   */
#define SYS_MFP_P30_CLKO            0x00000100UL    /*!< P3_MFP pin 0 setting for CLKO        */
#define SYS_MFP_P30_Msk             0x00000101UL    /*!< P3_MFP pin 0 Msk                     */

#define SYS_MFP_P31_GPIO            0x00000000UL    /*!< P3_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P31_UART0_TXD       0x00000002UL    /*!< P3_MFP pin 1 setting for UART0_TXD   */
#define SYS_MFP_P31_ACMP0_O         0x00000200UL    /*!< P3_MFP pin 1 setting for ACMP0_O     */
#define SYS_MFP_P31_Msk             0x00000202UL    /*!< P3_MFP pin 1 Msk                     */

#define SYS_MFP_P32_GPIO            0x00000000UL    /*!< P3_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P32_INT0            0x00000004UL    /*!< P3_MFP pin 2 setting for INT0        */
#define SYS_MFP_P32_Msk             0x00000004UL    /*!< P3_MFP pin 2 Msk                     */

#define SYS_MFP_P33_GPIO            0x00000000UL    /*!< P3_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P33_INT1            0x00000008UL    /*!< P3_MFP pin 3 setting for INT1        */
#define SYS_MFP_P33_Msk             0x00000008UL    /*!< P3_MFP pin 3 Msk                     */

#define SYS_MFP_P34_GPIO            0x00000000UL    /*!< P3_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P34_TM0             0x00000010UL    /*!< P3_MFP pin 4 setting for TM0         */
#define SYS_MFP_P34_I2C0_SDA        0x00001000UL    /*!< P3_MFP pin 4 setting for I2C0_SDA    */
#define SYS_MFP_P34_Msk             0x00001010UL    /*!< P3_MFP pin 4 setting for Msk         */

#define SYS_MFP_P35_GPIO            0x00000000UL    /*!< P3_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P35_TM1             0x00000020UL    /*!< P3_MFP pin 5 setting for TM1         */
#define SYS_MFP_P35_I2C0_SCL        0x00002000UL    /*!< P3_MFP pin 5 setting for I2C0_SCL    */
#define SYS_MFP_P35_Msk             0x00002020UL    /*!< P3_MFP pin 5 Msk                     */

#define SYS_MFP_P36_GPIO            0x00000000UL    /*!< P3_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P36_Msk             0x00000040UL    /*!< P3_MFP pin 6 Msk                     */

#define SYS_MFP_P37_GPIO            0x00000000UL    /*!< P3_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P37_Msk             0x00000080UL    /*!< P3_MFP pin 7 Msk                     */

#define SYS_MFP_P40_GPIO            0x00000000UL    /*!< P4_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P40_ECAP1_IC0       0x00000001UL    /*!< P4_MFP pin 0 setting for ECAP1_IC0   */
#define SYS_MFP_P40_Msk             0x00000001UL    /*!< P4_MFP pin 0 Msk                     */

#define SYS_MFP_P41_GPIO            0x00000000UL    /*!< P4_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P41_ECAP1_IC1       0x00000002UL    /*!< P4_MFP pin 1 setting for ECAP1_IC1   */
#define SYS_MFP_P41_Msk             0x00000002UL    /*!< P4_MFP pin 1 setting for Msk         */

#define SYS_MFP_P42_GPIO            0x00000000UL    /*!< P4_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P42_ECAP1_IC2       0x00000004UL    /*!< P4_MFP pin 2 setting for ECAP1_IC2   */
#define SYS_MFP_P42_Msk             0x00000004UL    /*!< P4_MFP pin 2 Msk                     */

#define SYS_MFP_P43_GPIO            0x00000000UL    /*!< P4_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P43_Msk             0x00000008UL    /*!< P4_MFP pin 3 Msk                     */

#define SYS_MFP_P44_GPIO            0x00000000UL    /*!< P4_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P44_Msk             0x00000010UL    /*!< P4_MFP pin 4 Msk                     */

#define SYS_MFP_P45_GPIO            0x00000000UL    /*!< P4_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P45_Msk             0x00000020UL    /*!< P4_MFP pin 5 Msk                     */

#define SYS_MFP_P46_GPIO            0x00000000UL    /*!< P4_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P46_TM2             0x00000040UL    /*!< P4_MFP pin 6 setting for TM2         */
#define SYS_MFP_P46_Msk             0x00004040UL    /*!< P4_MFP pin 6 Msk                     */

#define SYS_MFP_P47_GPIO            0x00000000UL    /*!< P4_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P47_TM3             0x00000080UL    /*!< P4_MFP pin 7 setting for TM3         */
#define SYS_MFP_P47_Msk             0x00000080UL    /*!< P4_MFP pin 7 Msk                     */

#define SYS_MFP_P50_GPIO            0x00000000UL    /*!< P5_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P50_SPI0_MOSI       0x00000001UL    /*!< P5_MFP pin 0 setting for SPI0_MOSI   */
#define SYS_MFP_P50_UART0_nRTS      0x00000100UL    /*!< P5_MFP pin 0 setting for UART0_nRTS  */
#define SYS_MFP_P50_I2C0_SCL        0x00000101UL    /*!< P5_MFP pin 0 setting for I2C0_SCL    */
#define SYS_MFP_P50_Msk             0x00000101UL    /*!< P5_MFP pin 0 Msk                     */

#define SYS_MFP_P51_GPIO            0x00000000UL    /*!< P5_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P51_SPI0_MISO       0x00000002UL    /*!< P5_MFP pin 1 setting for SPI0_MISO   */
#define SYS_MFP_P51_UART0_nCTS      0x00000200UL    /*!< P5_MFP pin 1 setting for UART0_nCTS  */
#define SYS_MFP_P51_I2C0_SDA        0x00000202UL    /*!< P5_MFP pin 1 setting for I2C0_SDA    */
#define SYS_MFP_P51_Msk             0x00000202UL    /*!< P5_MFP pin 1 Msk                     */

#define SYS_MFP_P52_GPIO            0x00000000UL    /*!< P5_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P52_SPI2_MISO       0x00000004UL    /*!< P5_MFP pin 2 setting for SPI2_MISO   */
#define SYS_MFP_P52_ACMP1_O         0x00000400UL    /*!< P5_MFP pin 2 setting for ACMP1_O     */
#define SYS_MFP_P52_Msk             0x00000404UL    /*!< P5_MFP pin 2 Msk                     */

#define SYS_MFP_P53_GPIO            0x00000000UL    /*!< P5_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P53_SPI2_CLK        0x00000008UL    /*!< P5_MFP pin 3 setting for SPI2_CLK    */
#define SYS_MFP_P53_Msk             0x00000008UL    /*!< P5_MFP pin 3 Msk                     */

#define SYS_MFP_P54_GPIO            0x00000000UL    /*!< P5_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P54_SPI2_SS         0x00000010UL    /*!< P5_MFP pin 4 setting for SPI2_SS     */
#define SYS_MFP_P54_Msk             0x00000010UL    /*!< P5_MFP pin 4 Msk                     */

#define SYS_MFP_P55_GPIO            0x00000000UL    /*!< P5_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P55_CLKO            0x00000020UL    /*!< P5_MFP pin 5 setting for CLKO        */
#define SYS_MFP_P55_Msk             0x00000020UL    /*!< P5_MFP pin 5 Msk                     */

#define SYS_MFP_P56_GPIO            0x00000000UL    /*!< P5_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P56_BPWM0_CH0       0x00000040UL    /*!< P5_MFP pin 6 setting for BPWM0_CH0   */
#define SYS_MFP_P56_Msk             0x00000040UL    /*!< P5_MFP pin 6 Msk                     */

#define SYS_MFP_P57_GPIO            0x00000000UL    /*!< P5_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P57_BPWM0_CH1       0x00000080UL    /*!< P5_MFP pin 7 setting for BPWM0_CH1   */
#define SYS_MFP_P57_Msk             0x00000080UL    /*!< P5_MFP pin 7 Msk                     */

#define SYS_MFP_P60_GPIO            0x00000000UL    /*!< P6_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P60_EADC0_CH0       0x00000001UL    /*!< P6_MFP pin 0 setting for EADC0_CH0   */
#define SYS_MFP_P60_Msk             0x00000001UL    /*!< P6_MFP pin 0 Msk                     */

#define SYS_MFP_P61_GPIO            0x00000000UL    /*!< P6_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P61_EADC0_CH1       0x00000002UL    /*!< P6_MFP pin 1 setting for EADC0_CH1   */
#define SYS_MFP_P61_Msk             0x00000002UL    /*!< P6_MFP pin 1 Msk                     */

#define SYS_MFP_P62_GPIO            0x00000000UL    /*!< P6_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P62_EADC0_CH2       0x00000004UL    /*!< P6_MFP pin 2 setting for EADC0_CH2   */
#define SYS_MFP_P62_Msk             0x00000004UL    /*!< P6_MFP pin 2 Msk                     */

#define SYS_MFP_P63_GPIO            0x00000000UL    /*!< P6_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P63_EADC0_CH3       0x00000008UL    /*!< P6_MFP pin 3 setting for EADC0_CH3   */
#define SYS_MFP_P63_Msk             0x00000008UL    /*!< P6_MFP pin 3 Msk                     */

#define SYS_MFP_P64_GPIO            0x00000000UL    /*!< P6_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P64_EADC0_CH4       0x00000010UL    /*!< P6_MFP pin 4 setting for EADC0_CH4   */
#define SYS_MFP_P64_ACMP1_N         0x00000010UL    /*!< P6_MFP pin 4 setting for ACMP1_N     */
#define SYS_MFP_P64_Msk             0x00000010UL    /*!< P6_MFP pin 4 Msk                     */

#define SYS_MFP_P65_GPIO            0x00000000UL    /*!< P6_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P65_EADC0_CH5       0x00000020UL    /*!< P6_MFP pin 5 setting for EADC0_CH5   */
#define SYS_MFP_P65_ACMP1_P         0x00000020UL    /*!< P6_MFP pin 5 setting for ACMP1_P     */
#define SYS_MFP_P65_Msk             0x00000020UL    /*!< P6_MFP pin 5 Msk                     */

#define SYS_MFP_P66_GPIO            0x00000000UL    /*!< P6_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P66_EADC0_CH6       0x00000040UL    /*!< P6_MFP pin 6 setting for EADC0_CH6   */
#define SYS_MFP_P66_Msk             0x00000040UL    /*!< P6_MFP pin 6 Msk                     */

#define SYS_MFP_P67_GPIO            0x00000000UL    /*!< P6_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P67_EADC0_CH7       0x00000080UL    /*!< P6_MFP pin 7 setting for EADC0_CH7   */
#define SYS_MFP_P67_Msk             0x00000080UL    /*!< P6_MFP pin 7 Msk                     */

#define SYS_MFP_P70_GPIO            0x00000000UL    /*!< P7_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P70_EADC1_CH0       0x00000001UL    /*!< P7_MFP pin 0 setting for EADC1_CH0   */
#define SYS_MFP_P70_Msk             0x00000001UL    /*!< P7_MFP pin 0 setting for Msk         */

#define SYS_MFP_P71_GPIO            0x00000000UL    /*!< P7_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P71_EADC1_CH1       0x00000002UL    /*!< P7_MFP pin 1 setting for EADC1_CH1   */
#define SYS_MFP_P71_Msk             0x00000002UL    /*!< P7_MFP pin 1 Msk                     */

#define SYS_MFP_P72_GPIO            0x00000000UL    /*!< P7_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P72_EADC1_CH2       0x00000004UL    /*!< P7_MFP pin 2 setting for EADC1_CH2   */
#define SYS_MFP_P72_Msk             0x00000004UL    /*!< P7_MFP pin 2 Msk                     */

#define SYS_MFP_P73_GPIO            0x00000000UL    /*!< P7_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P73_EADC1_CH3       0x00000008UL    /*!< P7_MFP pin 3 setting for EADC1_CH3   */
#define SYS_MFP_P73_Msk             0x00000008UL    /*!< P7_MFP pin 3 Msk                     */

#define SYS_MFP_P74_GPIO            0x00000000UL    /*!< P7_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P74_EADC1_CH4       0x00000010UL    /*!< P7_MFP pin 4 setting for EADC1_CH4   */
#define SYS_MFP_P74_ACMP2_N         0x00000010UL    /*!< P7_MFP pin 4 setting for ACMP2_N     */
#define SYS_MFP_P74_Msk             0x00000010UL    /*!< P7_MFP pin 4 Msk                     */

#define SYS_MFP_P75_GPIO            0x00000000UL    /*!< P7_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P75_EADC1_CH5       0x00000020UL    /*!< P7_MFP pin 5 setting for EADC1_CH5   */
#define SYS_MFP_P75_ACMP2_P         0x00000020UL    /*!< P7_MFP pin 5 setting for ACMP2_P     */
#define SYS_MFP_P75_Msk             0x00000020UL    /*!< P7_MFP pin 5 Msk                     */

#define SYS_MFP_P76_GPIO            0x00000000UL    /*!< P7_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P76_EADC1_CH6       0x00000040UL    /*!< P7_MFP pin 6 setting for EADC1_CH6   */
#define SYS_MFP_P76_Msk             0x00000040UL    /*!< P7_MFP pin 6 Msk                     */

#define SYS_MFP_P77_GPIO            0x00000000UL    /*!< P7_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P77_EADC1_CH7       0x00000080UL    /*!< P7_MFP pin 7 setting for EADC1_CH7   */
#define SYS_MFP_P77_Msk             0x00000080UL    /*!< P7_MFP pin 7 Msk                     */

#define SYS_MFP_P80_GPIO            0x00000000UL    /*!< P8_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P80_OP0_P           0x00000001UL    /*!< P8_MFP pin 0 setting for OP0_P       */
#define SYS_MFP_P80_Msk             0x00000001UL    /*!< P8_MFP pin 0 Msk                     */

#define SYS_MFP_P81_GPIO            0x00000000UL    /*!< P8_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P81_OP0_N           0x00000002UL    /*!< P8_MFP pin 1 setting for OP0_N       */
#define SYS_MFP_P81_Msk             0x00000002UL    /*!< P8_MFP pin 1 Msk                     */

#define SYS_MFP_P82_GPIO            0x00000000UL    /*!< P8_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P82_OP0_O           0x00000004UL    /*!< P8_MFP pin 2 setting for OP0_O       */
#define SYS_MFP_P82_Msk             0x00000004UL    /*!< P8_MFP pin 2 Msk                     */

#define SYS_MFP_P83_GPIO            0x00000000UL    /*!< P8_MFP pin 3 setting for GPIO        */
#define SYS_MFP_P83_ACMP0_N         0x00000008UL    /*!< P8_MFP pin 3 setting for ACMP0_N     */
#define SYS_MFP_P83_Msk             0x00000008UL    /*!< P8_MFP pin 3 Msk                     */

#define SYS_MFP_P84_GPIO            0x00000000UL    /*!< P8_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P84_ACMP0_P         0x00000010UL    /*!< P8_MFP pin 4 setting for ACMP0_P     */
#define SYS_MFP_P84_Msk             0x00000010UL    /*!< P8_MFP pin 4 Msk                     */

#define SYS_MFP_P85_GPIO            0x00000000UL    /*!< P8_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P85_Msk             0x00000020UL    /*!< P8_MFP pin 5 Msk                     */

#define SYS_MFP_P86_GPIO            0x00000000UL    /*!< P8_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P86_Msk             0x00000040UL    /*!< P8_MFP pin 6 Msk                     */

#define SYS_MFP_P87_GPIO            0x00000000UL    /*!< P8_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P87_ACMP0_O         0x00000080UL    /*!< P8_MFP pin 7 setting for ACMP0_O     */
#define SYS_MFP_P87_Msk             0x00000080UL    /*!< P8_MFP pin 7 Msk                     */

#define SYS_MFP_P90_GPIO            0x00000000UL    /*!< P9_MFP pin 0 setting for GPIO        */
#define SYS_MFP_P90_OP1_O           0x00000001UL    /*!< P9_MFP pin 0 setting for OP1_O       */
#define SYS_MFP_P90_Msk             0x00000001UL    /*!< P9_MFP pin 0 Msk                     */

#define SYS_MFP_P91_GPIO            0x00000000UL    /*!< P9_MFP pin 1 setting for GPIO        */
#define SYS_MFP_P91_OP1_N           0x00000002UL    /*!< P9_MFP pin 1 setting for OP1_N       */
#define SYS_MFP_P91_Msk             0x00000002UL    /*!< P9_MFP pin 1 Msk                     */

#define SYS_MFP_P92_GPIO            0x00000000UL    /*!< P9_MFP pin 2 setting for GPIO        */
#define SYS_MFP_P92_OP1_P           0x00000004UL    /*!< P9_MFP pin 2 setting for OP1_P       */
#define SYS_MFP_P92_Msk             0x00000004UL    /*!< P9_MFP pin 2 Msk                     */

#define SYS_MFP_P93_GPIO            0x00000000UL    /*!< P9_MFP pin 3 setting for GPIO         */
#define SYS_MFP_P93_EPWM1_BRAKE1    0x00000008UL    /*!< P9_MFP pin 3 setting for EPWM1_BRAKE1 */
#define SYS_MFP_P93_Msk             0x00000008UL    /*!< P9_MFP pin 3 Msk                      */

#define SYS_MFP_P94_GPIO            0x00000000UL    /*!< P9_MFP pin 4 setting for GPIO        */
#define SYS_MFP_P94_SPI1_CLK        0x00000010UL    /*!< P9_MFP pin 4 setting for SPI1_CLK    */
#define SYS_MFP_P94_Msk             0x00000010UL    /*!< P9_MFP pin 4 Msk                     */

#define SYS_MFP_P95_GPIO            0x00000000UL    /*!< P9_MFP pin 5 setting for GPIO        */
#define SYS_MFP_P95_SPI1_MISO       0x00000020UL    /*!< P9_MFP pin 5 setting for SPI1_MISO   */
#define SYS_MFP_P95_Msk             0x00000020UL    /*!< P9_MFP pin 5 Msk                     */

#define SYS_MFP_P96_GPIO            0x00000000UL    /*!< P9_MFP pin 6 setting for GPIO        */
#define SYS_MFP_P96_SPI1_MOSI       0x00000040UL    /*!< P9_MFP pin 6 setting for SPI1_MOSI   */
#define SYS_MFP_P96_Msk             0x00000040UL    /*!< P9_MFP pin 6 setting for Msk         */

#define SYS_MFP_P97_GPIO            0x00000000UL    /*!< P9_MFP pin 7 setting for GPIO        */
#define SYS_MFP_P97_SPI1_SS         0x00000080UL    /*!< P9_MFP pin 7 setting for SPI1_SS     */
#define SYS_MFP_P97_Msk             0x00000080UL    /*!< P9_MFP pin 7 Msk                     */

#define SYS_MFP_PA0_GPIO            0x00000000UL    /*!< PA_MFP pin 0 setting for GPIO        */
#define SYS_MFP_PA0_UART1_TXD       0x00000001UL    /*!< PA_MFP pin 0 setting for UART1_TXD   */
#define SYS_MFP_PA0_I2C0_SDA        0x00000101UL    /*!< PA_MFP pin 0 setting for I2C0_SDA    */
#define SYS_MFP_PA0_Msk             0x00000101UL    /*!< PA_MFP pin 0 Msk                     */

#define SYS_MFP_PA1_GPIO            0x00000000UL    /*!< PA_MFP pin 1 setting for GPIO        */
#define SYS_MFP_PA1_UART1_RXD       0x00000002UL    /*!< PA_MFP pin 1 setting for UART1_RXD   */
#define SYS_MFP_PA1_I2C0_SCL        0x00000202UL    /*!< PA_MFP pin 1 setting for I2C0_SCL    */
#define SYS_MFP_PA1_Msk             0x00000202UL    /*!< PA_MFP pin 1 Msk                     */


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
  *             - \ref SYS_BODCR_BOD_VL_4_4V
  *             - \ref SYS_BODCR_BOD_VL_3_7V
  *             - \ref SYS_BODCR_BOD_VL_2_7V
  *             - \ref SYS_BODCR_BOD_VL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The register write-protection function should be disabled before using this macro.
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

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
