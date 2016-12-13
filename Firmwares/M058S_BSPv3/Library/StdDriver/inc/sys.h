/**************************************************************************//**
 * @file     SYS.h
 * @version  V3.00
 * $Revision: 8 $
 * $Date: 15/04/08 5:58p $
 * @brief    SYS driver header file
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
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
#define GPIO_RST    ((0x4<<24) | SYS_IPRSTC2_GPIO_RST_Pos   ) /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24) | SYS_IPRSTC2_TMR0_RST_Pos   ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24) | SYS_IPRSTC2_TMR1_RST_Pos   ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24) | SYS_IPRSTC2_TMR2_RST_Pos   ) /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24) | SYS_IPRSTC2_TMR3_RST_Pos   ) /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4<<24) | SYS_IPRSTC2_I2C0_RST_Pos   ) /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x4<<24) | SYS_IPRSTC2_I2C1_RST_Pos   ) /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24) | SYS_IPRSTC2_SPI0_RST_Pos   ) /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24) | SYS_IPRSTC2_UART0_RST_Pos  ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define PWM03_RST   ((0x4<<24) | SYS_IPRSTC2_PWM03_RST_Pos  ) /*!< PWM03 reset is one of the SYS_ResetModule parameter */
#define ADC_RST     ((0x4<<24) | SYS_IPRSTC2_ADC_RST_Pos    ) /*!< ADC reset is one of the SYS_ResetModule parameter */


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
Example: If user want to set P0.2 as TXD and P0.3 as RXD in initial function,
         user can issue following command to achieve it.

         SYS->P0_MFP &= ~(SYS_MFP_P02_Msk | SYS_MFP_P03_Msk);
         SYS->P0_MFP |= (SYS_MFP_P02_TXD | SYS_MFP_P03_RXD);
*/

#define SYS_MFP_P00_GPIO    0x00000000UL /*!< P0_MFP pin 0 setting for GPIO */
#define SYS_MFP_P00_Msk     0x01000101UL /*!< P0_MFP pin 0 mask             */

#define SYS_MFP_P01_GPIO    0x00000000UL /*!< P0_MFP pin 1 setting for GPIO */
#define SYS_MFP_P01_Msk     0x02000202UL /*!< P0_MFP pin 1 mask             */

#define SYS_MFP_P02_GPIO    0x00000000UL /*!< P0_MFP pin 2 setting for GPIO */
#define SYS_MFP_P02_CTS     0x00000400UL /*!< P0_MFP pin 2 setting for CTS  */
#define SYS_MFP_P02_TXD     0x00000404UL /*!< P0_MFP pin 2 setting for TXD  */
#define SYS_MFP_P02_Msk     0x00000404UL /*!< P0_MFP pin 2 mask             */

#define SYS_MFP_P03_GPIO    0x00000000UL /*!< P0_MFP pin 3 setting for GPIO */
#define SYS_MFP_P03_RTS     0x00000800UL /*!< P0_MFP pin 3 setting for RTS  */
#define SYS_MFP_P03_RXD     0x00000808UL /*!< P0_MFP pin 3 setting for RXD  */
#define SYS_MFP_P03_Msk     0x00000808UL /*!< P0_MFP pin 3 mask             */

#define SYS_MFP_P04_GPIO    0x00000000UL /*!< P0_MFP pin 4 setting for GPIO  */
#define SYS_MFP_P04_SPISS   0x00001000UL /*!< P0_MFP pin 4 setting for SPISS */
#define SYS_MFP_P04_Msk     0x00001010UL /*!< P0_MFP pin 4 mask              */

#define SYS_MFP_P05_GPIO    0x00000000UL /*!< P0_MFP pin 5 setting for GPIO */
#define SYS_MFP_P05_MOSI    0x00002000UL /*!< P0_MFP pin 5 setting for MOSI */
#define SYS_MFP_P05_Msk     0x00002020UL /*!< P0_MFP pin 5 mask             */

#define SYS_MFP_P06_GPIO    0x00000000UL /*!< P0_MFP pin 6 setting for GPIO */
#define SYS_MFP_P06_MISO    0x00004000UL /*!< P0_MFP pin 6 setting for MISO */
#define SYS_MFP_P06_Msk     0x00004040UL /*!< P0_MFP pin 6 mask             */

#define SYS_MFP_P07_GPIO    0x00000000UL /*!< P0_MFP pin 7 setting for GPIO   */
#define SYS_MFP_P07_SPICLK  0x00008000UL /*!< P0_MFP pin 7 setting for SPICLK */
#define SYS_MFP_P07_Msk     0x00008080UL /*!< P0_MFP pin 7 mask               */

#define SYS_MFP_P10_GPIO    0x00000000UL /*!< P1_MFP pin 0 setting for GPIO */
#define SYS_MFP_P10_AIN0    0x00000001UL /*!< P1_MFP pin 0 setting for AIN0 */
#define SYS_MFP_P10_T2      0x00000100UL /*!< P1_MFP pin 0 setting for T2   */
#define SYS_MFP_P10_Msk     0x00000101UL /*!< P1_MFP pin 0 mask             */

#define SYS_MFP_P11_GPIO    0x00000000UL /*!< P1_MFP pin 1 setting for GPIO */
#define SYS_MFP_P11_AIN1    0x00000002UL /*!< P1_MFP pin 1 setting for AIN1 */
#define SYS_MFP_P11_T3      0x00000200UL /*!< P1_MFP pin 1 setting for T3   */
#define SYS_MFP_P11_Msk     0x00000202UL /*!< P1_MFP pin 1 mask             */

#define SYS_MFP_P12_GPIO    0x00000000UL /*!< P1_MFP pin 2 setting for GPIO */
#define SYS_MFP_P12_AIN2    0x00000004UL /*!< P1_MFP pin 2 setting for AIN2 */
#define SYS_MFP_P12_Msk     0x00000404UL /*!< P1_MFP pin 2 mask             */

#define SYS_MFP_P13_GPIO    0x00000000UL /*!< P1_MFP pin 3 setting for GPIO */
#define SYS_MFP_P13_AIN3    0x00000008UL /*!< P1_MFP pin 3 setting for AIN3 */
#define SYS_MFP_P13_Msk     0x00000808UL /*!< P1_MFP pin 3 mask             */

#define SYS_MFP_P14_GPIO    0x00000000UL /*!< P1_MFP pin 4 setting for GPIO  */
#define SYS_MFP_P14_AIN4    0x00000010UL /*!< P1_MFP pin 4 setting for AIN4  */
#define SYS_MFP_P14_SPISS   0x00001000UL /*!< P1_MFP pin 4 setting for SPISS */
#define SYS_MFP_P14_Msk     0x00001010UL /*!< P1_MFP pin 4 mask              */

#define SYS_MFP_P15_GPIO    0x00000000UL /*!< P1_MFP pin 5 setting for GPIO */
#define SYS_MFP_P15_AIN5    0x00000020UL /*!< P1_MFP pin 5 setting for AIN5 */
#define SYS_MFP_P15_MOSI    0x00002000UL /*!< P1_MFP pin 5 setting for MOSI */
#define SYS_MFP_P15_Msk     0x00002020UL /*!< P1_MFP pin 5 mask             */

#define SYS_MFP_P16_GPIO    0x00000000UL /*!< P1_MFP pin 6 setting for GPIO */
#define SYS_MFP_P16_AIN6    0x00000040UL /*!< P1_MFP pin 6 setting for AIN6 */
#define SYS_MFP_P16_MISO    0x00004000UL /*!< P1_MFP pin 6 setting for MISO */
#define SYS_MFP_P16_Msk     0x00004040UL /*!< P1_MFP pin 6 mask             */

#define SYS_MFP_P17_GPIO    0x00000000UL /*!< P1_MFP pin 7 setting for GPIO   */
#define SYS_MFP_P17_AIN7    0x00000080UL /*!< P1_MFP pin 7 setting for AIN7   */
#define SYS_MFP_P17_SPICLK  0x00008000UL /*!< P1_MFP pin 7 setting for SPICLK */
#define SYS_MFP_P17_Msk     0x00008080UL /*!< P1_MFP pin 7 mask               */

#define SYS_MFP_P20_GPIO    0x00000000UL /*!< P2_MFP pin 0 setting for GPIO */
#define SYS_MFP_P20_PWM0    0x00000100UL /*!< P2_MFP pin 0 setting for PWM0 */
#define SYS_MFP_P20_Msk     0x00000101UL /*!< P2_MFP pin 0 mask             */

#define SYS_MFP_P21_GPIO    0x00000000UL /*!< P2_MFP pin 1 setting for GPIO */
#define SYS_MFP_P21_PWM1    0x00000200UL /*!< P2_MFP pin 1 setting for PWM1 */
#define SYS_MFP_P21_Msk     0x00000202UL /*!< P2_MFP pin 1 mask             */

#define SYS_MFP_P22_GPIO    0x00000000UL /*!< P2_MFP pin 2 setting for GPIO */
#define SYS_MFP_P22_PWM2    0x00000400UL /*!< P2_MFP pin 2 setting for PWM2 */
#define SYS_MFP_P22_Msk     0x00000404UL /*!< P2_MFP pin 2 mask             */

#define SYS_MFP_P23_GPIO    0x00000000UL /*!< P2_MFP pin 3 setting for GPIO */
#define SYS_MFP_P23_PWM3    0x00000800UL /*!< P2_MFP pin 3 setting for PWM3 */
#define SYS_MFP_P23_Msk     0x00000808UL /*!< P2_MFP pin 3 mask             */

#define SYS_MFP_P24_GPIO    0x00000000UL /*!< P2_MFP pin 4 setting for GPIO */
#define SYS_MFP_P24_Msk     0x00001010UL /*!< P2_MFP pin 4 mask             */

#define SYS_MFP_P25_GPIO    0x00000000UL /*!< P2_MFP pin 5 setting for GPIO */
#define SYS_MFP_P25_Msk     0x00002020UL /*!< P2_MFP pin 5 mask             */

#define SYS_MFP_P26_GPIO    0x00000000UL /*!< P2_MFP pin 6 setting for GPIO */
#define SYS_MFP_P26_Msk     0x00004040UL /*!< P2_MFP pin 6 mask             */

#define SYS_MFP_P27_GPIO    0x00000000UL /*!< P2_MFP pin 7 setting for GPIO */
#define SYS_MFP_P27_Msk     0x00008080UL /*!< P2_MFP pin 7 mask             */

#define SYS_MFP_P30_GPIO    0x00000000UL /*!< P3_MFP pin 0 setting for GPIO */
#define SYS_MFP_P30_RXD     0x00000001UL /*!< P3_MFP pin 0 setting for RXD  */
#define SYS_MFP_P30_Msk     0x00000101UL /*!< P3_MFP pin 0 mask             */

#define SYS_MFP_P31_GPIO    0x00000000UL /*!< P3_MFP pin 1 setting for GPIO */
#define SYS_MFP_P31_TXD     0x00000002UL /*!< P3_MFP pin 1 setting for TXD  */
#define SYS_MFP_P31_Msk     0x00000202UL /*!< P3_MFP pin 1 mask             */

#define SYS_MFP_P32_GPIO    0x00000000UL /*!< P3_MFP pin 2 setting for GPIO  */
#define SYS_MFP_P32_nINT0   0x00000004UL /*!< P3_MFP pin 2 setting for nINT0 */
#define SYS_MFP_P32_T0EX    0x00000400UL /*!< P3_MFP pin 2 setting for T0EX  */
#define SYS_MFP_P32_Msk     0x00000404UL /*!< P3_MFP pin 2 mask              */

#define SYS_MFP_P33_GPIO    0x00000000UL /*!< P3_MFP pin 3 setting for GPIO  */
#define SYS_MFP_P33_nINT1   0x00000008UL /*!< P3_MFP pin 3 setting for nINT1 */
#define SYS_MFP_P33_T1EX    0x00000808UL /*!< P3_MFP pin 3 setting for T1EX  */
#define SYS_MFP_P33_Msk     0x00000808UL /*!< P3_MFP pin 3 mask              */

#define SYS_MFP_P34_GPIO    0x00000000UL /*!< P3_MFP pin 4 setting for GPIO */
#define SYS_MFP_P34_T0      0x00000010UL /*!< P3_MFP pin 4 setting for T0   */
#define SYS_MFP_P34_SDA0    0x00001000UL /*!< P3_MFP pin 4 setting for SDA0 */
#define SYS_MFP_P34_Msk     0x00001010UL /*!< P3_MFP pin 4 mask             */

#define SYS_MFP_P35_GPIO    0x00000000UL /*!< P3_MFP pin 5 setting for GPIO */
#define SYS_MFP_P35_T1      0x00000020UL /*!< P3_MFP pin 5 setting for T1   */
#define SYS_MFP_P35_SCL0    0x00002000UL /*!< P3_MFP pin 5 setting for SCL0 */
#define SYS_MFP_P35_CKO     0x00002020UL /*!< P3_MFP pin 5 setting for CKO  */
#define SYS_MFP_P35_Msk     0x00002020UL /*!< P3_MFP pin 5 mask             */

#define SYS_MFP_P36_GPIO    0x00000000UL /*!< P3_MFP pin 6 setting for GPIO */
#define SYS_MFP_P36_CKO     0x00004000UL /*!< P3_MFP pin 6 setting for CKO  */
#define SYS_MFP_P36_Msk     0x00004040UL /*!< P3_MFP pin 6 mask             */

#define SYS_MFP_P37_GPIO    0x00000000UL /*!< P3_MFP pin 7 setting for GPIO */
#define SYS_MFP_P37_Msk     0x00008080UL /*!< P3_MFP pin 7 mask             */

#define SYS_MFP_P40_GPIO    0x00000000UL /*!< P4_MFP pin 0 setting for GPIO */
#define SYS_MFP_P40_PWM0    0x00000001UL /*!< P4_MFP pin 0 setting for PWM0 */
#define SYS_MFP_P40_T2EX    0x00000100UL /*!< P4_MFP pin 0 setting for T2EX */
#define SYS_MFP_P40_Msk     0x00000101UL /*!< P4_MFP pin 0 mask             */

#define SYS_MFP_P41_GPIO    0x00000000UL /*!< P4_MFP pin 1 setting for GPIO */
#define SYS_MFP_P41_PWM1    0x00000002UL /*!< P4_MFP pin 1 setting for PWM1 */
#define SYS_MFP_P41_T3EX    0x00000200UL /*!< P4_MFP pin 1 setting for T3EX */
#define SYS_MFP_P41_Msk     0x00000202UL /*!< P4_MFP pin 1 mask             */

#define SYS_MFP_P42_GPIO    0x00000000UL /*!< P4_MFP pin 2 setting for GPIO */
#define SYS_MFP_P42_PWM2    0x00000004UL /*!< P4_MFP pin 2 setting for PWM2 */
#define SYS_MFP_P42_Msk     0x00000404UL /*!< P4_MFP pin 2 mask             */

#define SYS_MFP_P43_GPIO    0x00000000UL /*!< P4_MFP pin 3 setting for GPIO */
#define SYS_MFP_P43_PWM3    0x00000008UL /*!< P4_MFP pin 3 setting for PWM3 */
#define SYS_MFP_P43_Msk     0x00000808UL /*!< P4_MFP pin 3 mask             */

#define SYS_MFP_P44_GPIO    0x00000000UL /*!< P4_MFP pin 4 setting for GPIO */
#define SYS_MFP_P44_SCL1    0x00001000UL /*!< P4_MFP pin 4 setting for SCL1 */
#define SYS_MFP_P44_Msk     0x00001010UL /*!< P4_MFP pin 4 mask             */

#define SYS_MFP_P45_GPIO    0x00000000UL /*!< P4_MFP pin 5 setting for GPIO */
#define SYS_MFP_P45_SDA1    0x00002000UL /*!< P4_MFP pin 5 setting for SDA1 */
#define SYS_MFP_P45_Msk     0x00002020UL /*!< P4_MFP pin 5 mask             */

#define SYS_MFP_P46_GPIO    0x00000000UL /*!< P4_MFP pin 6 setting for GPIO    */
#define SYS_MFP_P46_ICE_CLK 0x00000040UL /*!< P4_MFP pin 6 setting for ICE_CLK */
#define SYS_MFP_P46_Msk     0x00004040UL /*!< P4_MFP pin 6 mask                */

#define SYS_MFP_P47_GPIO    0x00000000UL /*!< P4_MFP pin 7 setting for GPIO    */
#define SYS_MFP_P47_ICE_DAT 0x00000080UL /*!< P4_MFP pin 7 setting for ICE_DAT */
#define SYS_MFP_P47_Msk     0x00008080UL /*!< P4_MFP pin 7 mask                */

#define SYS_MFP_P50_GPIO    0x00000000UL /*!< P5_MFP pin 0 setting for GPIO */
#define SYS_MFP_P50_T0EX    0x00000001UL /*!< P5_MFP pin 0 setting for T0EX */
#define SYS_MFP_P50_Msk     0x00000101UL /*!< P5_MFP pin 0 mask             */

#define SYS_MFP_P51_GPIO    0x00000000UL /*!< P5_MFP pin 1 setting for GPIO */
#define SYS_MFP_P51_T1EX    0x00000002UL /*!< P5_MFP pin 1 setting for T1EX */
#define SYS_MFP_P51_Msk     0x00000202UL /*!< P5_MFP pin 1 mask             */

#define SYS_MFP_P52_GPIO    0x00000000UL /*!< P5_MFP pin 2 setting for GPIO */
#define SYS_MFP_P52_SDA0    0x00000004UL /*!< P5_MFP pin 2 setting for SDA0 */
#define SYS_MFP_P52_Msk     0x00000404UL /*!< P5_MFP pin 2 mask             */

#define SYS_MFP_P53_GPIO    0x00000000UL /*!< P5_MFP pin 3 setting for GPIO */
#define SYS_MFP_P53_SCL0    0x00000008UL /*!< P5_MFP pin 3 setting for SCL0 */
#define SYS_MFP_P53_Msk     0x00000808UL /*!< P5_MFP pin 3 mask             */

#define SYS_MFP_P70_GPIO    0x00000000UL /*!< P7_MFP pin 0 setting for GPIO  */
#define SYS_MFP_P70_XTAL2   0x00000001UL /*!< P7_MFP pin 0 setting for XTAL2 */
#define SYS_MFP_P70_Msk     0x00000001UL /*!< P7_MFP pin 0 mask              */

#define SYS_MFP_P71_GPIO    0x00000000UL /*!< P7_MFP pin 1 setting for GPIO  */
#define SYS_MFP_P71_XTAL1   0x00000002UL /*!< P7_MFP pin 1 setting for XTAL1 */
#define SYS_MFP_P71_Msk     0x00000002UL /*!< P7_MFP pin 1 mask              */



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
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSRC & SYS_RSTSRC_RSTS_MCU_Msk)

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
  *             - \ref SYS_RSTSRC_RSTS_MCU_Msk
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
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGWRPROT = 0;
}

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
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
