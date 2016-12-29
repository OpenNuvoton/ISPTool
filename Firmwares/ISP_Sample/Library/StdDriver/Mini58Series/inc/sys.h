/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * $Revision: 15 $
 * $Date: 15/06/04 5:18p $ 
 * @brief    Mini58 series SYS driver header file
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
    
/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup Mini58_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/    
/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_RST  ((0x4<<24) | SYS_IPRST1_ADCRST_Pos    ) /*!< ADC  reset is one of the SYS_ResetModule parameter */
#define ACMP_RST ((0x4<<24) | SYS_IPRST1_ACMPRST_Pos   ) /*!< ACMP reset is one of the SYS_ResetModule parameter */
#define PWM0_RST  ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos    ) /*!< PWM  reset is one of the SYS_ResetModule parameter */
#define UART0_RST ((0x4<<24) | SYS_IPRST1_UART0RST_Pos ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST ((0x4<<24) | SYS_IPRST1_UART1RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos    ) /*!< SPI  reset is one of the SYS_ResetModule parameter */
#define I2C0_RST ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos  ) /*!< I2C0  reset is one of the SYS_ResetModule parameter */
#define I2C1_RST ((0x4<<24) | SYS_IPRST1_I2C1RST_Pos  ) /*!< I2C1  reset is one of the SYS_ResetModule parameter */
#define TMR1_RST ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos   ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR0_RST ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos   ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define GPIO_RST ((0x4<<24) | SYS_IPRST1_GPIORST_Pos   ) /*!< GPIO reset is one of the SYS_ResetModule parameter */
    
    
/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN             (1UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN       (0UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BODVL_4_4V             (3UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 4.4V */ 
#define SYS_BODCTL_BODVL_3_7V             (2UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_2_7V             (1UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_2V             (0UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_MFP_TYPE_Msk(bit)       (1UL << ((bit) +16)) /*!< TYPE mask for Multiple Function Port */
#define SYS_MFP_ALT_Msk(bit)        (1UL << ((bit) + 8)) /*!< ALT mask for Multiple Function Port */
#define SYS_MFP_MFP_Msk(bit)        (1UL << ((bit)    )) /*!< MFP mask for Multiple Function Port */
                                                                                         
#define SYS_MFP_P00_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P00_UART0_nCTS       0x00000100UL     /*!< Clear to Send input pin for UART0.               */
#define SYS_MFP_P00_UART0_TXD        0x00000101UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P00_Msk              0x00000101UL     /*!< P0_MFP pin 0 mask                                */
                                                       
#define SYS_MFP_P01_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P01_SPI0_SS          0x00000002UL     /*!< 1st SPI0 slave select pin.                       */
#define SYS_MFP_P01_UART0_nRTS       0x00000200UL     /*!< Request to Send output pin for UART0.            */
#define SYS_MFP_P01_UART0_RXD        0x00000202UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P01_Msk              0x00000202UL     /*!< P0_MFP pin 1 mask                                */
                                                        
#define SYS_MFP_P04_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P04_PWM0_CH5         0x00001010UL     /*!< PWM0 channel5 output/capture input.              */
#define SYS_MFP_P04_SPI0_SS          0x00001000UL     /*!< 1st SPI0 slave select pin.                       */
#define SYS_MFP_P04_Msk              0x00001010UL     /*!< P0_MFP pin 4 mask                                */
                                                        
#define SYS_MFP_P05_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P05_PWM0_CH4         0x00002020UL     /*!< PWM0 channel4 output/capture input.              */
#define SYS_MFP_P05_SPI0_MOSI        0x00002000UL     /*!< 1st SPI0 MOSI (Master Out, Slave In) pin.        */
#define SYS_MFP_P05_Msk              0x00002020UL     /*!< P0_MFP pin 5 mask                                */
                                                        
#define SYS_MFP_P06_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P06_PWM0_CH1         0x00004040UL     /*!< PWM0 channel1 output/capture input.              */
#define SYS_MFP_P06_SPI0_MISO        0x00004000UL     /*!< 1st SPI0 MISO (Master In, Slave Out) pin.        */
#define SYS_MFP_P06_Msk              0x00004040UL     /*!< P0_MFP pin 6 mask                                */
                                                        
#define SYS_MFP_P07_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P07_PWM0_CH0         0x00008080UL     /*!< PWM0 channel0 output/capture input.              */
#define SYS_MFP_P07_SPI0_CLK         0x00008000UL     /*!< SPI0 serial clock pin.                           */
#define SYS_MFP_P07_Msk              0x00008080UL     /*!< P0_MFP pin 7 mask                                */
                                                        
#define SYS_MFP_P10_ACMP0_P1         0x00000101UL     /*!< Analog comparator0 positive input pin.           */
#define SYS_MFP_P10_ADC_CH1          0x00000001UL     /*!< ADC channel1 analog input.                       */
#define SYS_MFP_P10_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P10_Msk              0x00000101UL     /*!< P1_MFP pin 0 mask                                */
                                                        
#define SYS_MFP_P12_ACMP0_P2         0x00000404UL     /*!< Analog comparator0 positive input pin.           */
#define SYS_MFP_P12_ADC_CH2          0x00000004UL     /*!< ADC channel2 analog input.                       */
#define SYS_MFP_P12_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P12_UART0_RXD        0x00000400UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P12_PWM0_CH0         0x04000000UL     /*!< PWM0 channel0 output/capture input.              */
#define SYS_MFP_P12_Msk              0x04000404UL     /*!< P1_MFP pin 2 mask                                */
                                                        
#define SYS_MFP_P13_ACMP0_P3         0x00000808UL     /*!< Analog comparator0 positive input pin.           */
#define SYS_MFP_P13_ADC_CH3          0x00000008UL     /*!< ADC channel3 analog input.                       */
#define SYS_MFP_P13_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P13_UART0_TXD        0x00000800UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P13_PWM0_CH1         0x08000000UL     /*!< PWM0 channel1 output/capture input.              */
#define SYS_MFP_P13_Msk              0x08000808UL     /*!< P1_MFP pin 3 mask                                */
                                                        
#define SYS_MFP_P14_ACMP0_N          0x00001010UL     /*!< Analog comparator0 negative input pin.           */
#define SYS_MFP_P14_ADC_CH4          0x00000010UL     /*!< ADC channel4 analog input.                       */
#define SYS_MFP_P14_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P14_UART1_RXD        0x00001000UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P14_PWM0_CH4         0x10000000UL     /*!< PWM0 channel4 output/capture input.              */
#define SYS_MFP_P14_Msk              0x10001010UL     /*!< P1_MFP pin 4 mask                                */
                                                        
#define SYS_MFP_P15_ACMP0_P0         0x00002020UL     /*!< Analog comparator0 positive input pin.           */
#define SYS_MFP_P15_ADC_CH5          0x00000020UL     /*!< ADC channel5 analog input.                       */
#define SYS_MFP_P15_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P15_UART1_TXD        0x00002000UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P15_Msk              0x00002020UL     /*!< P1_MFP pin 5 mask                                */
                                                        
#define SYS_MFP_P22_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P22_I2C1_SCL         0x00000404UL     /*!< I2C1 clock pin.                                  */
#define SYS_MFP_P22_PWM0_CH0         0x00000400UL     /*!< PWM0 channel0 output/capture input.              */
#define SYS_MFP_P22_Msk              0x00000404UL     /*!< P2_MFP pin 2 mask                                */
                                                     
#define SYS_MFP_P23_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P23_I2C1_SDA         0x00000808UL     /*!< I2C1 data input/output pin.                      */
#define SYS_MFP_P23_PWM0_CH1         0x00000800UL     /*!< PWM0 channel1 output/capture input.              */
#define SYS_MFP_P23_Msk              0x00000808UL     /*!< P2_MFP pin 3 mask                                */
                                                        
#define SYS_MFP_P24_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P24_PWM0_CH2         0x00001000UL     /*!< PWM0 channel2 output/capture input.              */
#define SYS_MFP_P24_UART1_RXD        0x00000010UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P24_Msk              0x00001010UL     /*!< P2_MFP pin 4 mask                                */
                                                        
#define SYS_MFP_P25_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P25_PWM0_CH3         0x00002000UL     /*!< PWM0 channel3 output/capture input.              */
#define SYS_MFP_P25_UART1_TXD        0x00000020UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P25_Msk              0x00002020UL     /*!< P2_MFP pin 5 mask                                */
                                                        
#define SYS_MFP_P26_ACMP1_O          0x00004040UL     /*!< Analog ccomparator1 output.                      */
#define SYS_MFP_P26_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P26_PWM0_CH4         0x00004000UL     /*!< PWM0 channel4 output/capture input.              */
#define SYS_MFP_P26_Msk              0x00004040UL     /*!< P2_MFP pin 6 mask                                */
                                                        
#define SYS_MFP_P30_ACMP1_N          0x00000100UL     /*!< Analog comparator1 negative input pin.           */
#define SYS_MFP_P30_ADC_CH6          0x00000101UL     /*!< ADC channel6 analog input.                       */
#define SYS_MFP_P30_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P30_Msk              0x00000101UL     /*!< P3_MFP pin 0 mask                                */
                                                        
#define SYS_MFP_P31_ACMP1_P0         0x00000200UL     /*!< Analog comparator1 positive input pin.           */
#define SYS_MFP_P31_ADC_CH7          0x00000202UL     /*!< ADC channel7 analog input.                       */
#define SYS_MFP_P31_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P31_Msk              0x00000202UL     /*!< P3_MFP pin 1 mask                                */
                                                        
#define SYS_MFP_P32_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P32_INT0             0x00000004UL     /*!< External interrupt0 input pin.                   */
#define SYS_MFP_P32_STADC            0x00000404UL     /*!< ADC external trigger input.                      */
#define SYS_MFP_P32_TM0_EXT          0x00000400UL     /*!< Timer0 external capture input.                   */
#define SYS_MFP_P32_ACMP1_P1         0x04000000UL     /*!< Analog comparator1 positive input pin.           */
#define SYS_MFP_P32_Msk              0x04000404UL     /*!< P3_MFP pin 2 mask                                */
                                                        
#define SYS_MFP_P34_ACMP1_P2         0x00001010UL     /*!< Analog comparator1 positive input pin.           */
#define SYS_MFP_P34_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P34_I2C0_SDA         0x00001000UL     /*!< I2C0 data input/output pin.                      */
#define SYS_MFP_P34_TM0_CNT_OUT      0x00000010UL     /*!< Timer0  event counter input/toggle output.       */
#define SYS_MFP_P34_Msk              0x00001010UL     /*!< P3_MFP pin 4 mask                                */
                                                        
#define SYS_MFP_P35_ACMP1_P3         0x00002020UL     /*!< Analog comparator1 positive input pin.           */
#define SYS_MFP_P35_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P35_I2C0_SCL         0x00002000UL     /*!< I2C0 clock pin.                                  */
#define SYS_MFP_P35_TM1_CNT_OUT      0x00000020UL     /*!< Timer1  event counter input/toggle output.       */
#define SYS_MFP_P35_Msk              0x00002020UL     /*!< P3_MFP pin 5 mask                                */
                                                        
#define SYS_MFP_P36_ACMP0_O          0x00004040UL     /*!< Analog ccomparator0 output.                      */
#define SYS_MFP_P36_CLKO             0x00004000UL     /*!< Clock output.                                    */
#define SYS_MFP_P36_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P36_TM1_EXT          0x00000040UL     /*!< Timer1 external capture input.                   */
#define SYS_MFP_P36_Msk              0x00004040UL     /*!< P3_MFP pin 6 mask                                */
                                                        
#define SYS_MFP_P46_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P46_ICE_CLK          0x00000040UL     /*!< Serial wired debugger clock pin.                 */
#define SYS_MFP_P46_UART1_RXD        0x00004000UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P46_Msk              0x00004040UL     /*!< P4_MFP pin 6 mask                                */
                                                        
#define SYS_MFP_P47_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P47_ICE_DAT          0x00000080UL     /*!< Serial wired debugger data pin.                  */
#define SYS_MFP_P47_UART1_TXD        0x00008000UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P47_Msk              0x00008080UL     /*!< P4_MFP pin 7 mask                                */
                                                        
#define SYS_MFP_P50_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P50_I2C1_SDA         0x00000100UL     /*!< I2C1 data input/output pin.                      */
#define SYS_MFP_P50_UART0_TXD        0x00000101UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P50_XT1_IN           0x00000001UL     /*!< External 4~24 MHz (high speed) crystal input pin. */
#define SYS_MFP_P50_Msk              0x00000101UL     /*!< P5_MFP pin 0 mask                                */
                                                        
#define SYS_MFP_P51_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P51_I2C1_SCL         0x00000200UL     /*!< I2C1 clock pin.                                  */
#define SYS_MFP_P51_UART0_RXD        0x00000202UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P51_XT1_OUT          0x00000002UL     /*!< External 4~24 MHz (high speed) crystal output pin. */
#define SYS_MFP_P51_Msk              0x00000202UL     /*!< P5_MFP pin 1 mask                                */
                                                        
#define SYS_MFP_P52_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P52_INT1             0x00000004UL     /*!< External interrupt1 input pin.                   */
#define SYS_MFP_P52_Msk              0x00000404UL     /*!< P5_MFP pin 2 mask                                */
                                                        
#define SYS_MFP_P53_ADC_CH0          0x00000008UL     /*!< ADC channel0 analog input.                       */
#define SYS_MFP_P53_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P53_Msk              0x00000808UL     /*!< P5_MFP pin 3 mask                                */
                                                        
#define SYS_MFP_P54_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P54_Msk              0x00001010UL     /*!< P5_MFP pin 4 mask                                */
                                                        
#define SYS_MFP_P55_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P55_Msk              0x00002020UL     /*!< P5_MFP pin 5 mask                                */


/*@}*/ /* end of group Mini58_SYS_EXPORTED_CONSTANTS */

/** @addtogroup Mini58_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
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
#define SYS_DISABLE_BOD()               (SYS->BODCTL = (SYS->BODCTL &~(SYS_BODCTL_BODVL_Msk|SYS_BODCTL_BODEN_Msk))|SYS_BODCTL_BODVL_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL = (SYS->BODCTL | SYS_BODCTL_BODEN_Msk))

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
  *             - \ref SYS_BODCTL_BODVL_4_4V
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
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
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
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);    

/*@}*/ /* end of group Mini58_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_SYS_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SYS_H__
