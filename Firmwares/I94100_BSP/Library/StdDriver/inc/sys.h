/**************************************************************************//**
 * @file     SYS.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/06/14 9:35a $
 * @brief    I94100 Series Global Control and Clock Control Driver Header File
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

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup I94100_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_RST    ((0x0<<24) | SYS_IPRST0_PDMARST_Pos )   /*!< Reset PDMA */
#define CRC_RST     ((0x0<<24) | SYS_IPRST0_CRCRST_Pos )    /*!< Reset CRC */
#define GPIO_RST    ((0x4<<24) | SYS_IPRST1_GPIORST_Pos )   /*!< Reset GPIO */
#define TMR0_RST    ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos )   /*!< Reset TMR0 */
#define TMR1_RST    ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos )   /*!< Reset TMR1 */
#define TMR2_RST    ((0x4<<24) | SYS_IPRST1_TMR2RST_Pos )   /*!< Reset TMR2 */
#define TMR3_RST    ((0x4<<24) | SYS_IPRST1_TMR3RST_Pos )   /*!< Reset TMR3 */
#define I2C0_RST    ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos )   /*!< Reset I2C0 */
#define I2C1_RST    ((0x4<<24) | SYS_IPRST1_I2C1RST_Pos )   /*!< Reset I2C1 */
#define SPI0_RST    ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos )   /*!< Reset SPI0 */
#define SPI1_RST    ((0x4<<24) | SYS_IPRST1_SPI1RST_Pos )   /*!< Reset SPI1 */
#define SPI2_RST    ((0x4<<24) | SYS_IPRST1_SPI2RST_Pos )   /*!< Reset SPI2 */
#define UART0_RST   ((0x4<<24) | SYS_IPRST1_UART0RST_Pos )  /*!< Reset UART0 */
#define USBD_RST   	((0x4<<24) | SYS_IPRST1_USBDRST_Pos )  	/*!< Reset USBD */
#define EADC_RST    ((0x4<<24) | SYS_IPRST1_EADCRST_Pos )   /*!< Reset EADC */
#define I2S0_RST    ((0x4<<24) | SYS_IPRST1_I2S0RST_Pos )   /*!< Reset I2S0 */
#define DMIC_RST	((0x4<<24) | SYS_IPRST1_DMICRST_Pos )	/*!< Reset DMIC */
#define PWM0_RST    ((0x8<<24) | SYS_IPRST2_PWM0RST_Pos )   /*!< Reset PWM0 */
#define DPWM_RST	((0x8<<24) | SYS_IPRST2_DPWMRST_Pos )	/*!< Reset DPWM */

/*---------------------------------------------------------------------------------------------------------*/
/*  BODCTL constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_BODCTL_BODVL_1_6V        (0x0UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 1.6V \hideinitializer */
#define  SYS_BODCTL_BODVL_1_8V        (0x1UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 1.8V \hideinitializer */
#define  SYS_BODCTL_BODVL_2_0V        (0x2UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 2.0V \hideinitializer */
#define  SYS_BODCTL_BODVL_2_4V        (0x4UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 2.4V \hideinitializer */
#define  SYS_BODCTL_BODVL_2_6V        (0x5UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 2.6V \hideinitializer */
#define  SYS_BODCTL_BODVL_2_8V        (0x6UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 2.8V \hideinitializer */
#define  SYS_BODCTL_BODVL_3_0V        (0x7UL<<SYS_BODCTL_BODVL_Pos)        /*!<Threshold voltage of BOD is selected 3.0V \hideinitializer */

#define  SYS_BODCTL_LVRDGSEL_0HCLK    (0x0UL)        /*!<LVR without de-glitch function \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_4HCLK    (0x1UL)        /*!<LVR de-glitch time is selected 4 HCLK \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_8HCLK    (0x2UL)        /*!<LVR de-glitch time is selected 8 HCLK \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_16HCLK   (0x3UL)        /*!<LVR de-glitch time is selected 16 HCLK \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_32HCLK   (0x4UL)        /*!<LVR de-glitch time is selected 32 HCLK \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_64HCLK   (0x5UL)        /*!<LVR de-glitch time is selected 64 HCLK \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_128HCLK  (0x6UL)        /*!<LVR de-glitch time is selected 128 HCLK \hideinitializer */
#define  SYS_BODCTL_LVRDGSEL_256HCLK  (0x7UL)        /*!<LVR de-glitch time is selected 256 HCLK \hideinitializer */

#define  SYS_BODCTL_BODDGSEL_LIRC     (0x0UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is sampled by RC10K clock \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_4HCLK    (0x1UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 4 HCLK \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_8HCLK    (0x2UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 8 HCLK \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_16HCLK   (0x3UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 16 HCLK \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_32HCLK   (0x4UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 32 HCLK \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_64HCLK   (0x5UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 64 HCLK \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_128HCLK  (0x6UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 128 HCLK \hideinitializer */
#define  SYS_BODCTL_BODDGSEL_256HCLK  (0x7UL<<SYS_BODCTL_BODDGSEL_Pos)        /*!<Brown-out de-glitch time is selected 256 HCLK \hideinitializer */

#define  SYS_BODCTL_BODRSTEN          (0x1UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!<Enable reset function of BOD. \hideinitializer */
#define  SYS_BODCTL_BODINTEN          (0x0UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!<Enable interrupt function of BOD. \hideinitializer */
#define  SYS_BODCTL_BODLPM            (0x1UL<<SYS_BODCTL_BODLPM_Pos)       /*!<BOD work in low power mode. \hideinitializer */
#define  SYS_BODCTL_LVREN             (0x1UL<<SYS_BODCTL_LVREN_Pos)        /*!<Enable LVR function. \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTCTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_IRCTCTL_FREQSEL_48M   (0x1UL<<SYS_IRCTCTL_FREQSEL_Pos)    /*!<Enable HIRC auto trim function and trim HIRC to 48 MHz. \hideinitializer */
#define  SYS_IRCTCTL_FREQSEL_49M   (0x3UL<<SYS_IRCTCTL_FREQSEL_Pos)    /*!<Enable HIRC auto trim function and trim HIRC to 49.152 MHz. \hideinitializer */

#define  SYS_IRCTCTL_LOOPSEL_4     (0x0UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 4 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_LOOPSEL_8     (0x1UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 8 clocks of reference clock. \hideinitializer */
#define  SYS_IRCTCTL_LOOPSEL_16    (0x2UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 16 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_LOOPSEL_32    (0x3UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 32 clocks of reference clock. \hideinitializer */

#define  SYS_IRCTCTL_RETRYCNT_64   (0x0UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 4 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_RETRYCNT_128  (0x1UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 8 clocks of reference clock. \hideinitializer */
#define  SYS_IRCTCTL_RETRYCNT_256  (0x2UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 16 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_RETRYCNT_512  (0x3UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 32 clocks of reference clock. \hideinitializer */

#define  SYS_IRCTCTL_REFCLK_LXT    (0x0UL<<SYS_IRCTCTL_REFCKSEL_Pos)   /*!<HIRC trim reference clock is from LXT (32.768 kHz). \hideinitializer */
#define  SYS_IRCTCTL_REFCLK_USBSOF (0x1UL<<SYS_IRCTCTL_REFCKSEL_Pos)   /*!<HIRC trim reference clock is from USB SOF (Start-Of-Frame) packet.  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTIEN constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_IRCTIEN_TRIMFAIL_INT_MASK  (SYS_IRCTIEN_TFAILIEN_Msk)    /*!<Enable trim fail interrupt function. \hideinitializer */
#define  SYS_IRCTIEN_CLKERROR_INT_MASK  (SYS_IRCTIEN_CLKEIEN_Msk)     /*!<Enable clock error interrupt function. \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTISTS constant definitions.                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_IRCTISTS_TRIMFAIL_INT_FLAG (SYS_IRCTISTS_TFAILIF_Msk)    /*!< Trim Fail Interrupt Flag.  \hideinitializer */
#define  SYS_IRCTISTS_CLKERROR_INT_FLAG (SYS_IRCTISTS_CLKERRIF_Msk)   /*!< Clock Error Interrupt Flag.  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as SC0_CD in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFPL  = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0_MFP_Msk) ) | SYS_GPA_MFPL_PA0_MFP_SC0_CD  ;

*/

/********************* Bit definition of GPA_MFPL register **********************/
//GPA_MFPL_PA0MFP
#define SYS_GPA_MFPL_PA0MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA0MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA0MFP_SPI0_SS1            (0x01UL<<SYS_GPA_MFPL_PA0MFP_Pos) /* 2nd SPI0 slave select pin. */ 
#define SYS_GPA_MFPL_PA0MFP_EADC0_CH0           (0x02UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*  */

//GPA_MFPL_PA1MFP
#define SYS_GPA_MFPL_PA1MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA1MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA1MFP_SPI0_MOSI1          (0x01UL<<SYS_GPA_MFPL_PA1MFP_Pos) /* 2nd SPI0 MOSI (Master Out, Slave In) pin. */ 
#define SYS_GPA_MFPL_PA1MFP_EADC0_CH1           (0x02UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*  */ 

//GPA_MFPL_PA2MFP
#define SYS_GPA_MFPL_PA2MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA2MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA2MFP_SPI0_MISO1          (0x01UL<<SYS_GPA_MFPL_PA2MFP_Pos) /* 2nd SPI0 MISO (Master In, Slave Out) pin. */ 
#define SYS_GPA_MFPL_PA2MFP_EADC0_CH2           (0x02UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*  */ 

//GPA_MFPL_PA3MFP
#define SYS_GPA_MFPL_PA3MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA3MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA3MFP_SPI0_MOSI0          (0x01UL<<SYS_GPA_MFPL_PA3MFP_Pos) /* 1st SPI0 MOSI (Master Out, Slave In) pin. */ 
#define SYS_GPA_MFPL_PA3MFP_EADC0_CH3           (0x02UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*  */ 

//GPA_MFPL_PA4MFP
#define SYS_GPA_MFPL_PA4MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA4MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA4MFP_SPI0_MISO0          (0x01UL<<SYS_GPA_MFPL_PA4MFP_Pos) /* 1st SPI0 MISO (Master In, Slave Out) pin. */ 
#define SYS_GPA_MFPL_PA4MFP_EADC0_CH4           (0x02UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*  */ 

//GPA_MFPL_PA5MFP
#define SYS_GPA_MFPL_PA5MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA5MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA5MFP_SPI0_CLK            (0x01UL<<SYS_GPA_MFPL_PA5MFP_Pos) /* SPI0 serial clock pin. */ 
#define SYS_GPA_MFPL_PA5MFP_EADC0_CH5           (0x02UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*  */ 

//GPA_MFPL_PA6MFP
#define SYS_GPA_MFPL_PA6MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA6MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA6MFP_SPI0_SS0            (0x01UL<<SYS_GPA_MFPL_PA6MFP_Pos) /* 1st SPI0 slave select pin. */
#define SYS_GPA_MFPL_PA6MFP_EADC0_CH6           (0x02UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*  */ 

//GPA_MFPL_PA7MFP
#define SYS_GPA_MFPL_PA7MFP_GPIO                (0x00UL<<SYS_GPA_MFPL_PA7MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPL_PA7MFP_UART0_TXD           (0x01UL<<SYS_GPA_MFPL_PA7MFP_Pos) /* Data transmitter output pin for UART0. */ 
#define SYS_GPA_MFPL_PA7MFP_EADC0_CH7           (0x02UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*  */ 
#define SYS_GPA_MFPL_PA7MFP_SPI2I2S_DI          (0x04UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*  */ 

/********************* Bit definition of GPA_MFPH register **********************/
//GPA_MFPH_PA8MFP
#define SYS_GPA_MFPH_PA8MFP_GPIO                (0x00UL<<SYS_GPA_MFPH_PA8MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA8MFP_UART0_RXD           (0x01UL<<SYS_GPA_MFPH_PA8MFP_Pos) /* Data receiver input pin for UART0. */ 
#define SYS_GPA_MFPH_PA8MFP_EADC0_CH8           (0x02UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*  */ 
#define SYS_GPA_MFPH_PA8MFP_SPI2I2S_DO          (0x04UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*  */ 

//GPA_MFPH_PA9MFP
#define SYS_GPA_MFPH_PA9MFP_GPIO                (0x00UL<<SYS_GPA_MFPH_PA9MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA9MFP_I2C0_SCL            (0x01UL<<SYS_GPA_MFPH_PA9MFP_Pos) /* I2C0 clock pin. */ 
#define SYS_GPA_MFPH_PA9MFP_EADC0_CH9           (0x02UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*  */ 
#define SYS_GPA_MFPH_PA9MFP_SPI2I2S_LRCK        (0x04UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*  */ 

//GPA_MFPH_PA10MFP
#define SYS_GPA_MFPH_PA10MFP_GPIO               (0x00UL<<SYS_GPA_MFPH_PA10MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA10MFP_I2C0_SDA           (0x01UL<<SYS_GPA_MFPH_PA10MFP_Pos) /* I2C0 data input/output pin. */ 
#define SYS_GPA_MFPH_PA10MFP_EADC0_ST           (0x02UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*  */ 
#define SYS_GPA_MFPH_PA10MFP_SPI2I2S_BCLK       (0x04UL<<SYS_GPA_MFPH_PA10MFP_Pos) /*  */ 

//GPA_MFPH_PA11MFP
#define SYS_GPA_MFPH_PA11MFP_GPIO               (0x00UL<<SYS_GPA_MFPH_PA11MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA11MFP_I2C0_SMBSUS        (0x01UL<<SYS_GPA_MFPH_PA11MFP_Pos) /*  */ 
#define SYS_GPA_MFPH_PA11MFP_T0                 (0x02UL<<SYS_GPA_MFPH_PA11MFP_Pos) /* SPI* serial clock pin. */ 

//GPA_MFPH_PA12MFP
#define SYS_GPA_MFPH_PA12MFP_GPIO               (0x00UL<<SYS_GPA_MFPH_PA12MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA12MFP_I2C0_SMBAL         (0x01UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*  */ 
#define SYS_GPA_MFPH_PA12MFP_T0_EXT             (0x02UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*  */
#define SYS_GPA_MFPH_PA12MFP_SPI2I2S_MCLK       (0x04UL<<SYS_GPA_MFPH_PA12MFP_Pos) /*  */

//GPA_MFPH_PA13MFP
#define SYS_GPA_MFPH_PA13MFP_GPIO               (0x00UL<<SYS_GPA_MFPH_PA13MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA13MFP_CLKO               (0x01UL<<SYS_GPA_MFPH_PA13MFP_Pos) /* Output selected clock. */ 
#define SYS_GPA_MFPH_PA13MFP_INT0               (0x02UL<<SYS_GPA_MFPH_PA13MFP_Pos) /* External interrupt0 input pin. */ 

//GPA_MFPH_PA14MFP
#define SYS_GPA_MFPH_PA14MFP_GPIO               (0x00UL<<SYS_GPA_MFPH_PA14MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA14MFP_SPI0_SS0           (0x01UL<<SYS_GPA_MFPH_PA14MFP_Pos) /* 1st SPI0 slave select pin. */ 
#define SYS_GPA_MFPH_PA14MFP_T1                 (0x02UL<<SYS_GPA_MFPH_PA14MFP_Pos) /*  */ 

//GPA_MFPH_PA15MFP
#define SYS_GPA_MFPH_PA15MFP_GPIO               (0x00UL<<SYS_GPA_MFPH_PA15MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPA_MFPH_PA15MFP_INT0               (0x01UL<<SYS_GPA_MFPH_PA15MFP_Pos) /* External interrupt0 input pin. */ 
#define SYS_GPA_MFPH_PA15MFP_T1_EXT             (0x02UL<<SYS_GPA_MFPH_PA15MFP_Pos) /* Timer1 external capture input. */ 

/********************* Bit definition of GPB_MFPL register **********************/
//GPB_MFPL_PB0MFP
#define SYS_GPB_MFPL_PB0MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB0MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB0MFP_PWM0_SYNC_IN        (0x01UL<<SYS_GPB_MFPL_PB0MFP_Pos) /* PWM0 counter synchronous trigger input pin. */ 
#define SYS_GPB_MFPL_PB0MFP_I2C0_SCL            (0x02UL<<SYS_GPB_MFPL_PB0MFP_Pos) /* I2C0 clock pin. */ 

//GPB_MFPL_PB1MFP
#define SYS_GPB_MFPL_PB1MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB1MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB1MFP_PWM0_SYNC_OUT       (0x01UL<<SYS_GPB_MFPL_PB1MFP_Pos) /* PWM0 counter synchronous trigger output pin. */ 
#define SYS_GPB_MFPL_PB1MFP_I2C0_SDA            (0x02UL<<SYS_GPB_MFPL_PB1MFP_Pos) /* I2C0 data input/output pin. */ 

//GPB_MFPL_PB2MFP
#define SYS_GPB_MFPL_PB2MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB2MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB2MFP_PWM0_CH0            (0x01UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*  */ 
#define SYS_GPB_MFPL_PB2MFP_T2                  (0x02UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*  */ 

//GPB_MFPL_PB3MFP
#define SYS_GPB_MFPL_PB3MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB3MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB3MFP_PWM0_CH1            (0x01UL<<SYS_GPB_MFPL_PB3MFP_Pos) /* PWM0 channel1 output/capture input. */ 
#define SYS_GPB_MFPL_PB3MFP_T2_EXT              (0x02UL<<SYS_GPB_MFPL_PB3MFP_Pos) /* Timer2 external capture input. */
#define SYS_GPB_MFPL_PB3MFP_DMIC_DAT1           (0x03UL<<SYS_GPB_MFPL_PB3MFP_Pos) /* Digital microphone channel 1 data input pin. */

//GPB_MFPL_PB4MFP
#define SYS_GPB_MFPL_PB4MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB4MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB4MFP_UART1_nCTS          (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /* Clear to Send input pin for UART1. */ 
#define SYS_GPB_MFPL_PB4MFP_PWM0_CH0            (0x02UL<<SYS_GPB_MFPL_PB4MFP_Pos) /* PWM0 channel0 output/capture input. */ 
#define SYS_GPB_MFPL_PB4MFP_DMIC_CLK1           (0x03UL<<SYS_GPB_MFPL_PB4MFP_Pos) /* Digital microphone channel 1 clock output pin. */

//GPB_MFPL_PB5MFP
#define SYS_GPB_MFPL_PB5MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB5MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB5MFP_XT1_OUT             (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /* External 4~24 MHz (high speed) crystal output pin. */ 
#define SYS_GPB_MFPL_PB5MFP_PWM0_CH1            (0x02UL<<SYS_GPB_MFPL_PB5MFP_Pos) /* PWM0 channel1 output/capture input. */ 
#define SYS_GPB_MFPL_PB5MFP_DMIC_DAT0           (0x05UL<<SYS_GPB_MFPL_PB5MFP_Pos) /* Digital microphone channel 0 data input pin. */

//GPB_MFPL_PB6MFP
#define SYS_GPB_MFPL_PB6MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB6MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB6MFP_XT1_IN              (0x01UL<<SYS_GPB_MFPL_PB6MFP_Pos) /* External 4~24 MHz (high speed) crystal input pin. */  
#define SYS_GPB_MFPL_PB6MFP_PWM0_CH2            (0x02UL<<SYS_GPB_MFPL_PB6MFP_Pos) /* PWM0 channel2 output/capture input. */ 
#define SYS_GPB_MFPL_PB6MFP_DMIC_CLK0           (0x05UL<<SYS_GPB_MFPL_PB6MFP_Pos) /* Digital microphone channel 0 clock output pin. */

//GPB_MFPL_PB7MFP
#define SYS_GPB_MFPL_PB7MFP_GPIO                (0x00UL<<SYS_GPB_MFPL_PB7MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPL_PB7MFP_UART1_nRTS          (0x01UL<<SYS_GPB_MFPL_PB7MFP_Pos) /* Request to Send output pin for UART1. */ 
#define SYS_GPB_MFPL_PB7MFP_PWM0_CH3            (0x02UL<<SYS_GPB_MFPL_PB7MFP_Pos) /* PWM0 channel3 output/capture input. */ 

/********************* Bit definition of GPB_MFPH register **********************/
//GPB_MFPH_PB8MFP
#define SYS_GPB_MFPH_PB8MFP_GPIO                (0x00UL<<SYS_GPB_MFPH_PB8MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPH_PB8MFP_UART0_TXD           (0x01UL<<SYS_GPB_MFPH_PB8MFP_Pos) /* Data transmitter output pin for UART0. */ 
#define SYS_GPB_MFPH_PB8MFP_PWM0_CH4            (0x02UL<<SYS_GPB_MFPH_PB8MFP_Pos) /* PWM0 channel4 output/capture input. */ 

//GPB_MFPH_PB9MFP
#define SYS_GPB_MFPH_PB9MFP_GPIO                (0x00UL<<SYS_GPB_MFPH_PB9MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPH_PB9MFP_UART0_RXD           (0x01UL<<SYS_GPB_MFPH_PB9MFP_Pos) /* Data receiver input pin for UART0. */ 
#define SYS_GPB_MFPH_PB9MFP_PWM0_CH5            (0x02UL<<SYS_GPB_MFPH_PB9MFP_Pos) /* PWM0 channel5 output/capture input. */
 
#define SYS_GPB_MFPH_PB13MFP_GPIO               (0x00UL<<SYS_GPB_MFPH_PB13MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPH_PB13MFP_USBD_DN            (0x01UL<<SYS_GPB_MFPH_PB13MFP_Pos) /* USBD D+ pin */ 
#define SYS_GPB_MFPH_PB13MFP_I2S0_DI            (0x02UL<<SYS_GPB_MFPH_PB13MFP_Pos) /* Data receiver input pin for I2S0 */ 

#define SYS_GPB_MFPH_PB14MFP_GPIO               (0x00UL<<SYS_GPB_MFPH_PB14MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPH_PB14MFP_USBD_DP            (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos) /* USBD D- pin */ 
#define SYS_GPB_MFPH_PB14MFP_I2S0_DO            (0x02UL<<SYS_GPB_MFPH_PB14MFP_Pos) /* Data sender output pin for I2S0  */ 

#define SYS_GPB_MFPH_PB15MFP_GPIO               (0x00UL<<SYS_GPB_MFPH_PB15MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPB_MFPH_PB15MFP_USBD_VBUS          (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos) /* USBD VBUS pin */ 
#define SYS_GPB_MFPH_PB15MFP_I2S0_MCLK          (0x02UL<<SYS_GPB_MFPH_PB15MFP_Pos) /* MCLK pin for I2S0  */ 

/********************* Bit definition of GPC_MFPL register **********************/
//GPC_MFPL_PC0MFP
#define SYS_GPC_MFPL_PC0MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC0MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC0MFP_I2C1_SCL            (0x01UL<<SYS_GPC_MFPL_PC0MFP_Pos) /* I2C1 clock pin. */ 
#define SYS_GPC_MFPL_PC0MFP_X32_OUT             (0x02UL<<SYS_GPC_MFPL_PC0MFP_Pos) /* External 32.768 kHz (low speed) crystal output pin. */ 

//GPC_MFPL_PC1MFP
#define SYS_GPC_MFPL_PC1MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC1MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC1MFP_I2C1_SDA            (0x01UL<<SYS_GPC_MFPL_PC1MFP_Pos) /* I2C1 data input/output pin. */ 
#define SYS_GPC_MFPL_PC1MFP_X32_IN              (0x02UL<<SYS_GPC_MFPL_PC1MFP_Pos) /* External 32.768 kHz (low speed) crystal input pin. */ 

//GPC_MFPL_PC2MFP
#define SYS_GPC_MFPL_PC2MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC2MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC2MFP_I2C1_SMBSUS         (0x01UL<<SYS_GPC_MFPL_PC2MFP_Pos) /* CAN1 bus transmitter output. */ 
#define SYS_GPC_MFPL_PC2MFP_T3                  (0x02UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*  */ 

//GPC_MFPL_PC3MFP
#define SYS_GPC_MFPL_PC3MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC3MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC3MFP_I2C1_SMBAL          (0x01UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*  */ 
#define SYS_GPC_MFPL_PC3MFP_T3_EXT              (0x02UL<<SYS_GPC_MFPL_PC3MFP_Pos) /* Timer3 external capture input. */ 

//GPC_MFPL_PC4MFP
#define SYS_GPC_MFPL_PC4MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC4MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC4MFP_PWM0_CH2            (0x01UL<<SYS_GPC_MFPL_PC4MFP_Pos) /* PWM0 channel2 output/capture input. */ 
#define SYS_GPC_MFPL_PC4MFP_CLKO                (0x02UL<<SYS_GPC_MFPL_PC4MFP_Pos) /* Output selected clock. */ 

//GPC_MFPL_PC5MFP
#define SYS_GPC_MFPL_PC5MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC5MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC5MFP_INT1                (0x01UL<<SYS_GPC_MFPL_PC5MFP_Pos) /* External interrupt1 input pin. */ 
#define SYS_GPC_MFPL_PC5MFP_SPI2_MOSI           (0x02UL<<SYS_GPC_MFPL_PC5MFP_Pos) /* 1st SPI2 MOSI (Master Out, Slave In) pin. */  

//GPC_MFPL_PC6MFP
#define SYS_GPC_MFPL_PC6MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC6MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC6MFP_INT2                (0x01UL<<SYS_GPC_MFPL_PC6MFP_Pos) /* External interrupt2 input pin. */ 
#define SYS_GPC_MFPL_PC6MFP_SPI2_MISO           (0x02UL<<SYS_GPC_MFPL_PC6MFP_Pos) /* 1st SPI2 MISO (Master In, Slave Out) pin. */ 

//GPC_MFPL_PC7MFP
#define SYS_GPC_MFPL_PC7MFP_GPIO                (0x00UL<<SYS_GPC_MFPL_PC7MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPL_PC7MFP_SPI0_SS0            (0x01UL<<SYS_GPC_MFPL_PC7MFP_Pos) /* 1st SPI0 slave select pin. */ 
#define SYS_GPC_MFPL_PC7MFP_SPI2_CLK            (0x02UL<<SYS_GPC_MFPL_PC7MFP_Pos) /* SPI2 serial clock pin. */ 

/********************* Bit definition of GPC_MFPH register **********************/
//GPC_MFPH_PC8MFP
#define SYS_GPC_MFPH_PC8MFP_GPIO                (0x00UL<<SYS_GPC_MFPH_PC8MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC8MFP_SPI0_MOSI1          (0x01UL<<SYS_GPC_MFPH_PC8MFP_Pos) /* 2nd SPI0 MOSI (Master Out, Slave In) pin. */ 
#define SYS_GPC_MFPH_PC8MFP_SPI2_SS             (0x02UL<<SYS_GPC_MFPH_PC8MFP_Pos) /* 1st SPI2 slave select pin. */ 

//GPC_MFPH_PC9MFP
#define SYS_GPC_MFPH_PC9MFP_GPIO                (0x00UL<<SYS_GPC_MFPH_PC9MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC9MFP_SPI0_MISO1          (0x01UL<<SYS_GPC_MFPH_PC9MFP_Pos) /* 2nd SPI0 MISO (Master In, Slave Out) pin. */ 
#define SYS_GPC_MFPH_PC9MFP_SPI2_I2SMCLK        (0x02UL<<SYS_GPC_MFPH_PC9MFP_Pos) /*  */

//GPC_MFPH_PC10MFP
#define SYS_GPC_MFPH_PC10MFP_GPIO               (0x00UL<<SYS_GPC_MFPH_PC10MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC10MFP_SPI0_MOSI0         (0x01UL<<SYS_GPC_MFPH_PC10MFP_Pos) /* 1st SPI0 MOSI (Master Out, Slave In) pin. */ 
#define SYS_GPC_MFPH_PC10MFP_TM_BRAKE0          (0x02UL<<SYS_GPC_MFPH_PC10MFP_Pos) /* Brake input pin 0 of Timer. */ 
#define SYS_GPC_MFPH_PC10MFP_DPWM_RN            (0x03UL<<SYS_GPC_MFPH_PC10MFP_Pos) /* DPWM Right-Positive pin. */ 

//GPC_MFPH_PC11MFP
#define SYS_GPC_MFPH_PC11MFP_GPIO               (0x00UL<<SYS_GPC_MFPH_PC11MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC11MFP_SPI0_MISO0         (0x01UL<<SYS_GPC_MFPH_PC11MFP_Pos) /* 1st SPI0 MISO (Master In, Slave Out) pin. */ 
#define SYS_GPC_MFPH_PC11MFP_TM_BRAKE1          (0x02UL<<SYS_GPC_MFPH_PC11MFP_Pos) /* Brake input pin 1 of Timer. */ 
#define SYS_GPC_MFPH_PC11MFP_DPWM_RP            (0x03UL<<SYS_GPC_MFPH_PC11MFP_Pos) /* DPWM Right-Positive pin. */ 

//GPC_MFPH_PC12MFP
#define SYS_GPC_MFPH_PC12MFP_GPIO               (0x00UL<<SYS_GPC_MFPH_PC12MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC12MFP_SPI0_CLK           (0x01UL<<SYS_GPC_MFPH_PC12MFP_Pos) /* SPI0 serial clock pin. */ 
#define SYS_GPC_MFPH_PC12MFP_TM_BRAKE2          (0x02UL<<SYS_GPC_MFPH_PC12MFP_Pos) /* Brake input pin 2 of Timer. */ 
#define SYS_GPC_MFPH_PC12MFP_DPWM_LN            (0x03UL<<SYS_GPC_MFPH_PC12MFP_Pos) /* DPWM Left-Negative pin. */ 

//GPC_MFPH_PC13MFP
#define SYS_GPC_MFPH_PC13MFP_GPIO               (0x00UL<<SYS_GPC_MFPH_PC13MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC13MFP_PWM0_CH3           (0x01UL<<SYS_GPC_MFPH_PC13MFP_Pos) /* PWM0 channel3 output/capture input. */  
#define SYS_GPC_MFPH_PC13MFP_I2C0_SCL           (0x02UL<<SYS_GPC_MFPH_PC13MFP_Pos) /* I2C0 clock pin. */
#define SYS_GPC_MFPH_PC13MFP_DPWM_LP			(0x03UL<<SYS_GPC_MFPH_PC13MFP_Pos) /* DPWM Left-Positive pin. */

//GPC_MFPH_PC14MFP
#define SYS_GPC_MFPH_PC14MFP_GPIO               (0x00UL<<SYS_GPC_MFPH_PC14MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC14MFP_PWM0_CH4           (0x01UL<<SYS_GPC_MFPH_PC14MFP_Pos) /* PWM0 channel4 output/capture input. */ 
#define SYS_GPC_MFPH_PC14MFP_I2C0_SDA           (0x02UL<<SYS_GPC_MFPH_PC14MFP_Pos) /* I2C0 data input/output pin. */ 

//GPC_MFPH_PC15MFP
#define SYS_GPC_MFPH_PC15MFP_GPIO               (0x00UL<<SYS_GPC_MFPH_PC15MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPC_MFPH_PC15MFP_SPI0_SS1           (0x01UL<<SYS_GPC_MFPH_PC15MFP_Pos) /* 2nd SPI0 slave select pin. */ 
#define SYS_GPC_MFPH_PC15MFP_TM_BRAKE3          (0x02UL<<SYS_GPC_MFPH_PC15MFP_Pos) /* Brake input pin 3 of Timer. */ 

/********************* Bit definition of GPD_MFPL register **********************/
//GPD_MFPL_PD0MFP
#define SYS_GPD_MFPL_PD0MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD0MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD0MFP_INT3                (0x01UL<<SYS_GPD_MFPL_PD0MFP_Pos) /* External interrupt3 input pin. */ 
#define SYS_GPD_MFPL_PD0MFP_I2C1_SCL            (0x02UL<<SYS_GPD_MFPL_PD0MFP_Pos) /* I2C1 clock pin. */ 
#define SYS_GPD_MFPL_PD0MFP_DPWM1_N             (0x05UL<<SYS_GPD_MFPL_PD0MFP_Pos) /*  */ 

//GPD_MFPL_PD1MFP
#define SYS_GPD_MFPL_PD1MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD1MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD1MFP_INT4                (0x01UL<<SYS_GPD_MFPL_PD1MFP_Pos) /* External interrupt4 input pin. */ 
#define SYS_GPD_MFPL_PD1MFP_I2C1_SDA            (0x02UL<<SYS_GPD_MFPL_PD1MFP_Pos) /* I2C1 data input/output pin. */ 
#define SYS_GPD_MFPL_PD1MFP_DPWM1_P             (0x05UL<<SYS_GPD_MFPL_PD1MFP_Pos) /*  */ 

//GPD_MFPL_PD2MFP
#define SYS_GPD_MFPL_PD2MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD2MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD2MFP_ETM_TRACE_CLK       (0x01UL<<SYS_GPD_MFPL_PD2MFP_Pos) /* ETM interface clock output pin. */ 
#define SYS_GPD_MFPL_PD2MFP_SPI1_MOSI           (0x02UL<<SYS_GPD_MFPL_PD2MFP_Pos) /* 1st SPI1 MOSI (Master Out, Slave In) pin. */ 
#define SYS_GPD_MFPL_PD2MFP_I2S0_MCLK           (0x03UL<<SYS_GPD_MFPL_PD2MFP_Pos) /* */ 

//GPD_MFPL_PD3MFP
#define SYS_GPD_MFPL_PD3MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD3MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD3MFP_ETM_TRACE_DATA0     (0x01UL<<SYS_GPD_MFPL_PD3MFP_Pos) /* ETM interface output bus bit0. */ 
#define SYS_GPD_MFPL_PD3MFP_SPI1_MISO           (0x02UL<<SYS_GPD_MFPL_PD3MFP_Pos) /* 1st SPI1 MISO (Master In, Slave Out) pin. */ 
#define SYS_GPD_MFPL_PD3MFP_I2S0_LRCK           (0x03UL<<SYS_GPD_MFPL_PD3MFP_Pos) /* */ 
#define SYS_GPD_MFPL_PD3MFP_DMIC_CLK1           (0x04UL<<SYS_GPD_MFPL_PD3MFP_Pos) /* Digital microphone channel 1 clock output pin. */ 

//GPD_MFPL_PD4MFP
#define SYS_GPD_MFPL_PD4MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD4MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD4MFP_ETM_TRACE_DATA1     (0x01UL<<SYS_GPD_MFPL_PD4MFP_Pos) /* ETM interface output bus bit1. */  
#define SYS_GPD_MFPL_PD4MFP_SPI1_CLK            (0x02UL<<SYS_GPD_MFPL_PD4MFP_Pos) /* SPI1 serial clock pin. */ 
#define SYS_GPD_MFPL_PD4MFP_I2S0_DI             (0x03UL<<SYS_GPD_MFPL_PD4MFP_Pos) /* */ 
#define SYS_GPD_MFPL_PD4MFP_DMIC_DAT1           (0x04UL<<SYS_GPD_MFPL_PD4MFP_Pos) /* Digital microphone channel 1 data input pin. */  

//GPD_MFPL_PD5MFP
#define SYS_GPD_MFPL_PD5MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD5MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD5MFP_ETM_TRACE_DATA2     (0x01UL<<SYS_GPD_MFPL_PD5MFP_Pos) /* ETM interface output bus bit2. */ 
#define SYS_GPD_MFPL_PD5MFP_SPI1_SS             (0x02UL<<SYS_GPD_MFPL_PD5MFP_Pos) /* 1st SPI1 slave select pin. */ 
#define SYS_GPD_MFPL_PD5MFP_I2S0_DO             (0x03UL<<SYS_GPD_MFPL_PD5MFP_Pos) /* */
#define SYS_GPD_MFPL_PD5MFP_DMIC_CLK0           (0x04UL<<SYS_GPD_MFPL_PD5MFP_Pos) /* Digital microphone channel 0 clock output pin. */ 
#define SYS_GPD_MFPL_PD5MFP_DPWM0_N             (0x05UL<<SYS_GPD_MFPL_PD5MFP_Pos) /*  */ 

//GPD_MFPL_PD6MFP
#define SYS_GPD_MFPL_PD6MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD6MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPL_PD6MFP_ETM_TRACE_DATA3     (0x01UL<<SYS_GPD_MFPL_PD6MFP_Pos) /* ETM interface output bus bit3. */ 
#define SYS_GPD_MFPL_PD6MFP_SPI1_I2SMCLK        (0x02UL<<SYS_GPD_MFPL_PD6MFP_Pos) /*  */ 
#define SYS_GPD_MFPL_PD6MFP_I2S0_BCLK           (0x03UL<<SYS_GPD_MFPL_PD6MFP_Pos) /*  */ 
#define SYS_GPD_MFPL_PD6MFP_DMIC_DAT0           (0x04UL<<SYS_GPD_MFPL_PD6MFP_Pos) /* Digital microphone channel 0 data input pin. */ 
#define SYS_GPD_MFPL_PD6MFP_DPWM0_P             (0x05UL<<SYS_GPD_MFPL_PD6MFP_Pos) /*  */ 

//GPD_MFPL_PD7MFP
#define SYS_GPD_MFPL_PD7MFP_GPIO                (0x00UL<<SYS_GPD_MFPL_PD7MFP_Pos) /* General purpose digital I/O pin. */  
#define SYS_GPD_MFPL_PD7MFP_PWM0_CH5            (0x01UL<<SYS_GPD_MFPL_PD7MFP_Pos) /* PWM0 channel5 output/capture input. */ 
#define SYS_GPD_MFPL_PD7MFP_INT1                (0x02UL<<SYS_GPD_MFPL_PD7MFP_Pos) /* External interrupt1 input pin. */  

/********************* Bit definition of GPD_MFPH register **********************/
//GPD_MFPH_PD8MFP
#define SYS_GPD_MFPH_PD8MFP_GPIO                (0x00UL<<SYS_GPD_MFPH_PD8MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD8MFP_ICE_CLK             (0x01UL<<SYS_GPD_MFPH_PD8MFP_Pos) /* Serial wired debugger clock pin. */ 
#define SYS_GPD_MFPH_PD8MFP_T0                  (0x02UL<<SYS_GPD_MFPH_PD8MFP_Pos) /*  */

//GPD_MFPH_PD9MFP
#define SYS_GPD_MFPH_PD9MFP_GPIO                (0x00UL<<SYS_GPD_MFPH_PD9MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD9MFP_ICE_DAT             (0x01UL<<SYS_GPD_MFPH_PD9MFP_Pos) /* Serial wired debugger data pin. */ 
#define SYS_GPD_MFPH_PD9MFP_T0_EXT              (0x02UL<<SYS_GPD_MFPH_PD9MFP_Pos) /* Timer0 external capture input. */ 

//GPD_MFPH_PD10MFP
#define SYS_GPD_MFPH_PD10MFP_GPIO               (0x00UL<<SYS_GPD_MFPH_PD10MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD10MFP_INT5               (0x01UL<<SYS_GPD_MFPH_PD10MFP_Pos) /* External interrupt5 input pin. */ 
#define SYS_GPD_MFPH_PD10MFP_EADC0_ST           (0x02UL<<SYS_GPD_MFPH_PD10MFP_Pos) /*  */ 

//GPD_MFPH_PD11MFP
#define SYS_GPD_MFPH_PD11MFP_GPIO               (0x00UL<<SYS_GPD_MFPH_PD11MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD11MFP_UART0_TXD          (0x01UL<<SYS_GPD_MFPH_PD11MFP_Pos) /* Data transmitter output pin for UART0. */ 
#define SYS_GPD_MFPH_PD11MFP_INT2               (0x02UL<<SYS_GPD_MFPH_PD11MFP_Pos) /* External interrupt2 input pin. */ 

//GPD_MFPH_PD12MFP
#define SYS_GPD_MFPH_PD12MFP_GPIO               (0x00UL<<SYS_GPD_MFPH_PD12MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD12MFP_UART0_RXD          (0x01UL<<SYS_GPD_MFPH_PD12MFP_Pos) /* Data receiver input pin for UART0. */ 
#define SYS_GPD_MFPH_PD12MFP_INT3               (0x02UL<<SYS_GPD_MFPH_PD12MFP_Pos) /* External interrupt3 input pin. */ 

//GPD_MFPH_PD13MFP
#define SYS_GPD_MFPH_PD13MFP_GPIO               (0x00UL<<SYS_GPD_MFPH_PD13MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD13MFP_SPI0_SS1           (0x01UL<<SYS_GPD_MFPH_PD13MFP_Pos) /* 2nd SPI0 slave select pin. */ 
#define SYS_GPD_MFPH_PD13MFP_EADC0_CH10         (0x02UL<<SYS_GPD_MFPH_PD13MFP_Pos) /*  */ 

//GPD_MFPH_PD14MFP
#define SYS_GPD_MFPH_PD14MFP_GPIO               (0x00UL<<SYS_GPD_MFPH_PD14MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD14MFP_UART0_nCTS         (0x01UL<<SYS_GPD_MFPH_PD14MFP_Pos) /* Clear to Send input pin for UART3. */ 
#define SYS_GPD_MFPH_PD14MFP_EADC0_CH11         (0x02UL<<SYS_GPD_MFPH_PD14MFP_Pos) /*  */ 
#define SYS_GPD_MFPH_PD14MFP_I2C1_SCL           (0x05UL<<SYS_GPD_MFPH_PD14MFP_Pos) /*  */ 

//GPD_MFPH_PD15MFP
#define SYS_GPD_MFPH_PD15MFP_GPIO               (0x00UL<<SYS_GPD_MFPH_PD15MFP_Pos) /* General purpose digital I/O pin. */ 
#define SYS_GPD_MFPH_PD15MFP_UART0_nRTS         (0x01UL<<SYS_GPD_MFPH_PD15MFP_Pos) /* Request to Send output pin for UART0. */ 
#define SYS_GPD_MFPH_PD15MFP_EADC0_CH12         (0x02UL<<SYS_GPD_MFPH_PD15MFP_Pos) /*  */ 
#define SYS_GPD_MFPH_PD15MFP_I2C1_SDA           (0x05UL<<SYS_GPD_MFPH_PD15MFP_Pos) /*  */ 

/*@}*/ /* end of group I94100_SYS_EXPORTED_CONSTANTS */

/** @addtogroup I94100_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
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
#define SYS_DISABLE_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

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
  * @brief      Set Brown-out detector reset or interrupt function
  * @param      u32function is Brown-out detector function. Including :
  *             - \ref SYS_BODCTL_BODRSTEN, "RESET" function enabled.
  *             - \ref SYS_BODCTL_BODINTEN,	"INTERRUPT" function enabled.			
  * @return     None
  * @details    This macro set Brown-out detect function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_FUNCTION(u32function)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODRSTEN_Msk) | (u32function))

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_3_0V
  *             - \ref SYS_BODCTL_BODVL_2_8V
  *             - \ref SYS_BODCTL_BODVL_2_6V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  *             - \ref SYS_BODCTL_BODVL_1_6V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Set Brown-out detector de-glitch time
  * @param[in]  u32time is de-glitch time selection. Including :
  *             - \ref SYS_BODCTL_BODDGSEL_256HCLK
  *             - \ref SYS_BODCTL_BODDGSEL_128HCLK
  *             - \ref SYS_BODCTL_BODDGSEL_64HCLK
  *             - \ref SYS_BODCTL_BODDGSEL_32HCLK
  *             - \ref SYS_BODCTL_BODDGSEL_16HCLK
  *             - \ref SYS_BODCTL_BODDGSEL_8HCLK
  *             - \ref SYS_BODCTL_BODDGSEL_4HCLK
  *				- \ref SYS_BODCTL_BODDGSEL_LIRC
  * @return     None
  * @details    Set Brown-out detector de-glitch time.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_DEGLITCH_TIME(u32time)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODDGSEL_Msk) | (u32time))

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
  * @brief      Set Low-Voltage-Reset de-glitch time
  * @param[in]  u32time is de-glitch time selection. Including :
  *             - \ref SYS_BODCTL_LVRDGSEL_256HCLK
  *             - \ref SYS_BODCTL_LVRDGSEL_128HCLK
  *             - \ref SYS_BODCTL_LVRDGSEL_64HCLK
  *             - \ref SYS_BODCTL_LVRDGSEL_32HCLK
  *             - \ref SYS_BODCTL_LVRDGSEL_16HCLK
  *             - \ref SYS_BODCTL_LVRDGSEL_8HCLK
  *             - \ref SYS_BODCTL_LVRDGSEL_4HCLK
  *				- \ref SYS_BODCTL_LVRDGSEL_0HCLK
  * @return     None
  * @details    Set Low-Voltage-Reset de-glitch time.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_LVR_DEGLITCH_TIME(u32time)     //(SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_LVRDGSEL_Msk) | (u32time))

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
  * @brief      Enable Trim HIRC
  * @param      Enable interrupt type(SYS_IRCTIEN_TRIMFAIL_INT_MASK or SYS_IRCTIEN_CLKERROR_INT_MASK)
  * @return     None
  * @details    This macro enable trim HIRC interrupt function.(clock error & trim fail)
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_TRIMHIRC_INT(u32Mask)     (SYS->IRCTIEN = (SYS->IRCTIEN&~(SYS_IRCTIEN_CLKEIEN_Msk|SYS_IRCTIEN_TFAILIEN_Msk))|u32Mask)

/**
  * @brief      Disable Trim HIRC
  * @param      None
  * @return     None
  * @details    This macro disable trim HIRC interrupt function.(clock error & trim fail)
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_TRIMHIRC_INT()           (SYS->IRCTIEN = 0)

/**
  * @brief      Get trim HIRC interrupt flag
  * @param      Check triggered interrupt flag mask.
  *             - \ref SYS_IRCTISTS_TRIMFAIL_INT_FLAG
  *             - \ref SYS_IRCTISTS_CLKERROR_INT_FLAG
  * @retval     Triggered interrupt flag.
  * @details    This macro get trim HIRC interrupt flag.
  */
#define SYS_GET_TRIMHIRC_INT_FLAG(u32Mask)   (SYS->IRCTISTS & (u32Mask))

/**
  * @brief      Clear trim HIRC interrupt flag
  * @param      Clear interrupt flag mask.
  *             - \ref SYS_IRCTISTS_TRIMFAIL_INT_FLAG
  *             - \ref SYS_IRCTISTS_CLKERROR_INT_FLAG
  * @return     None
  * @details    This macro clear trim HIRC interrupt flag.
  */
#define SYS_CLEAR_TRIMHIRC_INT_FLAG(u32Mask) (SYS->IRCTISTS |= (u32Mask))

/**
  * @brief      Trim HIRC is done(lock set frequency)
  * @param      None
  * @return     0   The internal high-speed oscillator frequency doesn't lock at setting frequency yest 
  * @retval     >=1 The internal high-speed oscillator frequency locked lock at setting frequency. 
  * @details    This macro get Trim HIRC is done status.
  */
#define SYS_IS_TRIMHIRC_DONE()               (SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk)

#define SYS_SET_TRIMHIRC_LOOPSEL(u32LoopSel)     (SYS->IRCTCTL = (SYS->IRCTCTL&~SYS_IRCTCTL_LOOPSEL_Msk) | u32LoopSel)
#define SYS_SET_TRIMHIRC_RETRYCNT(u32RetryCnt)   (SYS->IRCTCTL = (SYS->IRCTCTL&~SYS_IRCTCTL_RETRYCNT_Msk) | u32RetryCnt)
#define SYS_ENABLE_TRIMHIRC_CLKERRSTOP()         (SYS->IRCTCTL |= SYS_IRCTCTL_CESTOPEN_Msk)
#define SYS_DISABLE_TRIMHIRC_CLKERRSTOP()        (SYS->IRCTCTL &= ~SYS_IRCTCTL_CESTOPEN_Msk)
#define SYS_SET_TRIMHIRC_REFCLK(u32RefClk)       (SYS->IRCTCTL = (SYS->IRCTCTL&~SYS_IRCTCTL_REFCKSEL_Msk) | u32RefClk)

/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
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
void SYS_EnableTrimHIRC(uint32_t u32FreqSel);
void SYS_DisableTrimHIRC(void);
void SYS_LockReg(void);
void SYS_UnlockReg(void);
void SYS_Lock(uint8_t u8Lock);
uint8_t SYS_Unlock(void);

/*@}*/ /* end of group I94100_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_SYS_Driver */

/*@}*/ /* end of group I94100_Device_Driver */
 
#ifdef __cplusplus
}
#endif

#endif  //__SYS_H__

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

