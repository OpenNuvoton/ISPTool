/**************************************************************************//**
 * @file     sys_reg.h
 * @version  V1.00
 * @brief    SYS register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_REG_H__
#define __SYS_REG_H__


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */




/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller (SYS)
    Memory Mapped Structure for SYS Controller
@{ */



typedef struct
{



/**
 * @var GCR_T::PDID
 * Offset: 0x00  Part Device Identification Number Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDID      |Part Device Identification Number
 * |        |          |This register reflects device part number code.
 * |        |          |Software can read this register to identify which device is used.
 * @var GCR_T::RSTSRC
 * Offset: 0x04  System Reset Source Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RSTS_POR  |Power-on Reset Flag
 * |        |          |The RSTS_POR flag is set by the "reset signal" from the Power-On Reset (POR) controller or bit CHIP_RST (IPRSTC1[0]) to indicate the previous reset source.
 * |        |          |0 = No reset from POR or CHIP_RST (IPRSTC1[0]).
 * |        |          |1 = Power-on Reset (POR) or CHIP_RST (IPRSTC1[0]) had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[1]     |RSTS_RESET|Reset Pin Reset Flag
 * |        |          |The RSTS_RESET flag is set by the "Reset Signal" from the /RESET pin to indicate the previous reset source.
 * |        |          |0 = No reset from /RESET pin.
 * |        |          |1 = The Pin /RESET had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[2]     |RSTS_WDT  |Watchdog Reset Flag
 * |        |          |The RSTS_WDT flag is set by The "Reset Signal" from the Watchdog Timer to indicate the previous reset source
 * |        |          |0 = No reset from watchdog timer.
 * |        |          |1 = The watchdog timer had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[3]     |RSTS_LVR  |Low Voltage Reset Flag
 * |        |          |The RSTS_LVR flag is set by The "Reset Signal" from the Low-Voltage-Reset Controller to indicate the previous reset source
 * |        |          |0 = No reset from LVR.
 * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[4]     |RSTS_BOD  |Brown-out Detector Reset Flag
 * |        |          |The RSTS_BOD flag is set by the "Reset Signal" from the Brown-Out Detector to indicate the previous reset source.
 * |        |          |0 = No reset from BOD.
 * |        |          |1 = The BOD had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[5]     |RSTS_SYS  |System Reset Flag
 * |        |          |The RSTS_SYS flag is set by the "Reset Signal" from the Cortex-M0 kernel to indicate the previous reset source.
 * |        |          |0 = No reset from Cortex-M0.
 * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 kernel.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[7]     |RSTS_CPU  |CPU Reset Flag
 * |        |          |The RSTS_CPU flag is set by hardware if software writes CPU_RST (IPRSTC1[1]) 1 to reset Cortex-M0 CPU kernel and flash. Memory Controller (FMC)
 * |        |          |0 = No reset from CPU.
 * |        |          |1 = Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST(IPRSTC1[1]) to 1.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * @var GCR_T::IPRSTC1
 * Offset: 0x08  IP Reset Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CHIP_RST  |CHIP One-Shot Reset (Write Protect)
 * |        |          |Setting this bit will reset the whole chip, including CPU kernel and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
 * |        |          |The CHIP_RST is the same as the POR reset, all the chip controllers are reset and the chip setting from flash are also reload.
 * |        |          |0 = CHIP normal operation.
 * |        |          |1 = CHIP one-shot reset.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[1]     |CPU_RST   |CPU Kernel One-Shot Reset (Write Protect)
 * |        |          |Setting this bit will only reset the CPU kernel and Flash Memory Controller(FMC), and this bit will automatically return 0 after the two clock cycles.
 * |        |          |0 = CPU normal operation.
 * |        |          |1 = CPU one-shot reset.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[2]     |PDMA_RST  |PDMA Controller Reset (Write Protect)
 * |        |          |Setting this bit to 1 will generate a reset signal to the PDMA.
 * |        |          |User need to set this bit to 0 to release from reset state.
 * |        |          |0 = PDMA controller normal operation.
 * |        |          |1 = PDMA controller reset.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[3]     |EBI_RST   |EBI Controller Reset (Write Protect)
 * |        |          |Setting this bit to 1 will generate a reset signal to the EBI.
 * |        |          |User need to set this bit to 0 to release from reset state.
 * |        |          |0 = EBI controller normal operation.
 * |        |          |1 = EBI controller reset.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var GCR_T::IPRSTC2
 * Offset: 0x0C  IP Reset Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |GPIO_RST  |GPIO Controller Reset
 * |        |          |0 = GPIO controller normal operation.
 * |        |          |1 = GPIO controller reset.
 * |[2]     |TMR0_RST  |Timer0 Controller Reset
 * |        |          |0 = Timer0 controller normal operation.
 * |        |          |1 = Timer0 controller reset.
 * |[3]     |TMR1_RST  |Timer1 Controller Reset
 * |        |          |0 = Timer1 controller normal operation.
 * |        |          |1 = Timer1 controller reset.
 * |[4]     |TMR2_RST  |Timer2 Controller Reset
 * |        |          |0 = Timer2 controller normal operation.
 * |        |          |1 = Timer2 controller reset.
 * |[5]     |TMR3_RST  |Timer3 Controller Reset
 * |        |          |0 = Timer3 controller normal operation.
 * |        |          |1 = Timer3 controller reset.
 * |[8]     |I2C0_RST  |I2C0 Controller Reset
 * |        |          |0 = I2C0 controller normal operation.
 * |        |          |1 = I2C0 controller reset.
 * |[9]     |I2C1_RST  |I2C1 Controller Reset
 * |        |          |0 = I2C1 controller normal operation.
 * |        |          |1 = I2C1 controller reset.
 * |[12]    |SPI0_RST  |SPI0 Controller Reset
 * |        |          |0 = SPI0 controller normal operation.
 * |        |          |1 = SPI0 controller reset.
 * |[13]    |SPI1_RST  |SPI1 Controller Reset
 * |        |          |0 = SPI1 controller normal operation.
 * |        |          |1 = SPI1 controller reset.
 * |[16]    |UART0_RST |UART0 Controller Reset
 * |        |          |0 = UART0 controller normal operation.
 * |        |          |1 = UART0 controller reset.
 * |[17]    |UART1_RST |UART1 Controller Reset
 * |        |          |0 = UART1 controller normal operation.
 * |        |          |1 = UART1 controller reset.
 * |[18]    |UART2_RST |UART2 Controller Reset
 * |        |          |0 = UART2 controller normal operation.
 * |        |          |1 = UART2 controller reset.
 * |[20]    |PWM03_RST |PWM03 Controller Reset
 * |        |          |0 = PWM03 controller normal operation.
 * |        |          |1 = PWM03 controller reset.
 * |[21]    |PWM45_RST |PWM45 Controller Reset
 * |        |          |0 = PWM45 controller normal operation.
 * |        |          |1 = PWM45 controller reset.
 * |[27]    |USBD_RST  |USB Device Controller Reset
 * |        |          |0 = USB device controller normal operation.
 * |        |          |1 = USB device controller reset.
 * |[28]    |ADC_RST   |ADC Controller Reset
 * |        |          |0 = ADC controller normal operation.
 * |        |          |1 = ADC controller reset.
 * @var GCR_T::BODCR
 * Offset: 0x18  Brown-out Detector Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BOD_EN    |Brown-Out Detector Enable Bit (Write Protect)
 * |        |          |The default value is set by flash memory controller user configuration register CBODEN (Config0[23]) bit.
 * |        |          |0 = Brown-out Detector function Disabled.
 * |        |          |1 = Brown-out Detector function Enabled.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[2:1]   |BOD_VL    |Brown-Out Detector Threshold Voltage Selection (Write Protect)
 * |        |          |The default value is set by flash controller user configuration register CBOV (Config0[22:21]) bit.
 * |        |          |00 = Brown-out voltage is 2.2V.
 * |        |          |01 = Brown-out voltage is 2.7V.
 * |        |          |10 = Brown-out voltage is 3.7V.
 * |        |          |11 = Brown-out voltage is 4.4V.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[3]     |BOD_RSTEN |Brown-out Reset Enable Control (Write Protect)
 * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
 * |        |          |While the BOD function is enabled (BOD_EN high) and BOD interrupt function is enabled (BOD_RSTEN
 * |        |          |low), BOD will assert an interrupt if BOD_OUT is high.
 * |        |          |BOD interrupt will keep till to the BOD_EN set to 0. BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BOD_EN low).
 * |        |          |1 = Brown-out "RESET" function Enabled.
 * |        |          |Note1: While the Brown-out Detector function is enabled (BOD_EN high) and BOD reset function is enabled (BOD_RSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BOD_OUT high).
 * |        |          |Note2: The default value is set by flash controller user configuration register CBORST (Config0[20]) bit.
 * |        |          |Note3: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[4]     |BOD_INTF  |Brown-out Detector Interrupt Flag
 * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BOD_VL setting.
 * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BOD_VL setting or the VDD is raised up through the voltage of BOD_VL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[5]     |BOD_LPM   |Brown-out Detector Low Power Mode (Write Protection)
 * |        |          |0 = BOD operated in Normal mode (default).
 * |        |          |1 = BOD Low Power mode Enabled.
 * |        |          |Note1: The BOD consumes about 100 uA in Normal mode, and the low power mode can reduce the current to about 1/10 but slow the BOD response.
 * |        |          |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[6]     |BOD_OUT   |Brown-out Detector Output Status
 * |        |          |0 = Brown-out Detector output status is 0. It means the detected voltage is higher than BOD_VL setting or BOD_EN is 0.
 * |        |          |1 = Brown-out Detector output status is 1. It means the detected voltage is lower than BOD_VL setting. If the BOD_EN is 0, BOD function disabled, this bit always responds to 0.
 * |[7]     |LVR_EN    |Low Voltage Reset Enable Bit (Write Protect)
 * |        |          |The LVR function reset the chip when the input power voltage is lower than LVR circuit setting.
 * |        |          |LVR function is enabled by default.
 * |        |          |0 = Low Voltage Reset function Disabled.
 * |        |          |1 = Low Voltage Reset function Enabled - After enabling the bit, the LVR function will be active with 100us delay for LVR output stable (default).
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var GCR_T::TEMPCR
 * Offset: 0x1C  Temperature Sensor Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VTEMP_EN  |Temperature Sensor Enable Control
 * |        |          |This bit is used to enable/disable temperature sensor function.
 * |        |          |0 = Temperature sensor function Disabled (default).
 * |        |          |1 = Temperature sensor function Enabled.
 * |        |          |Note: After this bit is set to 1, the value of temperature can be obtained from ADC conversion result by ADC channel selecting channel 7 and alternative multiplexer channel selecting temperature sensor.
 * |        |          |Please refer to the ADC function chapter for detail ADC conversion functional description.
 * @var GCR_T::PORCR
 * Offset: 0x24  Power-on-Reset Controller Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |POR_DIS_CODE|Power-on Reset Enable Control (Write Protect)
 * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
 * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
 * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
 * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
 * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
 * @var GCR_T::GPA_MFP
 * Offset: 0x30  GPIOA Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GPA_MFP0  |PA.0 Pin Function Selection
 * |        |          |0 = GPIO function is selected.
 * |        |          |1 = ADC0 function is selected.
 * |[1]     |GPA_MFP1  |PA.1 Pin Function Selection
 * |        |          |Bits EBI_HB_EN[4] (ALT_MFP[20]), EBI_EN (ALT_MFP[11]) and GPA_MFP[1] determine the PA.1 function.
 * |        |          |(EBI_HB_EN[4], EBI_EN, GPA_MFP1) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC1 function is selected.
 * |        |          |(1, 1, 1) = AD12 function is selected.
 * |[2]     |GPA_MFP2  |PA.2 Pin Function Selection
 * |        |          |Bits EBI_HB_EN[3] (ALT_MFP[19]), EBI_EN (ALT_MFP[11]) and GPA_MFP[2] determine the PA.2 function.
 * |        |          |(EBI_HB_EN[3], EBI_EN, GPA_MFP2) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC2 function is selected.
 * |        |          |(1, 1, 1) = AD11 function is selected.
 * |[3]     |GPA_MFP3  |PA.3 Pin Function Selection   
 * |        |          |Bits EBI_HB_EN[2] (ALT_MFP[18]), EBI_EN (ALT_MFP[11]) and GPA_MFP[3] determine the PA.3 function.
 * |        |          |(EBI_HB_EN[2], EBI_EN, GPA_MFP3) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC3 function is selected.
 * |        |          |(1, 1, 1) = AD10 function is selected.
 * |[4]     |GPA_MFP4  |PA.4 Pin Function Selection     
 * |        |          |Bits EBI_HB_EN[1] (ALT_MFP[17]), EBI_EN (ALT_MFP[11]) and GPA_MFP[4] determine the PA.4 function.
 * |        |          |(EBI_HB_EN[1], EBI_EN, GPA_MFP4) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC4 function is selected.
 * |        |          |(1, 1, 1) = AD9 function is selected.
 * |[5]     |GPA_MFP5  |PA.5 Pin Function Selection    
 * |        |          |Bits EBI_HB_EN[0] (ALT_MFP[16]), EBI_EN (ALT_MFP[11]) and GPA_MFP[5] determine the PA.5 function.
 * |        |          |(EBI_HB_EN[0], EBI_EN, GPA_MFP5) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC5 function is selected.
 * |        |          |(1, 1, 1) = AD8 function is selected.
 * |[6]     |GPA_MFP6  |PA.6 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPA_MFP[6] determine the PA.6 function.
 * |        |          |(EBI_EN, GPA_MFP6) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = ADC6 function is selected.
 * |        |          |(1, 1) = AD7 function is selected.
 * |[8]     |GPA_MFP8  |PA.8 Pin Function Selection 
 * |        |          |Bit GPA_MFP[8] determines the PA.8 function.
 * |        |          |0 = GPIO function is selected to the pin PA.8.
 * |        |          |1 = I2C0_SDA function is selected to the pin PA.8.
 * |[9]     |GPA_MFP9  |PA.9 Pin Function Selection 
 * |        |          |Bit GPA_MFP[9] determines the PA.9 function.
 * |        |          |0 = GPIO function is selected.
 * |        |          |1 = I2C0_SCL function is selected.
 * |[10]    |GPA_MFP10 |PA.10 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPA_MFP[10] determine the PA.10 function.
 * |        |          |(EBI_EN, GPA_MFP10) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = I2C1_SDA function is selected.
 * |        |          |(1, 1) = nWR(EBI) function is selected.
 * |[11]    |GPA_MFP11 |PA.11 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPA_MFP[11] determine the PA.11 function.
 * |        |          |(EBI_EN, GPA_MFP11) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = I2C1_SCL function is selected.
 * |        |          |(1, 1) = nRD(EBI) function is selected.
 * |[12]    |GPA_MFP12 |PA.12 Pin Function Selection
 * |        |          |Bits EBI_HB_EN[5] (ALT_MFP[21]), EBI_EN (ALT_MFP[11]) and GPA_MFP[12] determine the PA.12 function.
 * |        |          |(EBI_HB_EN[5], EBI_EN, GPA_MFP12) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM0 function is selected.
 * |        |          |(1, 1, 1) = AD13 function is selected.
 * |[13]    |GPA_MFP13 |PA.13 Pin Function Selection     
 * |        |          |Bits EBI_HB_EN[6] (ALT_MFP[22]), EBI_EN (ALT_MFP[11]) and GPA_MFP[13] determine the PA.13 function.
 * |        |          |(EBI_HB_EN[6], EBI_EN, GPA_MFP13) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM1 function is selected.
 * |        |          |(1, 1, 1) = AD14 function is selected.
 * |[14]    |GPA_MFP14 |PA.14 Pin Function Selection
 * |        |          |Bits EBI_HB_EN[7] (ALT_MFP[23]), EBI_EN (ALT_MFP[11]) and GPA_MFP[14] determine the PA.14 function.
 * |        |          |(EBI_HB_EN[7], EBI_EN, GPA_MFP14) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM2 function is selected.
 * |        |          |(1, 1, 1) = AD15 function is selected..     
 * |[15]    |GPA_MFP15 |PA.15 Pin Function Selection    
 * |        |          |0 = GPIOA function is selected.
 * |        |          |1 = PWM3 function is selected.
 * |[31:16] |GPA_TYPEn |Trigger Function Selection
 * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::GPB_MFP
 * Offset: 0x34  GPIOB Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GPB_MFP0  |PB.0 Pin Function Selection     
 * |        |          |Bit GPB_MFP[0] determines the PB.0 function.
 * |        |          |0 = GPIO function is selected to the pin PB.0.
 * |        |          |1 = UART0_RXD function is selected to the pin PB.0.       
 * |[1]     |GPB_MFP1  |PB.1 Pin Function Selection
 * |        |          |Bit GPB_MFP[1] determines the PB.1 function.
 * |        |          |0 = GPIO function is selected to the pin PB.1.
 * |        |          |1 = UART0_TXD function is selected to the pin PB.1.     
 * |[2]     |GPB_MFP2  |PB.2 Pin Function Selection
 * |        |          |Bits EBI_nWRL_EN (ALT_MFP[13]), EBI_EN (ALT_MFP[11]), PB2_TM2 (ALT_MFP2[4]), PB2_T2EX (ALT_MFP[26]) and GPB_MFP[2] determine the PB.2 function.
 * |        |          |(EBI_nWRL_EN, EBI_EN, PB2_TM2, PB2_T2EX, GPB_MFP2) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nRTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM2_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM2 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRL(EBI) function is selected. 
 * |[3]     |GPB_MFP3  |PB.3 Pin Function Selection
 * |        |          |Bits EBI_nWRH_EN (ALT_MFP[14]), EBI_EN (ALT_MFP[11]), PB3_TM3 (ALT_MFP2[5]), PB3_T3EX (ALT_MFP[27]) and GPB_MFP[3] determine the PB.3 function.
 * |        |          |(EBI_nWRH_EN, EBI_EN, PB3_TM3, PB3_T3EX, GPB_MFP3) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nCTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM3_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM3 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRH(EBI) function is selected.       
 * |[4]     |GPB_MFP4  |PB.4 Pin Function Selection
 * |        |          |Bit GPB_MFP[4] determines the PB.4 function.
 * |        |          |0 = GPIO function is selected to the pin PB.4.
 * |        |          |1 = UART1_RXD function is selected to the pin PB.4.     
 * |[5]     |GPB_MFP5  |PB.5 Pin Function Selection
 * |        |          |Bit GPB_MFP[5] determines the PB.5 function.
 * |        |          |0 = GPIO function is selected to the pin PB.5.
 * |        |          |1 = UART1_TXD function is selected to the pin PB.5.     
 * |[6]     |GPB_MFP6  |PB.6 Pin Function Selection 
 * |        |          |Bits EBI_EN (ALT_MFP[11]), GPB_MFP[6] determines the PB.6 function.
 * |        |          |(EBI_EN, GPB_MFP6) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected to the pin PB.6.
 * |        |          |(0, 1) = UART1_nRTS function is selected to the pin PB.6.
 * |        |          |(1, 1) = ALE(EBI) function is selected to the pin PB.6.
 * |[7]     |GPB_MFP7  |PB.7 Pin Function Selection
 * |        |          |Bit EBI_EN (ALT_MFP[11]), GPB_MFP[7] determines the PB.7 function.
 * |        |          |(EBI_EN, GPB_MFP7) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected to the pin PB.7.
 * |        |          |(0, 1) = UART1_nCTS function is selected to the pin PB.7.
 * |        |          |(1, 1) = nCS(EBI) function is selected to the pin PB.7.         
 * |[8]     |GPB_MFP8  |PB.8 Pin Function Selection
 * |        |          |Bits PB8_CLKO (ALT_MFP[29]) and GPB_MFP[8] determine the PB.8 function.
 * |        |          |(PB8_CLKO, GPB_MFP8) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM0 function is selected to the pin PB.8.
 * |        |          |(1, 0) = STADC function is selected to the pin PB.8.
 * |        |          |(1, 1) = CLKO function is selected to the pin PB.8.        
 * |[9]     |GPB_MFP9  |PB.9 Pin Function Selection
 * |        |          |Bits PB9_S11 (ALT_MFP[1]) and GPB_MFP[9] determine the PB.9 function.
 * |        |          |(PB9_S11, GPB_MFP9) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM1 function is selected.
 * |        |          |(1, 1) = UART2_TXD function is selected.     
 * |[10]    |GPB_MFP10 |PB.10 Pin Function Selection
 * |        |          |Bits PB10_S01 (ALT_MFP[0]) and GPB_MFP[10] determine the PB.10 function.
 * |        |          |(PB10_S01, GPB_MFP10) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM2 function is selected.
 * |        |          |(1, 1) = UART2_RXD function is selected.
 * |[11]    |GPB_MFP11 |PB.11 Pin Function Selection
 * |        |          |Bits PB11_PWM4 (ALT_MFP[4]) and GPB_MFP[11] determine the PB.11 function.
 * |        |          |(PB11_PWM4, GPB_MFP11) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM3 function is selected.     
 * |        |          |(1, 1) = PWM4 function is selected.     
 * |[13]    |GPB_MFP13 |PB.13 Pin Function Selection
 * |        |          |0 = GPIO function is selected to the pin PB.13.
 * |        |          |1 = AD1 function is selected.
 * |[14]    |GPB_MFP14 |PB.14 Pin Function Selection 
 * |        |          |Bits PB14_15_EBI (ALT_MFP2[1]) and GPB_MFP[14] determine the PB.14 function.
 * |        |          |(PB14_15_EBI, GPB_MFP14) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = INT0 function is selected.
 * |        |          |(1, 1) = AD0 function is selected.
 * |[15]    |GPB_MFP15 |PB.15 Pin Function Selection
 * |        |          |Bits PB14_15_EBI (ALT_MFP2[1]), PB15_T0EX (ALT_MFP[24]), PB15_TM0 (ALT_MFP2[2]) and GPB_MFP[15] determine the PB.15 function.
 * |        |          |(PB14_15_EBI, PB15_T0EX, PB15_TM0, GPB_MFP15) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 1) = INT1 function is selected.
 * |        |          |(0, 0, 1, 1) = TM0 function is selected.
 * |        |          |(0, 1, 0, 0) = ADC11 function is selected.
 * |        |          |(0, 1, 0, 1) = TM0_EXT function is selected.
 * |        |          |(1, 0, 0, 1) = AD6 function is selected. 
 * |[31:16] |GPB_TYPEn |Trigger Function Selection
 * |        |          |0 = GPIOB[15:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = GPIOB[15:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::GPC_MFP
 * Offset: 0x38  GPIOC Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GPC_MFP0  |PC.0 Pin Function Selection
 * |        |          |Bit GPC_MFP[0] determine the PC.0 function. 
 * |        |          |0 = GPIO function is selected.
 * |        |          |1 = SPI0_SS0 function is selected.
 * |[1]     |GPC_MFP1  |PC.1 Pin Function Selection
 * |        |          |Bit GPC_MFP[1] determine the PC.1 function. 
 * |        |          |0 = GPIO function is selected.
 * |        |          |1 = SPI0_CLK function is selected.   
 * |[2]     |GPC_MFP2  |PC.2 Pin Function Selection
 * |        |          |Bit GPC_MFP[2] determine the PC.2 function. 
 * |        |          |0 = GPIO function is selected.
 * |        |          |1 = SPI0_MISO0 function is selected.
 * |[3]     |GPC_MFP3  |PC.3 Pin Function Selection
 * |        |          |Bit GPC_MFP[3] determine the PC.3 function.
 * |        |          |0 = GPIO function is selected.
 * |        |          |1 = SPI0_MOSI0 function is selected.   
 * |[6]     |GPC_MFP6  |PC.6 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPC_MFP[6] determine the PC.6 function.
 * |        |          |(EBI_EN, GPB_MFP6) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = ADC9 function is selected.
 * |        |          |(1, 1) = AD4 function is selected. 
 * |[7]     |GPC_MFP7  |PC.7 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPC_MFP[7] determine the PC.7 function.
 * |        |          |(EBI_EN, GPC_MFP7) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC7 function is selected.
 * |        |          |(1, 0, 1) = AD5 function is selected. 
 * |[8]     |GPC_MFP8  |PC.8 Pin Function Selection
 * |        |          |Bits EBI_MCLK_EN (ALT_MFP[12]), EBI_EN (ALT_MFP[11]), GPC_MFP[8] determine the PC.8 function.
 * |        |          |(EBI_MCLK_EN, EBI_EN, GPC_MFP8) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected to the pin PC.8.
 * |        |          |(0, 0 ,1) = SPI1_SS0 function is selected to the pin PC.8.
 * |        |          |(1, 1, 1) = MCLK(EBI) function is selected to the pin PC.8.     
 * |[9]     |GPC_MFP9  |PC.9 Pin Function Selection
 * |        |          |Bit GPC_MFP[9] determines the PC.9 function.
 * |        |          |0 = GPIO function is selected to the pin PC.9.
 * |        |          |1 = SPI1_CLK function is selected to the pin PC.9.          
 * |[10]    |GPC_MFP10 |PC.10 Pin Function Selection
 * |        |          |Bit GPC_MFP[10] determines the PC.10 function.
 * |        |          |0 = GPIO function is selected to the pin PC.10.
 * |        |          |1 = SPI1_MISO0 function is selected to the pin PC.10.    
 * |[11]    |GPC_MFP11 |PC.11 Pin Function Selection
 * |        |          |Bit GPC_MFP[11] determines the PC.11 function.
 * |        |          |0 = GPIO function is selected to the pin PC.11.
 * |        |          |1 = SPI1_MOSI0 function is selected to the pin PC.11.        
 * |[14]    |GPC_MFP14 |PC.14 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPC_MFP[14] determine the PC.14 function.
 * |        |          |(EBI_EN, GPC_MFP14) value and function mapping is as following list
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = ADC10 function is selected.
 * |        |          |(1, 1) = AD2 function is selected.       
 * |[15]    |GPC_MFP15 |PC.15 Pin Function Selection
 * |        |          |Bits EBI_EN (ALT_MFP[11]) and GPC_MFP[15] determine the PC.15 function.
 * |        |          |(EBI_EN, GPC_MFP15) value and function mapping is as following list
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = ADC9 function is selected.
 * |        |          |(1, 1) = AD3 function is selected.        
 * |[31:16] |GPC_TYPEn |Trigger Function Selection
 * |        |          |0 = GPIOC[15:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = GPIOC[15:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::GPE_MFP
 * Offset: 0x40  GPIOE Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5]     |GPE_MFP5  |PE.5 Pin Function Selection
 * |        |          |Bits PE5_T1EX (ALT_MFP[25]), PE5_TM1 (ALT_MFP2[3]) and GPE_MFP5 determine the PE.5 function.
 * |        |          |(PE5_T1EX, GPE_MFP5) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM5 function is selected.
 * |        |          |(1, 0, 1) = TM1_EXT function is selected.
 * |        |          |(0, 1, 1) = TM1 function is selected.     
 * |[31:16] |GPE_TYPEn |Trigger Function Selection
 * |        |          |0 = GPIOD[15:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = GPIOD[15:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::GPF_MFP
 * Offset: 0x44  GPIOF Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GPF_MFP0  |PF.0 Pin Function Selection (Read Only)
 * |        |          |Bit GPF_MFP[0] determines the PF.0 function
 * |        |          |0 = GPIO function is selected to the pin PF.0.
 * |        |          |1 = XT1_OUT function is selected to the pin PF.0.
 * |        |          |Note: This bit is read only and is decided by user configuration CGPFMFP (CONFIG0[27]).     
 * |[1]     |GPF_MFP1  |PF.1 Pin Function Selection (Read Only)
 * |        |          |Bit GPF_MFP[1] determines the PF.1 function.
 * |        |          |0 = GPIO function is selected to the pin PF.1.
 * |        |          |1 = XT1_IN function is selected to the pin PF.1.
 * |        |          |Note: This bit is read only and is decided by user configuration CGPFMFP (CONFIG0[27]).     
 * |[19:16] |GPF_TYPEn |Trigger Function Selection
 * |        |          |0 = GPIOF[3:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = GPIOF[3:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::ALT_MFP
 * Offset: 0x50  Alternative Multiple Function Pin Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PB10_S01  |PB.10 Pin Alternative Function Selection
 * |        |          |Bits PB10_S01 (ALT_MFP[0]) and GPB_MFP[10] determine the PB.10 function.
 * |        |          |(PB10_S01, GPB_MFP10) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM2 function is selected.
 * |        |          |(1, 1) = UART2_RXD function is selected.  
 * |[1]     |PB9_S11   |PB.9 Pin Alternative Function Selection
 * |        |          |Bits PB9_S11 (ALT_MFP[1]) and GPB_MFP[9] determine the PB.9 function.
 * |        |          |(PB9_S11, GPB_MFP9) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM1 function is selected.
 * |        |          |(1, 1) = UART2_TXD function is selected.                        
 * |[4]     |PB11_PWM4 |PB.11 Pin Alternative Function Selection     
 * |        |          |Bits PB11_PWM4 (ALT_MFP[4]) and GPB_MFP[11] determine the PB.11 function.
 * |        |          |(PB11_PWM4, GPB_MFP11) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM3 function is selected.     
 * |        |          |(1, 1) = PWM4 function is selected.      
 * |[11]    |EBI_EN    |EBI Pin Function Selection
 * |        |          |EBI_EN is use to switch GPIO function to EBI function (AD[15:0], ALE, RE, WE, CS, MCLK), it need
 * |        |          |additional registers EBI_EN[7:0] and EBI_MCLK_EN for some GPIO to switch to EBI function(AD[15:8], MCLK).
 * |[12]    |EBI_MCLK_EN|PC.8 Pin Alternative Function Selection
 * |        |          |Bits EBI_MCLK_EN (ALT_MFP[12]), EBI_EN (ALT_MFP[11]), GPC_MFP[8] determine the PC.8 function.
 * |        |          |(EBI_MCLK_EN, EBI_EN, GPC_MFP8) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0 ,1) = SPI1_SS0 function is selected.
 * |        |          |(1, 1, 1) = MCLK(EBI) function is selected.
 * |[13]    |EBI_nWRL_EN|PB.2 Pin Alternative Function Selection
 * |        |          |Bits EBI_nWRL_EN (ALT_MFP[13]), EBI_EN (ALT_MFP[11]), PB2_TM2 (ALT_MFP2[4]), PB2_T2EX (ALT_MFP[26]) and GPB_MFP[2] determine the PB.2 function.
 * |        |          |(EBI_nWRL_EN, EBI_EN, PB2_TM2, PB2_T2EX, GPB_MFP2) value and function mapping is as following list.
 * |        |          |(0, 0, 0 , 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nRTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM2_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM2 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRL(EBI) function is selected.
 * |[14]    |EBI_nWRH_EN|PB.3 Pin Alternative Function Selection
 * |        |          |Bits EBI_nWRH_EN (ALT_MFP[14]), EBI_EN (ALT_MFP[11]), PB3_TM3 (ALT_MFP2[5]), PB3_T3EX (ALT_MFP[27]) and GPB_MFP[3] determine the PB.3 function.
 * |        |          |(EBI_nWRH_EN, EBI_EN, PB3_TM3, PB3_T3EX, GPB_MFP3) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nCTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM3_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM3 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRH(EBI) function is selected.    
 * |[16]    |EBI_HB_EN[0]|PA.5 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[0] (ALT_MFP[16]), EBI_EN (ALT_MFP[11]) and GPA_MFP[5] determine the PA.5 function.
 * |        |          |(EBI_HB_EN[0], EBI_EN, GPA_MFP5) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC5 function is selected.
 * |        |          |(1, 1, 1) = AD8 function is selected. 
 * |[17]    |EBI_HB_EN[1]|PA.4 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[1] (ALT_MFP[17]), EBI_EN (ALT_MFP[11]) and GPA_MFP[4] determine the PA.4 function.
 * |        |          |(EBI_HB_EN[1], EBI_EN, GPA_MFP4) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC4 function is selected.
 * |        |          |(1, 1, 1) = AD9 function is selected.      
 * |[18]    |EBI_HB_EN[2]|PA.3 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[2] (ALT_MFP[18]), EBI_EN (ALT_MFP[11]) and GPA_MFP[3] determine the PA.3 function.
 * |        |          |(EBI_HB_EN[2], EBI_EN, GPA_MFP3) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC3 function is selected.
 * |        |          |(1, 1, 1) = AD10 function is selected.
 * |[19]    |EBI_HB_EN[3]|PA.2 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[3] (ALT_MFP[19]), EBI_EN (ALT_MFP[11]) and GPA_MFP[2] determine the PA.2 function.
 * |        |          |(EBI_HB_EN[3], EBI_EN, GPA_MFP2) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC2 function is selected.
 * |        |          |(1, 1, 1) = AD11 function is selected.
 * |[20]    |EBI_HB_EN[4]|PA.1 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[4] (ALT_MFP[20]), EBI_EN (ALT_MFP[11]) and GPA_MFP[1] determine the PA.1 function.
 * |        |          |(EBI_HB_EN[4], EBI_EN, GPA_MFP1) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = ADC1 function is selected.
 * |        |          |(1, 1, 1) = AD12 function is selected.
 * |[21]    |EBI_HB_EN[5]|PA.12 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[5] (ALT_MFP[21]), EBI_EN (ALT_MFP[11]) and GPA_MFP[12] determine the PA.12 function.
 * |        |          |(EBI_HB_EN[5], EBI_EN, GPA_MFP12) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM0 function is selected.
 * |        |          |(1, 1, 1) = AD13 function is selected. 
 * |[22]    |EBI_HB_EN[6]|PA.13 Pin Alternative Function Selection
 * |        |          |Bits EBI_HB_EN[6] (ALT_MFP[22]), EBI_EN (ALT_MFP[11]) and GPA_MFP[13] determine the PA.13 function.
 * |        |          |(EBI_HB_EN[6], EBI_EN, GPA_MFP13) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM1 function is selected.
 * |        |          |(1, 1, 1) = AD14 function is selected.
 * |[23]    |EBI_HB_EN[7]|PA.14 Pin Alternative Function Selection     
 * |        |          |Bits EBI_HB_EN[7] (ALT_MFP[23]), EBI_EN (ALT_MFP[11]) and GPA_MFP[14] determine the PA.14 function.
 * |        |          |(EBI_HB_EN[7], EBI_EN, GPA_MFP14) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM2 function is selected.
 * |        |          |(1, 1, 1) = AD15 function is selected..        
 * |[24]    |PB15_T0EX |PB.15 Pin Alternative Function Selection  
 * |        |          |Bits PB14_15_EBI (ALT_MFP2[1]), PB15_T0EX (ALT_MFP[24]), PB15_TM0 (ALT_MFP2[2]) and GPB_MFP[15] determine the PB.15 function.
 * |        |          |(PB14_15_EBI, PB15_T0EX, PB15_TM0, GPB_MFP15) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 1) = INT1 function is selected.
 * |        |          |(0, 0, 1, 1) = TM0 function is selected.
 * |        |          |(0, 1, 0, 0) = ADC11 function is selected.
 * |        |          |(0, 1, 0, 1) = TM0_EXT function is selected.
 * |        |          |(1, 0, 0, 1) = AD6 function is selected.
 * |[25]    |PE5_T1EX  |PE.5 Pin Alternative Function Selection
 * |        |          |Bits PE5_T1EX (ALT_MFP[25]), PE5_TM1 (ALT_MFP2[3]) and GPE_MFP5 determine the PE.5 function.
 * |        |          |(PE5_T1EX, GPE_MFP5) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM5 function is selected.
 * |        |          |(1, 0, 1) = TM1_EXT function is selected.
 * |        |          |(0, 1, 1) = TM1 function is selected.        
 * |[26]    |PB2_T2EX  |PB.2 Pin Alternative Function Selection
 * |        |          |Bits EBI_nWRL_EN (ALT_MFP[13]), EBI_EN (ALT_MFP[11]), PB2_TM2 (ALT_MFP2[4]), PB2_T2EX (ALT_MFP[26]) and GPB_MFP[2] determine the PB.2 function.
 * |        |          |(EBI_nWRL_EN, EBI_EN, PB2_TM2, PB2_T2EX, GPB_MFP2) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nRTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM2_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM2 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRL(EBI) function is selected.      
 * |[27]    |PB3_T3EX  |PB.3 Pin Alternative Function Selection
 * |        |          |Bits EBI_nWRH_EN (ALT_MFP[14]), EBI_EN (ALT_MFP[11]), PB3_TM3 (ALT_MFP2[5]), PB3_T3EX (ALT_MFP[27]) and GPB_MFP[3] determine the PB.3 function.
 * |        |          |(EBI_nWRH_EN, EBI_EN, PB3_TM3, PB3_T3EX, GPB_MFP3) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nCTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM3_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM3 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRH(EBI) function is selected.                  
 * |[29]    |PB8_CLKO  |PB.8 Pin Alternative Function Selection
 * |        |          |Bits PB8_CLKO (ALT_MFP[29]) and GPB_MFP[8] determine the PB.8 function.
 * |        |          |(PB8_CLKO, GPB_MFP8) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = TM0 function is selected to the pin PB.8.
 * |        |          |(1, 0) = STADC function is selected to the pin PB.8.
 * |        |          |(1, 1) = CLKO function is selected to the pin PB.8.                      
 * @var GCR_T::ALT_MFP2
 * Offset: 0x5C  Alternative Multiple Function Pin Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |   
 * |[1]     |PB14_15_EBI|PB.14 and PB.15 Pin Alternative Function Selection  
 * |        |          |Bits PB14_15_EBI (ALT_MFP2[1]) and GPB_MFP[14] determine the PB.14 function.
 * |        |          |(PB14_15_EBI, GPB_MFP14) value and function mapping is as following list.
 * |        |          |(0, 0) = GPIO function is selected.
 * |        |          |(0, 1) = INT0 function is selected.
 * |        |          |(1, 1) = AD0 function is selected.
 * |        |          |Bits PB14_15_EBI (ALT_MFP2[1]), PB15_T0EX (ALT_MFP[24]), PB15_TM0 (ALT_MFP2[2]) and GPB_MFP[15] determine the PB.15 function.
 * |        |          |(PB14_15_EBI, PB15_T0EX, PB15_TM0, GPB_MFP15) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 1) = INT1 function is selected.
 * |        |          |(0, 0, 1, 1) = TM0 function is selected.
 * |        |          |(0, 1, 0, 0) = ADC11 function is selected.
 * |        |          |(0, 1, 0, 1) = TM0_EXT function is selected.
 * |        |          |(1, 0, 0, 1) = AD6 function is selected.  
 * |[2]     |PB15_TM0  |PB.15 Pin Alternative Function Selection 
 * |        |          |Bits PB14_15_EBI (ALT_MFP2[1]), PB15_T0EX (ALT_MFP[24]), PB15_TM0 (ALT_MFP2[2]) and GPB_MFP[15] determine the PB.15 function.
 * |        |          |(PB14_15_EBI, PB15_T0EX, PB15_TM0, GPB_MFP15) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 1) = INT1 function is selected.
 * |        |          |(0, 0, 1, 1) = TM0 function is selected.
 * |        |          |(0, 1, 0, 0) = ADC11 function is selected.
 * |        |          |(0, 1, 0, 1) = TM0_EXT function is selected.
 * |        |          |(1, 0, 0, 1) = AD6 function is selected. 
 * |[3]     |PE5_TM1   |PE.5 Pin Alternative Function Selection      
 * |        |          |Bits PE5_T1EX (ALT_MFP[25]), PE5_TM1 (ALT_MFP2[3]) and GPE_MFP5 determine the PE.5 function.
 * |        |          |(PE5_T1EX, GPE_MFP5) value and function mapping is as following list.
 * |        |          |(0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 1) = PWM5 function is selected.
 * |        |          |(1, 0, 1) = TM1_EXT function is selected.
 * |        |          |(0, 1, 1) = TM1 function is selected.         
 * |[4]     |PB2_TM2   |PB.2 Pin Alternative Function Selection 
 * |        |          |Bits EBI_nWRL_EN (ALT_MFP[13]), EBI_EN (ALT_MFP[11]), PB2_TM2 (ALT_MFP2[4]), PB2_T2EX (ALT_MFP[26]) and GPB_MFP[2] determine the PB.2 function.
 * |        |          |(EBI_nWRL_EN, EBI_EN, PB2_TM2, PB2_T2EX, GPB_MFP2) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nRTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM2_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM2 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRL(EBI) function is selected. 
 * |[5]     |PB3_TM3   |PB.2 Pin Alternative Function Selection 
 * |        |          |Bits EBI_nWRH_EN (ALT_MFP[14]), EBI_EN (ALT_MFP[11]), PB3_TM3 (ALT_MFP2[5]), PB3_T3EX (ALT_MFP[27]) and GPB_MFP[3] determine the PB.3 function.
 * |        |          |(EBI_nWRH_EN, EBI_EN, PB3_TM3, PB3_T3EX, GPB_MFP3) value and function mapping is as following list.
 * |        |          |(0, 0, 0, 0, 0) = GPIO function is selected.
 * |        |          |(0, 0, 0, 0, 1) = UART0_nCTS function is selected.
 * |        |          |(0, 0, 0, 1, 1) = TM3_EXT function is selected.
 * |        |          |(0, 0, 1, 0, 1) = TM3 function is selected.
 * |        |          |(1, 1, 0, 0, 1) = nWRH(EBI) function is selected.    
 * @var GCR_T::IRCTCTL
 * Offset: 0x80  IRC Trim Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |TRIM_SEL  |Trim Frequency Selection
 * |        |          |This field indicates the target frequency of internal 22.1184 MHz high speed oscillator will trim to precise 22.1184MHz or 24MHz automatically.
 * |        |          |If no any target frequency is selected (TRIM_SEL is 00), the HIRC auto trim function isdisabled.
 * |        |          |During auto trim operation, if clock error detected because of CLKERR_STOP_EN is set to 1 or trim retry limitation counts reached, this field will be cleared to 00 automatically.
 * |        |          |00 = HIRC auto trim function Disabled.
 * |        |          |01 = HIRC auto trim function Enabled and HIRC trimmed to 22.1184 MHz.
 * |        |          |10 = HIRC auto trim function Enabled and HIRC trimmed to 24 MHz.
 * |        |          |11 = Reserved.
 * |[5:4]   |TRIM_LOOP |Trim Calculation Loop
 * |        |          |This field defines that trim value calculation is based on how many 32.768 kHz clocks in.
 * |        |          |For example, if TRIM_LOOP is set as 00, auto trim circuit will calculate trim value based on the
 * |        |          |average frequency difference in 4 32.768 kHz clock.
 * |        |          |00 = Trim value calculation is based on average difference in 4 clocks.
 * |        |          |01 = Trim value calculation is based on average difference in 8 clocks.
 * |        |          |10 = Trim value calculation is based on average difference in 16 clocks.
 * |        |          |11 = Trim value calculation is based on average difference in 32 clocks.
 * |[7:6]   |TRIM_RETRY_CNT|Trim Value Update Limitation Count
 * |        |          |The field defines that how many times of HIRC trim value is updated by auto trim circuit before the HIRC frequency locked.
 * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
 * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC still doesn't lock, the auto trim operation will be disabled and TRIM_SEL will be cleared to 00.
 * |        |          |00 = Trim retry count limitation is 64.
 * |        |          |01 = Trim retry count limitation is 128.
 * |        |          |10 = Trim retry count limitation is 256.
 * |        |          |11 = Trim retry count limitation is 512.
 * |[8]     |CLKERR_STOP_EN|Clock Error Stop Enable Bit
 * |        |          |0 = The trim operation is kept going if clock is inaccuracy.
 * |        |          |1 = The trim operation is stopped if clock is inaccuracy.
 * @var GCR_T::IRCTIEN
 * Offset: 0x84  IRC Trim Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |TRIM_FAIL_IEN|Trim Failure Interrupt Enable
 * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by TRIM_SEL (IRCTCTL[1:0]).
 * |        |          |If this bit is high and TRIM_FAIL_INT (IRCTSTS[1]) is set during auto trim operation.
 * |        |          |An interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
 * |        |          |0 = TRIM_FAIL_INT (IRCTSTS[1]) status to trigger an interrupt to CPU Disabled.
 * |        |          |1 = TRIM_FAIL_INT (IRCTSTS[1]) status to trigger an interrupt to CPU Enabled.
 * |[2]     |CLKERR_IEN|Clock Error Interrupt Enable
 * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
 * |        |          |If this bit is set to1, and CLKERR_INT (IRCTSTS[2]) is set during auto trim operation.
 * |        |          |An interrupt will be triggered to notify the clock frequency is inaccuracy.
 * |        |          |0 = CLKERR_INT (IRCTSTS[2]) status to trigger an interrupt to CPU Disabled.
 * |        |          |1 = CLKERR_INT (IRCTSTS[2]) status to trigger an interrupt to CPU Enabled.
 * @var GCR_T::IRCTSTS
 * Offset: 0x88  IRC Trim Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FREQ_LOCK |HIRC Frequency Lock Status
 * |        |          |This bit indicates the internal 22.1184 MHz high speed oscillator frequency is locked.
 * |        |          |This is a status bit and doesn't trigger any interrupt.
 * |[1]     |TRIM_FAIL_INT|Trim Failure Interrupt Status
 * |        |          |This bit indicates that internal 22.1184 MHz high speed oscillator trim value update limitation
 * |        |          |count reached and the internal 22.1184 MHz high speed oscillator clock frequency still doesn't be locked.
 * |        |          |Once this bit is set, the auto trim operation stopped and TRIM_SEL (IRCTCTL [1:0]) will be cleared to 00 by hardware automatically.
 * |        |          |If this bit is set and TRIM_FAIL_IEN (IRCTIEN[1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
 * |        |          |Write 1 to clear this to 0.
 * |        |          |0 = Trim value update limitation count did not reach.
 * |        |          |1 = Trim value update limitation count reached and internal 22.1184 MHz high speed oscillator
 * |        |          |frequency was still not locked.
 * |[2]     |CLKERR_INT|Clock Error Interrupt Status
 * |        |          |When the frequency of external 32.768 kHz low speed crystal or internal 22.1184 MHz high speed oscillator is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
 * |        |          |Once this bit is set to 1, the auto trim operation stopped and TRIM_SEL (IRCTCTL [1:0]) will be cleared to 00 by hardware automatically if CLKERR_STOP_EN (IRCTCTL [8]) is set to 1.
 * |        |          |If this bit is set and CLKERR_IEN (IRCTIEN [2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy.
 * |        |          |Write 1 to clear this to 0.
 * |        |          |0 = Clock frequency is accurate.
 * |        |          |1 = Clock frequency is inaccurate.
 * @var GCR_T::HIRCTCTL
 * Offset: 0x90  HIRC Trim Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |FREQSEL   |Trim Frequency Selection
 * |        |          |This field indicates the target frequency of 48 MHz internal high speed RC oscillator (HIRC) auto trim.
 * |        |          |During auto trim operation, if clock error detected with CESTOPEN is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
 * |        |          |00 = Disable HIRC auto trim function.
 * |        |          |01 = Enable HIRC auto trim function and trim HIRC to 48 MHz.
 * |        |          |10 = Reserved.
 * |        |          |11 = Reserved.
 * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection		
 * |        |          |This field defines that trim value calculation is based on how many reference clocks.
 * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
 * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
 * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
 * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
 * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
 * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count	
 * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.
 * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
 * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC still does not lock, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
 * |        |          |00 = Trim retry count limitation is 64 loops.
 * |        |          |01 = Trim retry count limitation is 128 loops.
 * |        |          |10 = Trim retry count limitation is 256 loops.
 * |        |          |11 = Trim retry count limitation is 512 loops.
 * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit	
 * |        |          |0 = The trim operation is keep going if clock is inaccuracy.
 * |        |          |1 = The trim operation is stopped if clock is inaccuracy.
 * |[9]     |BOUNDEN   |Boundary Enable	 
 * |        |          |0 = Boundary function is disable.
 * |        |          |1 = Boundary function is enable.
 * |[20:16] |BOUNDARY  |Boundary Selection		
 * |        |          |Fill the boundary range from 1 to 31, 0 is reserved.
 * |        |          |Note: This field is effective only when the BOUNDEN(SYS_HIRCTCTL[9]) is enable.
 * @var GCR_T::HIRCTIEN
 * Offset: 0x94  HIRC Trim Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |TFALIEN_IEN|Trim Failure Interrupt Enable Bit
 * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by FREQSEL(SYS_HIRCTCTL[1:0]).
 * |        |          |If this bit is high and TFAILIF(SYS_HIRCTSTS[1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
 * |        |          |0 = Disable TFAILIF(SYS_HIRCTSTS[1]) status to trigger an interrupt to CPU.
 * |        |          |1 = Enable TFAILIF(SYS_HIRCTSTS[1]) status to trigger an interrupt to CPU.
 * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
 * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
 * |        |          |If this bit is set to 1, and CLKERRIF(SYS_HIRCTSTS[2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
 * |        |          |0 = Disable CLKERRIF(SYS_HIRCTSTS[2]) status to trigger an interrupt to CPU.
 * |        |          |1 = Enable CLKERRIF(SYS_HIRCTSTS[2]) status to trigger an interrupt to CPU.
 * @var GCR_T::HIRCTSTS
 * Offset: 0x98  HIRC Trim Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FREQLOCK  |HIRC Frequency Lock Status 
 * |        |          |This bit indicates the HIRC frequency is locked.
 * |        |          |This is a status bit and does not trigger any interrupt
 * |        |          |Write 1 to clear this to 0. This bit will be set automatically, if the frequecy is lock and the RC_TRIM is enabled. 
 * |        |          |0 = The internal high-speed oscillator frequency does not lock at 48 MHz yet.
 * |        |          |1 = The internal high-speed oscillator frequency locked at 48 MHz.
 * |        |          |Note: Reset by power-on reset.
 * |[1]     |TFAILIF   |Trim Failure Interrupt Status
 * |        |          |This bit indicates that HIRC trim value update limitation count reached and the HIRC clock frequency still does not be locked. 
 * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_HIRCTCTL[1:0]) will be cleared to 00 by hardware automatically.
 * |        |          |If this bit is set and TFAILIEN(SYS_HIRCTIEN[1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached. Write 1 to clear this to 0.
 * |        |          |0 = Trim value update limitation count does not reach.
 * |        |          |1 = Trim value update limitation count reached and HIRC frequency still not locked.
 * |        |          |Note: Reset by power-on reset.
 * |[2]     |CLKERIF   |Clock Error Interrupt Status
 * |        |          |When the reference clock or 48MHz internal high speed RC oscillator (HIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
 * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_HIRCTCTL[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_HIRCTCTL[8]) is set to 1.
 * |        |          |If this bit is set and CLKEIEN(SYS_HIRCTIEN[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy. Write 1 to clear this to 0.
 * |        |          |0 = Clock frequency is accuracy.
 * |        |          |1 = Clock frequency is inaccuracy.
 * |        |          |Note: Reset by power-on reset.
 * |[3]     |OVBDIF    |Over Boundary Status
 * |        |          |When the over boundary function is set, if there occurs the over boundary condition, this flag will be set.
 * |        |          |0 = Over boundary coundition did not occur.
 * |        |          |1 = Over boundary coundition occurred.
 * |        |          |Note: Write 1 to clear this flag.
 * @var GCR_T::REGWRPROT
 * Offset: 0x100  Register Write Protection Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |REGPROTDIS|Register Write-Protection Disable Index (Read Only)
 * |        |          |0 = Write-protection is enabled for writing protected registers. Any write to the protected register is ignored.
 * |        |          |1 = Write-protection is disabled for writing protected registers.
 * |        |          |Note: The bits which are write-protected will be noted as" (Write Protect)" beside the description.
 * |[7:0]   |REGWRPROT |Register Write-Protection Code (Write Only)
 * |        |          |Some registers have write-protection function.
 * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
 * |        |          |After this sequence is completed, the REGPROTDIS bit will be set to 1 and write-protection registers can be normal write.
 */

    __I  uint32_t PDID;          /* Offset: 0x00  Part Device Identification Number Register                         */
    __IO uint32_t RSTSRC;        /* Offset: 0x04  System Reset Source Register                                       */
    __IO uint32_t IPRSTC1;       /* Offset: 0x08  IP Reset Control Register 1                                        */
    __IO uint32_t IPRSTC2;       /* Offset: 0x0C  IP Reset Control Register 2                                        */
    __IO uint32_t IPRSTC3;       /* Offset: 0x10  IP Reset Control Register 3                                        */
    __I  uint32_t RESERVE0;     
    __IO uint32_t BODCR;         /* Offset: 0x18  Brown-out Detector Control Register                                */
    __IO uint32_t TEMPCR;        /* Offset: 0x1C  Temperature Sensor Control Register                                */
    __I  uint32_t RESERVE1;     
    __IO uint32_t PORCR;         /* Offset: 0x24  Power-on-Reset Controller Register                                 */
    __I  uint32_t RESERVE2[2];  
    __IO uint32_t GPA_MFP;       /* Offset: 0x30  GPIOA Multiple Function and Input Type Control Register            */
    __IO uint32_t GPB_MFP;       /* Offset: 0x34  GPIOB Multiple Function and Input Type Control Register            */
    __IO uint32_t GPC_MFP;       /* Offset: 0x38  GPIOC Multiple Function and Input Type Control Register            */
    __I  uint32_t RESERVE3;
    __IO uint32_t GPE_MFP;       /* Offset: 0x40  GPIOE Multiple Function and Input Type Control Register            */
    __IO uint32_t GPF_MFP;       /* Offset: 0x44  GPIOF Multiple Function and Input Type Control Register            */
    __I  uint32_t RESERVE4[2];  
    __IO uint32_t ALT_MFP;       /* Offset: 0x50  Alternative Multiple Function Pin Control Register                 */
    __I  uint32_t RESERVE5[2];     
    __IO uint32_t ALT_MFP2;      /* Offset: 0x5C  Alternative Multiple Function Pin Control Register 2               */
    __I  uint32_t RESERVE6[8];  
    __IO uint32_t IRCTCTL;       /* Offset: 0x80  IRC Trim Control Register                                          */
    __IO uint32_t IRCTIEN;       /* Offset: 0x84  IRC Trim Interrupt Enable Register                                 */
    __IO uint32_t IRCTSTS;       /* Offset: 0x88  IRC Trim Interrupt Status Register                                 */
    __I  uint32_t RESERVE7[1];
    __IO uint32_t HIRCTCTL;      /* Offset: 0x90  HIRC Trim Control Register                                         */
    __IO uint32_t HIRCTIEN;      /* Offset: 0x94  HIRC Trim Interrupt Enable Register                                */
    __IO uint32_t HIRCTSTS;      /* Offset: 0x98  HIRC Trim Interrupt Status Register                                */
    __I  uint32_t RESERVE8[25];
    __IO uint32_t REGWRPROT;     /* Offset: 0x100  Register Write Protection Register                                */

} GCR_T;


/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */


/* GCR RSTSRC Bit Field Definitions */
#define SYS_RSTSRC_RSTS_CPU_Pos                 7                                   /*!< GCR_T::RSTSRC: RSTS_CPU Position */
#define SYS_RSTSRC_RSTS_CPU_Msk                 (1ul << SYS_RSTSRC_RSTS_CPU_Pos)    /*!< GCR_T::RSTSRC: RSTS_CPU Mask */

#define SYS_RSTSRC_RSTS_SYS_Pos                 5                                   /*!< GCR_T::RSTSRC: RSTS_SYS Position */
#define SYS_RSTSRC_RSTS_SYS_Msk                 (1ul << SYS_RSTSRC_RSTS_SYS_Pos)    /*!< GCR_T::RSTSRC: RSTS_SYS Mask */

#define SYS_RSTSRC_RSTS_BOD_Pos                 4                                   /*!< GCR_T::RSTSRC: RSTS_BOD Position */
#define SYS_RSTSRC_RSTS_BOD_Msk                 (1ul << SYS_RSTSRC_RSTS_BOD_Pos)    /*!< GCR_T::RSTSRC: RSTS_BOD Mask */

#define SYS_RSTSRC_RSTS_LVR_Pos                 3                                   /*!< GCR_T::RSTSRC: RSTS_LVR Position */
#define SYS_RSTSRC_RSTS_LVR_Msk                 (1ul << SYS_RSTSRC_RSTS_LVR_Pos)    /*!< GCR_T::RSTSRC: RSTS_LVR Mask */

#define SYS_RSTSRC_RSTS_WDT_Pos                 2                                   /*!< GCR_T::RSTSRC: RSTS_WDT Position */
#define SYS_RSTSRC_RSTS_WDT_Msk                 (1ul << SYS_RSTSRC_RSTS_WDT_Pos)    /*!< GCR_T::RSTSRC: RSTS_WDT Mask */

#define SYS_RSTSRC_RSTS_RESET_Pos               1                                   /*!< GCR_T::RSTSRC: RSTS_RESET Position */
#define SYS_RSTSRC_RSTS_RESET_Msk               (1ul << SYS_RSTSRC_RSTS_RESET_Pos)  /*!< GCR_T::RSTSRC: RSTS_RESET Mask */

#define SYS_RSTSRC_RSTS_POR_Pos                 0                                   /*!< GCR_T::RSTSRC: RSTS_POR Position */
#define SYS_RSTSRC_RSTS_POR_Msk                 (1ul << SYS_RSTSRC_RSTS_POR_Pos)    /*!< GCR_T::RSTSRC: RSTS_POR Mask */

/* GCR IPRSTC1 Bit Field Definitions */
#define SYS_IPRSTC1_EBI_RST_Pos                 3                                   /*!< GCR_T::IPRSTC1: EBI_RST Position */
#define SYS_IPRSTC1_EBI_RST_Msk                 (1ul << SYS_IPRSTC1_EBI_RST_Pos)    /*!< GCR_T::IPRSTC1: EBI_RST Mask */

#define SYS_IPRSTC1_PDMA_RST_Pos                2                                   /*!< GCR_T::IPRSTC1: PDMA_RST Position */
#define SYS_IPRSTC1_PDMA_RST_Msk                (1ul << SYS_IPRSTC1_PDMA_RST_Pos)   /*!< GCR_T::IPRSTC1: PDMA_RST Mask */

#define SYS_IPRSTC1_CPU_RST_Pos                 1                                   /*!< GCR_T::IPRSTC1: CPU_RST Position */
#define SYS_IPRSTC1_CPU_RST_Msk                 (1ul << SYS_IPRSTC1_CPU_RST_Pos)    /*!< GCR_T::IPRSTC1: CPU_RST Mask */

#define SYS_IPRSTC1_CHIP_RST_Pos                0                                   /*!< GCR_T::IPRSTC1: CHIP_RST Position */
#define SYS_IPRSTC1_CHIP_RST_Msk                (1ul << SYS_IPRSTC1_CHIP_RST_Pos)   /*!< GCR_T::IPRSTC1: CHIP_RST Mask */

/* GCR IPRSTC2 Bit Field Definitions */
#define SYS_IPRSTC2_ADC_RST_Pos                 28                                  /*!< GCR_T::IPRSTC2: ADC_RST Position */
#define SYS_IPRSTC2_ADC_RST_Msk                 (1ul << SYS_IPRSTC2_ADC_RST_Pos)    /*!< GCR_T::IPRSTC2: ADC_RST Mask */

#define SYS_IPRSTC2_USBD_RST_Pos                27                                  /*!< GCR_T::IPRSTC2: USBD_RST Position */
#define SYS_IPRSTC2_USBD_RST_Msk                (1ul << SYS_IPRSTC2_USBD_RST_Pos)   /*!< GCR_T::IPRSTC2: USBD_RST Mask */

#define SYS_IPRSTC2_PWM45_RST_Pos               21                                  /*!< GCR_T::IPRSTC2: PWM45_RST Position */
#define SYS_IPRSTC2_PWM45_RST_Msk               (1ul << SYS_IPRSTC2_PWM45_RST_Pos)  /*!< GCR_T::IPRSTC2: PWM45_RST Mask */

#define SYS_IPRSTC2_PWM03_RST_Pos               20                                  /*!< GCR_T::IPRSTC2: PWM03_RST Position */
#define SYS_IPRSTC2_PWM03_RST_Msk               (1ul << SYS_IPRSTC2_PWM03_RST_Pos)  /*!< GCR_T::IPRSTC2: PWM03_RST Mask */

#define SYS_IPRSTC2_UART2_RST_Pos               18                                  /*!< GCR_T::IPRSTC2: UART2_RST Position */
#define SYS_IPRSTC2_UART2_RST_Msk               (1ul << SYS_IPRSTC2_UART2_RST_Pos)  /*!< GCR_T::IPRSTC2: UART2_RST Mask */

#define SYS_IPRSTC2_UART1_RST_Pos               17                                  /*!< GCR_T::IPRSTC2: UART1_RST Position */
#define SYS_IPRSTC2_UART1_RST_Msk               (1ul << SYS_IPRSTC2_UART1_RST_Pos)  /*!< GCR_T::IPRSTC2: UART1_RST Mask */

#define SYS_IPRSTC2_UART0_RST_Pos               16                                  /*!< GCR_T::IPRSTC2: UART0_RST Position */
#define SYS_IPRSTC2_UART0_RST_Msk               (1ul << SYS_IPRSTC2_UART0_RST_Pos)  /*!< GCR_T::IPRSTC2: UART0_RST Mask */

#define SYS_IPRSTC2_SPI1_RST_Pos                13                                  /*!< GCR_T::IPRSTC2: SPI1_RST Position */
#define SYS_IPRSTC2_SPI1_RST_Msk                (1ul << SYS_IPRSTC2_SPI1_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI1_RST Mask */

#define SYS_IPRSTC2_SPI0_RST_Pos                12                                  /*!< GCR_T::IPRSTC2: SPI0_RST Position */
#define SYS_IPRSTC2_SPI0_RST_Msk                (1ul << SYS_IPRSTC2_SPI0_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI0_RST Mask */

#define SYS_IPRSTC2_I2C1_RST_Pos                9                                   /*!< GCR_T::IPRSTC2: I2C1_RST Position */
#define SYS_IPRSTC2_I2C1_RST_Msk                (1ul << SYS_IPRSTC2_I2C1_RST_Pos)   /*!< GCR_T::IPRSTC2: I2C1_RST Mask */

#define SYS_IPRSTC2_I2C0_RST_Pos                8                                   /*!< GCR_T::IPRSTC2: I2C0_RST Position */
#define SYS_IPRSTC2_I2C0_RST_Msk                (1ul << SYS_IPRSTC2_I2C0_RST_Pos)   /*!< GCR_T::IPRSTC2: I2C0_RST Mask */

#define SYS_IPRSTC2_TMR3_RST_Pos                5                                   /*!< GCR_T::IPRSTC2: TMR3_RST Position */
#define SYS_IPRSTC2_TMR3_RST_Msk                (1ul << SYS_IPRSTC2_TMR3_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR3_RST Mask */

#define SYS_IPRSTC2_TMR2_RST_Pos                4                                   /*!< GCR_T::IPRSTC2: TMR2_RST Position */
#define SYS_IPRSTC2_TMR2_RST_Msk                (1ul << SYS_IPRSTC2_TMR2_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR2_RST Mask */

#define SYS_IPRSTC2_TMR1_RST_Pos                3                                   /*!< GCR_T::IPRSTC2: TMR1_RST Position */
#define SYS_IPRSTC2_TMR1_RST_Msk                (1ul << SYS_IPRSTC2_TMR1_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR1_RST Mask */

#define SYS_IPRSTC2_TMR0_RST_Pos                2                                   /*!< GCR_T::IPRSTC2: TMR0_RST Position */
#define SYS_IPRSTC2_TMR0_RST_Msk                (1ul << SYS_IPRSTC2_TMR0_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR0_RST Mask */

#define SYS_IPRSTC2_GPIO_RST_Pos                1                                   /*!< GCR_T::IPRSTC2: GPIO_RST Position */
#define SYS_IPRSTC2_GPIO_RST_Msk                (1ul << SYS_IPRSTC2_GPIO_RST_Pos)   /*!< GCR_T::IPRSTC2: GPIO_RST Mask */

/* GCR BODCR Bit Field Definitions */
#define SYS_BODCR_LVR_EN_Pos                    7                                   /*!< GCR_T::BODCR: LVR_EN Position */
#define SYS_BODCR_LVR_EN_Msk                    (1ul << SYS_BODCR_LVR_EN_Pos)       /*!< GCR_T::BODCR: LVR_EN Mask */

#define SYS_BODCR_BOD_OUT_Pos                   6                                   /*!< GCR_T::BODCR: BOD_OUT Position */
#define SYS_BODCR_BOD_OUT_Msk                   (1ul << SYS_BODCR_BOD_OUT_Pos)      /*!< GCR_T::BODCR: BOD_OUT Mask */

#define SYS_BODCR_BOD_LPM_Pos                   5                                   /*!< GCR_T::BODCR: BOD_LPM Position */
#define SYS_BODCR_BOD_LPM_Msk                   (1ul << SYS_BODCR_BOD_LPM_Pos)      /*!< GCR_T::BODCR: BOD_LPM Mask */

#define SYS_BODCR_BOD_INTF_Pos                  4                                   /*!< GCR_T::BODCR: BOD_INTF Position */
#define SYS_BODCR_BOD_INTF_Msk                  (1ul << SYS_BODCR_BOD_INTF_Pos)     /*!< GCR_T::BODCR: BOD_INTF Mask */

#define SYS_BODCR_BOD_RSTEN_Pos                 3                                   /*!< GCR_T::BODCR: BOD_RSTEN Position */
#define SYS_BODCR_BOD_RSTEN_Msk                 (1ul << SYS_BODCR_BOD_RSTEN_Pos)    /*!< GCR_T::BODCR: BOD_RSTEN Mask */

#define SYS_BODCR_BOD_VL_Pos                    1                                   /*!< GCR_T::BODCR: BOD_VL Position */
#define SYS_BODCR_BOD_VL_Msk                    (3ul << SYS_BODCR_BOD_VL_Pos)       /*!< GCR_T::BODCR: BOD_VL Mask */

#define SYS_BODCR_BOD_EN_Pos                    0                                   /*!< GCR_T::BODCR: BOD_EN Position */
#define SYS_BODCR_BOD_EN_Msk                    (1ul << SYS_BODCR_BOD_EN_Pos)       /*!< GCR_T::BODCR: BOD_EN Mask */

/* GCR TEMPCR Bit Field Definitions */
#define SYS_TEMPCR_VTEMP_EN_Pos                 0                                   /*!< GCR_T::TEMPCR: VTEMP_EN Position */
#define SYS_TEMPCR_VTEMP_EN_Msk                 (1ul << SYS_TEMPCR_VTEMP_EN_Pos)    /*!< GCR_T::TEMPCR: VTEMP_EN Mask */

/* GCR PORCR Bit Field Definitions */
#define SYS_PORCR_POR_DIS_CODE_Pos              0                                           /*!< GCR_T::PORCR: POR_DIS_CODE Position */
#define SYS_PORCR_POR_DIS_CODE_Msk              (0xFFFFul << SYS_PORCR_POR_DIS_CODE_Pos)    /*!< GCR_T::PORCR: POR_DIS_CODE Mask */

/* GCR GPAMFP Bit Field Definitions */
#define SYS_GPA_MFP_GPA_TYPE_Pos                 16                                         /*!< GCR_T::GPA_MFP: GPA_TYPE Position */
#define SYS_GPA_MFP_GPA_TYPE_Msk                 (0xFFFFul << SYS_GPA_MFP_GPA_TYPE_Pos)     /*!< GCR_T::GPA_MFP: GPA_TYPE Mask */

#define SYS_GPA_MFP_GPA_MFP_Pos                  0                                          /*!< GCR_T::GPA_MFP: GPA_MFP Position */
#define SYS_GPA_MFP_GPA_MFP_Msk                  (0xFFFFul << SYS_GPA_MFP_GPA_MFP_Pos)      /*!< GCR_T::GPA_MFP: GPA_MFP Mask */

/* GCR GPBMFP Bit Field Definitions */
#define SYS_GPB_MFP_GPB_TYPE_Pos                 16                                         /*!< GCR_T::GPB_MFP: GPB_TYPE Position */
#define SYS_GPB_MFP_GPB_TYPE_Msk                 (0xFFFFul << SYS_GPB_MFP_GPB_TYPE_Pos)     /*!< GCR_T::GPB_MFP: GPB_TYPE Mask */

#define SYS_GPB_MFP_GPB_MFP_Pos                  0                                          /*!< GCR_T::GPB_MFP: GPB_MFP Position */
#define SYS_GPB_MFP_GPB_MFP_Msk                  (0xFFFFul << SYS_GPB_MFP_GPB_MFP_Pos)      /*!< GCR_T::GPB_MFP: GPB_MFP Mask */

/* GCR GPCMFP Bit Field Definitions */
#define SYS_GPC_MFP_GPC_TYPE_Pos                 16                                         /*!< GCR_T::GPC_MFP: GPC_TYPE Position */
#define SYS_GPC_MFP_GPC_TYPE_Msk                 (0xFFFFul << SYS_GPC_MFP_GPC_TYPE_Pos)     /*!< GCR_T::GPC_MFP: GPC_TYPE Mask */

#define SYS_GPC_MFP_GPC_MFP_Pos                  0                                          /*!< GCR_T::GPC_MFP: GPC_MFP Position */
#define SYS_GPC_MFP_GPC_MFP_Msk                  (0xFFFFul << SYS_GPC_MFP_GPC_MFP_Pos)      /*!< GCR_T::GPC_MFP: GPC_MFP Mask */

/* GCR GPEMFP Bit Field Definitions */
#define SYS_GPE_MFP_GPE_TYPE_Pos                 16                                         /*!< GCR_T::GPE_MFP: GPE_TYPE Position */
#define SYS_GPE_MFP_GPE_TYPE_Msk                 (0xFFFFul << SYS_GPE_MFP_GPE_TYPE_Pos)     /*!< GCR_T::GPE_MFP: GPE_TYPE Mask */

#define SYS_GPE_MFP_GPE_MFP5_Pos                 5                                          /*!< GCR_T::GPE_MFP: GPE_MFP5 Position */
#define SYS_GPE_MFP_GPE_MFP5_Msk                 (1ul << SYS_GPE_MFP_GPE_MFP5_Pos)          /*!< GCR_T::GPE_MFP: GPE_MFP5 Mask */

/* GCR GPFMFP Bit Field Definitions */
#define SYS_GPF_MFP_GPF_TYPE_Pos                 16                                         /*!< GCR_T::GPF_MFP: GPF_TYPE Position */
#define SYS_GPF_MFP_GPF_TYPE_Msk                 (0xFul << SYS_GPF_MFP_GPF_TYPE_Pos)        /*!< GCR_T::GPF_MFP: GPF_TYPE Mask */

#define SYS_GPF_MFP_GPF_MFP1_Pos                 1                                          /*!< GCR_T::GPF_MFP: GPF_MFP1 Position */
#define SYS_GPF_MFP_GPF_MFP1_Msk                 (1ul << SYS_GPF_MFP_GPF_MFP1_Pos)          /*!< GCR_T::GPF_MFP: GPF_MFP1 Mask */

#define SYS_GPF_MFP_GPF_MFP0_Pos                 0                                          /*!< GCR_T::GPF_MFP: GPF_MFP0 Position */
#define SYS_GPF_MFP_GPF_MFP0_Msk                 (1ul << SYS_GPF_MFP_GPF_MFP0_Pos)          /*!< GCR_T::GPF_MFP: GPF_MFP0 Mask */

/* GCR ALTMFP Bit Field Definitions */
#define SYS_ALT_MFP_PB8_CLKO_Pos                 29                                         /*!< GCR_T::ALT_MFP: PB8_CLKO Position */
#define SYS_ALT_MFP_PB8_CLKO_Msk                 (1ul << SYS_ALT_MFP_PB8_CLKO_Pos)          /*!< GCR_T::ALT_MFP: PB8_CLKO Mask */

#define SYS_ALT_MFP_PB3_T3EX_Pos                 27                                         /*!< GCR_T::ALT_MFP: PB3_T3EX Position */
#define SYS_ALT_MFP_PB3_T3EX_Msk                 (1ul << SYS_ALT_MFP_PB3_T3EX_Pos)          /*!< GCR_T::ALT_MFP: PB3_T3EX Mask */

#define SYS_ALT_MFP_PB2_T2EX_Pos                 26                                         /*!< GCR_T::ALT_MFP: PB2_T2EX Position */
#define SYS_ALT_MFP_PB2_T2EX_Msk                 (1ul << SYS_ALT_MFP_PB2_T2EX_Pos)          /*!< GCR_T::ALT_MFP: PB3_T3EX Mask */

#define SYS_ALT_MFP_PE5_T1EX_Pos                 25                                         /*!< GCR_T::ALT_MFP: PE5_T1EX Position */
#define SYS_ALT_MFP_PE5_T1EX_Msk                 (1ul << SYS_ALT_MFP_PE5_T1EX_Pos)          /*!< GCR_T::ALT_MFP: PE5_T1EX Mask */

#define SYS_ALT_MFP_PB15_T0EX_Pos                24                                         /*!< GCR_T::ALT_MFP: PB15_T0EX Position */
#define SYS_ALT_MFP_PB15_T0EX_Msk                (1ul << SYS_ALT_MFP_PB15_T0EX_Pos)         /*!< GCR_T::ALT_MFP: PB15_T0EX Mask */

#define SYS_ALT_MFP_EBI_HB_EN_Pos                16                                         /*!< GCR_T::ALT_MFP: EBI_HB_EN Position */
#define SYS_ALT_MFP_EBI_HB_EN_Msk                (0xFFul << SYS_ALT_MFP_EBI_HB_EN_Pos)      /*!< GCR_T::ALT_MFP: EBI_HB_EN Mask */

#define SYS_ALT_MFP_EBI_nWRH_EN_Pos              14                                         /*!< GCR_T::ALT_MFP: EBI_nWRH_EN Position */
#define SYS_ALT_MFP_EBI_nWRH_EN_Msk              (1ul << SYS_ALT_MFP_EBI_nWRH_EN_Pos)       /*!< GCR_T::ALT_MFP: EBI_nWRH_EN Mask */

#define SYS_ALT_MFP_EBI_nWRL_EN_Pos              13                                         /*!< GCR_T::ALT_MFP: EBI_nWRL_EN Position */
#define SYS_ALT_MFP_EBI_nWRL_EN_Msk              (1ul << SYS_ALT_MFP_EBI_nWRL_EN_Pos)       /*!< GCR_T::ALT_MFP: EBI_nWRL_EN Mask */

#define SYS_ALT_MFP_EBI_MCLK_EN_Pos              12                                         /*!< GCR_T::ALT_MFP: EBI_MCLK_EN Position */
#define SYS_ALT_MFP_EBI_MCLK_EN_Msk              (1ul << SYS_ALT_MFP_EBI_MCLK_EN_Pos)       /*!< GCR_T::ALT_MFP: EBI_MCLK_EN Mask */

#define SYS_ALT_MFP_EBI_EN_Pos                   11                                         /*!< GCR_T::ALT_MFP: EBI_EN Position */
#define SYS_ALT_MFP_EBI_EN_Msk                   (1ul << SYS_ALT_MFP_EBI_EN_Pos)            /*!< GCR_T::ALT_MFP: EBI_EN Mask */

#define SYS_ALT_MFP_PB11_PWM4_Pos                4                                          /*!< GCR_T::ALT_MFP: PB11_PWM4 Position */
#define SYS_ALT_MFP_PB11_PWM4_Msk                (1ul << SYS_ALT_MFP_PB11_PWM4_Pos)         /*!< GCR_T::ALT_MFP: PB11_PWM4 Mask */

#define SYS_ALT_MFP_PB9_S11_Pos                  1                                          /*!< GCR_T::ALT_MFP: PB9_S11 Position */
#define SYS_ALT_MFP_PB9_S11_Msk                  (1ul << SYS_ALT_MFP_PB9_S11_Pos)           /*!< GCR_T::ALT_MFP: PB9_S11 Mask */

#define SYS_ALT_MFP_PB10_S01_Pos                 0                                          /*!< GCR_T::ALT_MFP: PB10_S01 Position */
#define SYS_ALT_MFP_PB10_S01_Msk                 (1ul << SYS_ALT_MFP_PB10_S01_Pos)          /*!< GCR_T::ALT_MFP: PB10_S01 Mask */

/* GCR ALTMFP2 Bit Field Definitions */
#define SYS_ALT_MFP2_PB3_TM3_Pos                5                                           /*!< GCR_T::ALT_MFP2: PB3_TM3 Position */
#define SYS_ALT_MFP2_PB3_TM3_Msk                (1ul << SYS_ALT_MFP2_PB3_TM3_Pos)           /*!< GCR_T::ALT_MFP2: PB3_TM3 Mask */

#define SYS_ALT_MFP2_PB2_TM2_Pos                4                                           /*!< GCR_T::ALT_MFP2: PB2_TM2 Position */
#define SYS_ALT_MFP2_PB2_TM2_Msk                (1ul << SYS_ALT_MFP2_PB2_TM2_Pos)           /*!< GCR_T::ALT_MFP2: PB2_TM2 Mask */

#define SYS_ALT_MFP2_PE5_TM1_Pos                3                                           /*!< GCR_T::ALT_MFP2: PE5_TM1 Position */
#define SYS_ALT_MFP2_PE5_TM1_Msk                (1ul << SYS_ALT_MFP2_PE5_TM1_Pos)           /*!< GCR_T::ALT_MFP2: PE5_TM1 Mask */

#define SYS_ALT_MFP2_PB15_TM0_Pos               2                                           /*!< GCR_T::ALT_MFP2: PB15_TM0 Position */
#define SYS_ALT_MFP2_PB15_TM0_Msk               (1ul << SYS_ALT_MFP2_PB15_TM0_Pos)          /*!< GCR_T::ALT_MFP2: PB15_TM0 Mask */

#define SYS_ALT_MFP2_PB14_15_EBI_Pos            0                                           /*!< GCR_T::ALT_MFP2: PB14_15_EBI Position */
#define SYS_ALT_MFP2_PB14_15_EBI_Msk            (1ul << SYS_ALT_MFP2_PB14_15_EBI_Pos)       /*!< GCR_T::ALT_MFP2: PB14_15_EBI Mask */

/* GCR IRCTCTL Bit Field Definitions */
#define SYS_IRCTCTL_CLKERR_STOP_EN_Pos          8                                           /*!< GCR_T::IRCTCTL: CLKERR_STOP_EN Position */
#define SYS_IRCTCTL_CLKERR_STOP_EN_Msk          (1ul << SYS_IRCTCTL_CLKERR_STOP_EN_Pos)     /*!< GCR_T::IRCTCTL: CLKERR_STOP_EN Mask */

#define SYS_IRCTCTL_TRIM_RETRY_CNT_Pos          6                                           /*!< GCR_T::IRCTCTL: TRIM_RETRY_CNT Position */
#define SYS_IRCTCTL_TRIM_RETRY_CNT_Msk          (3ul << SYS_IRCTCTL_TRIM_RETRY_CNT_Pos)     /*!< GCR_T::IRCTCTL: TRIM_RETRY_CNT Mask */

#define SYS_IRCTCTL_TRIM_LOOP_Pos               4                                           /*!< GCR_T::IRCTCTL: TRIM_LOOP Position */
#define SYS_IRCTCTL_TRIM_LOOP_Msk               (3ul << SYS_IRCTCTL_TRIM_LOOP_Pos)          /*!< GCR_T::IRCTCTL: TRIM_LOOP Mask */

#define SYS_IRCTCTL_TRIM_SEL_Pos                0                                           /*!< GCR_T::IRCTCTL: TRIM_SEL Position */
#define SYS_IRCTCTL_TRIM_SEL_Msk                (3ul << SYS_IRCTCTL_TRIM_SEL_Pos)           /*!< GCR_T::IRCTCTL: TRIM_SEL Mask */

/* GCR IRCTIEN Bit Field Definitions */
#define SYS_IRCTIEN_CLKERR_IEN_Pos              2                                           /*!< GCR_T::IRCTIEN: CLKERR_IEN Position */
#define SYS_IRCTIEN_CLKERR_IEN_Msk              (1ul << SYS_IRCTIEN_CLKERR_IEN_Pos)         /*!< GCR_T::IRCTIEN: CLKERR_IEN Mask */

#define SYS_IRCTIEN_TRIM_FAIL_IEN_Pos           1                                           /*!< GCR_T::IRCTIEN: TRIM_FAIL_IEN Position */
#define SYS_IRCTIEN_TRIM_FAIL_IEN_Msk           (1ul << SYS_IRCTIEN_TRIM_FAIL_IEN_Pos)      /*!< GCR_T::IRCTIEN: TRIM_FAIL_IEN Mask */

/* GCR IRCTSTS Bit Field Definitions */
#define SYS_IRCTSTS_CLKERR_INT_Pos              2                                           /*!< GCR_T::IRCTSTS: CLKERR_INT Position */
#define SYS_IRCTSTS_CLKERR_INT_Msk              (1ul << SYS_IRCTSTS_CLKERR_INT_Pos)         /*!< GCR_T::IRCTSTS: CLKERR_INT Mask */

#define SYS_IRCTSTS_TRIM_FAIL_INT_Pos           1                                           /*!< GCR_T::IRCTSTS: TRIM_FAIL_INT Position */
#define SYS_IRCTSTS_TRIM_FAIL_INT_Msk           (1ul << SYS_IRCTSTS_TRIM_FAIL_INT_Pos)      /*!< GCR_T::IRCTSTS: TRIM_FAIL_INT Mask */

#define SYS_IRCTSTS_FREQ_LOCK_Pos               0                                           /*!< GCR_T::IRCTSTS: FREQ_LOCK Position */
#define SYS_IRCTSTS_FREQ_LOCK_Msk               (1ul << SYS_IRCTSTS_FREQ_LOCK_Pos)          /*!< GCR_T::IRCTSTS: FREQ_LOCK Mask */

/* GCR HIRCTCTL Bit Field Definitions */
#define SYS_HIRCTCTL_BOUNDARY_Pos               16                                          /*!< GCR_T::HIRCTCTL: BOUNDARY Position */
#define SYS_HIRCTCTL_BOUNDARY_Msk               (0x1ful << SYS_HIRCTCTL_BOUNDARY_Pos)       /*!< GCR_T::HIRCTCTL: BOUNDARY Mask */

#define SYS_HIRCTCTL_BOUNDEN_Pos                9                                           /*!< GCR_T::HIRCTCTL: BOUNDEN Position */
#define SYS_HIRCTCTL_BOUNDEN_Msk                (1ul << SYS_HIRCTCTL_BOUNDEN_Pos)           /*!< GCR_T::HIRCTCTL: BOUNDEN Mask */

#define SYS_HIRCTCTL_CESTOPEN_Pos               8                                           /*!< GCR_T::HIRCTCTL: CESTOPEN Position */
#define SYS_HIRCTCTL_CESTOPEN_Msk               (1ul << SYS_HIRCTCTL_CESTOPEN_Pos)          /*!< GCR_T::HIRCTCTL: CESTOPEN Mask */

#define SYS_HIRCTCTL_RETRYCNT_Pos               6                                           /*!< GCR_T::HIRCTCTL: RETRYCNT Position */
#define SYS_HIRCTCTL_RETRYCNT_Msk               (3ul << SYS_HIRCTCTL_RETRYCNT_Pos)          /*!< GCR_T::HIRCTCTL: RETRYCNT Mask */

#define SYS_HIRCTCTL_LOOPSEL_Pos                4                                           /*!< GCR_T::HIRCTCTL: LOOPSEL Position */
#define SYS_HIRCTCTL_LOOPSEL_Msk                (3ul << SYS_HIRCTCTL_LOOPSEL_Pos)           /*!< GCR_T::HIRCTCTL: LOOPSEL Mask */

#define SYS_HIRCTCTL_FREQSEL_Pos                0                                           /*!< GCR_T::HIRCTCTL: FREQSEL Position */
#define SYS_HIRCTCTL_FREQSEL_Msk                (3ul << SYS_HIRCTCTL_FREQSEL_Pos)           /*!< GCR_T::HIRCTCTL: FREQSEL Mask */

/* GCR HIRCTIEN Bit Field Definitions */
#define SYS_HIRCTIEN_CLKEIEN_Pos                2                                           /*!< GCR_T::HIRCTIEN: CLKEIEN Position */
#define SYS_HIRCTIEN_CLKEIEN_Msk                (1ul << SYS_HIRCTIEN_CLKEIEN_Pos)           /*!< GCR_T::HIRCTIEN: CLKEIEN Mask */

#define SYS_HIRCTIEN_TFALIEN_Pos                1                                           /*!< GCR_T::HIRCTIEN: TFALIEN Position */
#define SYS_HIRCTIEN_TFALIEN_Msk                (1ul << SYS_HIRCTIEN_TFALIEN_Pos)           /*!< GCR_T::HIRCTIEN: TFALIEN Mask */

/* GCR HIRCTSTS Bit Field Definitions */
#define SYS_HIRCTSTS_OVBDIF_Pos                 3                                           /*!< GCR_T::HIRCTSTS: OVBDIF Position */
#define SYS_HIRCTSTS_OVBDIF_Msk                 (1ul << SYS_HIRCTSTS_OVBDIF_Pos)            /*!< GCR_T::HIRCTSTS: OVBDIF Mask */

#define SYS_HIRCTSTS_CLKERIF_Pos                2                                           /*!< GCR_T::HIRCTSTS: CLKERIF Position */
#define SYS_HIRCTSTS_CLKERIF_Msk                (1ul << SYS_HIRCTSTS_CLKERIF_Pos)           /*!< GCR_T::HIRCTSTS: CLKERIF Mask */

#define SYS_HIRCTSTS_TFAILIF_Pos                1                                           /*!< GCR_T::HIRCTSTS: TFAILIF Position */
#define SYS_HIRCTSTS_TFAILIF_Msk                (1ul << SYS_HIRCTSTS_TFAILIF_Pos)           /*!< GCR_T::HIRCTSTS: TFAILIF Mask */

#define SYS_HIRCTSTS_FREQLOCK_Pos               0                                           /*!< GCR_T::HIRCTSTS: FREQLOCK Position */
#define SYS_HIRCTSTS_FREQLOCK_Msk               (1ul << SYS_HIRCTSTS_FREQLOCK_Pos)          /*!< GCR_T::HIRCTSTS: FREQLOCK Mask */

/* GCR REGWRPROT Bit Field Definitions */
#define SYS_REGWRPROT_REGWRPROT_Pos             0                                           /*!< GCR_T::REGWRPROT: REGWRPROT Position */
#define SYS_REGWRPROT_REGWRPROT_Msk             (0xFFul << SYS_REGWRPROT_REGWRPROT_Pos)     /*!< GCR_T::REGWRPROT: REGWRPROT Mask */

#define SYS_REGWRPROT_REGPROTDIS_Pos            0                                       /*!< GCR_T::REGWRPROT: REGPROTDIS Position */
#define SYS_REGWRPROT_REGPROTDIS_Msk            (1ul << SYS_REGWRPROT_REGPROTDIS_Pos)   /*!< GCR_T::REGWRPROT: REGPROTDIS Mask */
/*@}*/ /* end of group SYS_CONST */



typedef struct
{

/**
 * @var GCR_INT_T::IRQSRC[32]
 * Offset: 0x00-0x7C  IRQn(n=0~31) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |IRQ_SRC[0].0 - BOD INT
 * |        |          |IRQ_SRC[1].0 - WDT INT
 * |        |          |IRQ_SRC[1].1 - WWDT INT
 * |        |          |IRQ_SRC[2].0 - EINT0, external interrupt 0 from PB.14
 * |        |          |IRQ_SRC[3].0 - EINT1, external interrupt 1 from PB.15
 * |        |          |IRQ_SRC[4].0 - GPA INT
 * |        |          |IRQ_SRC[4].1 - GPB INT
 * |        |          |IRQ_SRC[5].0 - GPC INT
 * |        |          |IRQ_SRC[5].2 - GPE INT
 * |        |          |IRQ_SRC[5].3 - GPF INT
 * |        |          |IRQ_SRC[6].0 - PWM0 INT
 * |        |          |IRQ_SRC[6].1 - PWM1 INT
 * |        |          |IRQ_SRC[6].2 - PWM2 INT
 * |        |          |IRQ_SRC[6].3 - PWM3 INT
 * |        |          |IRQ_SRC[7].0 - PWM4 INT
 * |        |          |IRQ_SRC[7].1 - PWM5 INT
 * |        |          |IRQ_SRC[8].0 - TMR0 INT
 * |        |          |IRQ_SRC[9].0 - TMR1 INT
 * |        |          |IRQ_SRC[10].0 - TMR2 INT
 * |        |          |IRQ_SRC[11].0 - TMR3 INT
 * |        |          |IRQ_SRC[12].0 - UART0 INT
 * |        |          |IRQ_SRC[12].1 - UART2 INT
 * |        |          |IRQ_SRC[13].0 - UART1 INT
 * |        |          |IRQ_SRC[14].0 - SPI0 INT
 * |        |          |IRQ_SRC[15].0 - SPI1 INT
 * |        |          |IRQ_SRC[18].0 - I2C0 INT
 * |        |          |IRQ_SRC[19].0 - I2C1 INT
 * |        |          |IRQ_SRC[23].0 - USB INT
 * |        |          |IRQ_SRC[26].0 - PDMA INT
 * |        |          |IRQ_SRC[28].0 - Power Down Wake up INT
 * |        |          |IRQ_SRC[29].0 - ADC INT
 * |        |          |IRQ_SRC[30].0 - IRC INT
 * |        |          |IRQ_SRC[31].0 - RTC INT
 * @var GCR_INT_T::NMISEL
 * Offset: 0x80  NMI Interrupt Source Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |NMI_SEL   |NMI interrupt source selection
 * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of IRQ0~IRQ31 by setting NMI_SEL with IRQ number.
 * |        |          |The default NMI interrupt is assigned as IRQ0 interrupt if NMI is enabled by setting NMI_SEL[8].
 * |[8]     |NMI_EN    |NMI interrupt enable (Write Protect)
 * |        |          |0 = IRQ0~31 assigned to NMI interrupt Disabled. (NMI still can be software triggered by setting its pending flag.)
 * |        |          |1 = IRQ0~31 assigned to NMI interrupt Enabled.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var GCR_INT_T::MCUIRQ
 * Offset: 0x84  MCU Interrupt Request Source Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |MCU_IRQ   |MCU IRQ Source Register
 * |        |          |The MCU_IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0.
 * |        |          |When the MCU_IRQ[n] is 0:
 * |        |          |0 = No effect.
 * |        |          |1 = Generate an interrupt to Cortex_M0 NVIC[n].
 * |        |          |When the MCU_IRQ[n] is 1 (means an interrupt is assert):
 * |        |          |0 = No effect.
 * |        |          |1 = Clear the interrupt and MCU_IRQ[n].
 * @var GCR_INT_T::MCUIRQCR
 * Offset: 0x88  MCU Interrupt Request Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FAST_IRQ  |Fast IRQ Latency Enable
 * |        |          |0 = MCU IRQ latency is fixed at 13 clock cycles of HCLK, MCU will enter IRQ handler after this fixed latency when interrupt happened.
 * |        |          |1 = MCU IRQ latency will not fixed, MCU will enter IRQ handler as soon as possible when interrupt happened.
 */

    __I  uint32_t IRQSRC[32];   /* Offset: 0x00-0x7C  IRQn(n=0~31) Interrupt Source Identity Register               */
    __IO uint32_t NMISEL;       /* Offset: 0x80  NMI Interrupt Source Select Control Register                       */
    __IO uint32_t MCUIRQ;       /* Offset: 0x84  MCU Interrupt Request Source Register                              */
    __IO uint32_t MCUIRQCR;     /* Offset: 0x88  MCU MCU Interrupt Request Control Register                         */ 

} GCR_INT_T;


/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */


/* INT IRQSRC Bit Field Definitions */
#define INT_IRQSRC_INT_SRC_Pos                  0                                   /*!< GCR_INT_T::IRQSRC: INT_SRC Position */
#define INT_IRQSRC_INT_SRC_Msk                  (0xFul << INT_IRQSRC_INT_SRC_Pos)

/* INT NMISEL Bit Field Definitions */
#define INT_NMISEL_NMI_EN_Pos                   8                                   /*!< GCR_INT_T::NMISEL: NMI_EN Position */
#define INT_NMISEL_NMI_EN_Msk                   (1ul << INT_NMISEL_NMI_EN_Pos)      /*!< GCR_INT_T::NMISEL: NMI_EN Mask */

#define INT_NMISEL_NMI_SEL_Pos                  0                                   /*!< GCR_INT_T::NMISEL: NMI_SEL Position */
#define INT_NMISEL_NMI_SEL_Msk                  (0x1Ful << INT_NMISEL_NMI_SEL_Pos)  /*!< GCR_INT_T::NMISEL: NMI_SEL Mask */

/* INT MCUIRQ Bit Field Definitions */
#define INT_MCUIRQ_MCU_IRQ_Pos                  0                                           /*!< GCR_INT_T::MCUIRQ: MCU_IRQ Position */
#define INT_MCUIRQ_MCU_IRQ_Msk                  (0xFFFFFFFFul << INT_MCUIRQ_MCU_IRQ_Pos)    /*!< GCR_INT_T::MCUIRQ: MCU_IRQ Mask */

/* INT MCUIRQCR Bit Field Definitions */
#define INT_MCUIRQCR_MCU_IRQ_Pos                0                                   /*!< GCR_INT_T::MCUIRQCR: FAST_IRQ Position */
#define INT_MCUIRQCR_MCU_IRQ_Msk                (1ul << INT_MCUIRQCR_MCU_IRQ_Pos)   /*!< GCR_INT_T::MCUIRQCR: FAST_IRQ Mask */

/*@}*/ /* end of group INT_CONST */
/*@}*/ /* end of group SYS */
/**@}*/ /* end of REGISTER group */

#endif /* __SYS_REG_H__ */
