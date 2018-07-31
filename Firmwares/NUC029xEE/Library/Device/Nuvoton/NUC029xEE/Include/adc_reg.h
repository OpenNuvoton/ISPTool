/**************************************************************************//**
 * @file     adc_reg.h
 * @version  V1.00
 * @brief    ADC register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ADC_REG_H__
#define __ADC_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*----------------------------- ADC Controller -------------------------------*/
/** @addtogroup ADC Analog to Digital Converter (ADC)
  Memory Mapped Structure for ADC Controller
  @{
 */


typedef struct
{

/**
 * @var ADC_T::ADDR
 * Offset: 0x00-0x1C  ADC Data Register  0 ~ 7
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RSLT      |A/D Conversion Result
 * |        |          |This field contains conversion result of ADC.
 * |        |          |When DMOF bit (ADCR[31]) set to 0, 12-bit ADC conversion result with unsigned format will be
 * |        |          |filled in RSLT (ADDRx[11:0], x=0~7) and zero will be filled in RSLT (ADDRx[15:12], x=0~7).
 * |        |          |When DMOF bit (ADCR[31]) set to 1, 12-bit ADC conversion result with 2'complement format will be
 * |        |          |filled in RSLT(ADDRx[11:0], x=0~7) and signed bits to will be filled in RSLT (ADDRx[15:12],
 * |        |          |x=0~7).
 * |[16]    |OVERRUN   |Overrun Flag
 * |        |          |0 = Data in RSLT (ADDRx[15:0], x=0~7) is recent conversion result.
 * |        |          |1 = Data in RSLT (ADDRx[15:0], x=0~7) is overwritten.
 * |        |          |If converted data in RSLT has not been read before new conversion result is loaded to this
 * |        |          |register, OVERRUN is set to 1 and previous conversion result is gone.
 * |        |          |It is cleared by hardware after ADDR register is read.
 * |        |          |This is a read only bit.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RSLT bits (ADDRx[15:0], x=0~7) is not valid.
 * |        |          |1 = Data in RSLT bits (ADDRx[15:0], x=0~7) is valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared
 * |        |          |by hardware after ADDR register is read.
 * |        |          |This is a read only bit
 * @var ADC_T::ADCR
 * Offset: 0x20  ADC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADEN      |A/D Converter Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |Before starting A/D conversion function, this bit should be set to 1.
 * |        |          |Clear it to 0 to disable A/D converter analog circuit for saving power consumption.
 * |[1]     |ADIE      |A/D Interrupt Enable
 * |        |          |0 = A/D interrupt function Disabled.
 * |        |          |1 = A/D interrupt function Enabled.
 * |        |          |A/D conversion end interrupt request is generated if ADIE bit (ADCR[1]) is set to 1.
 * |[3:2]   |ADMD      |A/D Converter Operation Mode
 * |        |          |00 = Single conversion.
 * |        |          |01 = Reserved.
 * |        |          |10 = Single-cycle scan.
 * |        |          |11 = Continuous scan.
 * |        |          |When changing the operation mode, software should disable ADST bit (ADCR[11]) firstly.
 * |[5:4]   |TRGS      |Hardware Trigger Source
 * |        |          |00 = A/D conversion is started by external STADC pin.
 * |        |          |11 = A/D conversion is started by PWM Center-aligned trigger.
 * |        |          |Others = Reserved.
 * |        |          |Software should disable TRGEN (ADCR[8]) and ADST (ADCR[11]) before change TRGS.
 * |[7:6]   |TRGCOND   |External Trigger Condition
 * |        |          |These two bits decide external pin STADC trigger event is level or edge.
 * |        |          |The signal must be kept at stable state at least 8 PCLKs for level trigger and 4 PCLKs at high
 * |        |          |and low state for edge trigger.
 * |        |          |00 = Low level.
 * |        |          |01 = High level.
 * |        |          |10 = Falling edge.
 * |        |          |11 = Rising edge.
 * |[8]     |TRGEN     |Hardware Trigger Enable
 * |        |          |Enable or disable triggering of A/D conversion by hardware (external STADC pin or PWM
 * |        |          |Center-aligned trigger).
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |ADC hardware trigger function is only supported in single-cycle scan mode.
 * |        |          |If hardware trigger mode, the ADST bit (ADCR[11]) can be set to 1 by the selected hardware
 * |        |          |trigger source.
 * |[9]     |PTEN      |PDMA Transfer Enable
 * |        |          |0 = PDMA data transfer Disabled.
 * |        |          |1 = PDMA data transfer in ADDR 0~7 Enabled.
 * |        |          |When A/D conversion is completed, the converted data is loaded into ADDR 0~7, software can
 * |        |          |enable this bit to generate a PDMA data transfer request.
 * |        |          |When PTEN=1, software must set ADIE=0 (ADCR[1]) to disable interrupt.
 * |[10]    |DIFFEN    |Differential Input Mode Enable
 * |        |          |0 = Single-end analog input mode.
 * |        |          |1 = Differential analog input mode.
 * |        |          |Differential   input Paired Channel
 * |        |          |Differential input voltage (Vdiff) = Vplus - Vminus, where Vplus
 * |        |          |is the analog input; Vminus is the inverted analog input.
 * |        |          |In differential input mode, only the even number of the two corresponding channels needs to be
 * |        |          |enabled in ADCHER.
 * |        |          |The conversion result will be placed to the corresponding data register of the enabled channel.
 * |[11]    |ADST      |A/D Conversion Start
 * |        |          |0 = Conversion stops and A/D converter enter idle state.
 * |        |          |1 = Conversion starts.
 * |        |          |ADST bit can be set to 1 from three sources: software, PWM Center-aligned trigger and external
 * |        |          |pin STADC.
 * |        |          |ADST will be cleared to 0 by hardware automatically at the ends of single mode and single-cycle
 * |        |          |scan mode.
 * |        |          |In continuous scan mode, A/D conversion is continuously performed until software writes 0 to
 * |        |          |this bit or chip reset.
 * |[31]    |DMOF      |A/D Differential Input Mode Output Format
 * |        |          |0 = A/D Conversion result will be filled in RSLT at ADDRx registers with unsigned format.
 * |        |          |1 = A/D Conversion result will be filled in RSLT at ADDRx registers with 2'complement format.
 * @var ADC_T::ADCHER
 * Offset: 0x24  ADC Channel Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CHEN      |Analog Input Channel Enable Bit
 * |        |          |Set CHEN[7:0] to enable the corresponding analog input channel 7 ~ 0.
 * |        |          |If DIFFEN bit (ADCR[10]) is set to 1, only the even number channels need to be enabled.
 * |        |          |0 = ADC input channel Disabled.
 * |        |          |1 = ADC input channel Enabled.
 * |[9:8]   |PRESEL    |Analog Input Channel 7 Select
 * |        |          |00 = External analog input.
 * |        |          |01 = Internal band-gap voltage.
 * |        |          |10 = Internal temperature sensor.
 * |        |          |11 = Reserved.
 * |        |          |Note:
 * |        |          |When software select the band-gap voltage as the analog input source of ADC channel 7, ADC clock
 * |        |          |rate needs to be limited to slower than 300 kHz.
 * |[13:10] |CHEN1     |Analog Input Channel Enable Bit 1
 * |        |          |Set CHEN[13:10] to enable the corresponding analog input channel 11 ~ 8.
 * |        |          |If DIFFEN bit (ADCR[10]) is set to 1, only the even number channels need to be enabled.
 * |        |          |0 = ADC input channel Disabled.
 * |        |          |1 = ADC input channel Enabled.
 * @var ADC_T::ADCMPR
 * Offset: 0x28  ADC Compare Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CMPEN     |Compare Enable
 * |        |          |0 = Compare function Disabled.
 * |        |          |1 = Compare function Enabled.
 * |        |          |Set this bit to 1 to enable ADC controller to compare CMPD (ADCMPR0/1[27:16]) with specified
 * |        |          |channel conversion result when converted data is loaded into ADDR register.
 * |[1]     |CMPIE     |Compare Interrupt Enable
 * |        |          |0 = Compare function interrupt Disabled.
 * |        |          |1 = Compare function interrupt Enabled.
 * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND
 * |        |          |(ADCMPR0/1[2]) and CMPMATCNT (ADCMPR0/1[11:8]), CMPF0/1 bit (ADSR[1]/[2]) will be asserted, in
 * |        |          |the meanwhile, if CMPIE (ADCMPR0/1[1]) is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the
 * |        |          |12-bit CMPD (ADCMPR0/1[27:16]), the internal match counter will increase one.
 * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater or equal to
 * |        |          |the 12-bit CMPD (ADCMPR0/1[27:16]), the internal match counter will increase one.
 * |        |          |Note: When the internal counter reaches the value to (CMPMATCNT (ADCMPR0/1[11:8])+1), the
 * |        |          |CMPF0/1 bit (ADSR[1]/[2]) will be set.
 * |[6:3]   |CMPCH     |Compare Channel Selection
 * |        |          |0000 = Channel 0 conversion result is selected to be compared.
 * |        |          |0001 = Channel 1 conversion result is selected to be compared.
 * |        |          |0010 = Channel 2 conversion result is selected to be compared.
 * |        |          |0011 = Channel 3 conversion result is selected to be compared.
 * |        |          |0100 = Channel 4 conversion result is selected to be compared.
 * |        |          |0101 = Channel 5 conversion result is selected to be compared.
 * |        |          |0110 = Channel 6 conversion result is selected to be compared.
 * |        |          |0111 = Channel 7 conversion result is selected to be compared.
 * |        |          |1000 = Channel 8 conversion result is selected to be compared.
 * |        |          |1001 = Channel 9 conversion result is selected to be compared.
 * |        |          |1010 = Channel 10 conversion result is selected to be compared.
 * |        |          |1011 = Channel 11 conversion result is selected to be compared.
 * |[11:8]  |CMPMATCNT |Compare Match Count
 * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by
 * |        |          |CMPCOND (ADCMPR0/1[2]), the internal match counter will increase 1.
 * |        |          |When the internal counter reaches the value to (CMPMATCNT (ADCMPR0/1[11:8]) +1), the CMPF0/1 bit
 * |        |          |(ADSR[1]/[2]) will be set.
 * |[27:16] |CMPD      |Comparison Data
 * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
 * |        |          |When DMOF bit (ADCR[31]) is set to 0, ADC comparator compares CMPD with conversion result with
 * |        |          |unsigned format.
 * |        |          |CMPD should be filled in unsigned format.
 * |        |          |When DMOF bit (ADCR[31]) is set to 1, ADC comparator compares CMPD with conversion result with
 * |        |          |2'complement format.
 * |        |          |CMPD should be filled in 2'complement format.
 * @var ADC_T::ADSR
 * Offset: 0x30  ADC Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADF       |A/D Conversion End Flag
 * |        |          |A status flag that indicates the end of A/D conversion.
 * |        |          |ADF is set to 1 at these two conditions:
 * |        |          |1. When A/D conversion ends in Single mode.
 * |        |          |2. When A/D conversion ends on all specified channels in Scan mode.
 * |        |          |This flag can be cleared by writing 1 to itself.
 * |[1]     |CMPF0     |Compare Flag
 * |        |          |When the selected channel A/D conversion result meets setting condition in ADCMPR0 then this bit
 * |        |          |is set to 1.
 * |        |          |And it is cleared by writing 1 to self.
 * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
 * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
 * |[2]     |CMPF1     |Compare Flag
 * |        |          |When the selected channel A/D conversion result meets setting condition in ADCMPR1 then this bit
 * |        |          |is set to 1.
 * |        |          |And it is cleared by writing 1 to self.
 * |        |          |0 = Conversion result in ADDR does not meet ADCMPR1 setting.
 * |        |          |1 = Conversion result in ADDR meets ADCMPR1 setting.
 * |[3]     |BUSY      |BUSY/IDLE
 * |        |          |0 = A/D converter is in idle state.
 * |        |          |1 = A/D converter is busy at conversion.
 * |        |          |This bit is mirror of as ADST bit (ADCR[11]).
 * |        |          |It is read only.
 * |[7:4]   |CHANNEL   |Current Conversion Channel
 * |        |          |This field reflects the current conversion channel when BUSY = 1 (ADSR[3]).
 * |        |          |When BUSY = 0, it shows the number of the next converted channel.
 * |        |          |It is read only.
 * |[15:8]  |VALID     |Data Valid Flag
 * |        |          |It is a mirror of VALID bit (ADDR0~7[17]).
 * |        |          |It is read only.
 * |[23:16] |OVERRUN   |Overrun Flag
 * |        |          |It is a mirror to OVERRUN bit (ADDR0~7[16]).
 * |        |          |It is read only.
 * |[27:24] |VALID1    |Data Valid Flag 1
 * |        |          |It is a mirror of VALID bit (ADDR8~11[17]).
 * |        |          |It is read only.
 * |[31:28] |OVERRUN   |Overrun Flag 1
 * |        |          |It is a mirror to OVERRUN bit (ADDR8~11[16]).
 * |        |          |It is read only.
 * @var ADC_T::ADPDMA
 * Offset: 0x40  ADC PDMA Current Transfer Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[17:0]  |AD_PDMA   |ADC PDMA Current Transfer Data Register
 * |        |          |When PDMA transferring, read this register can monitor current PDMA transfer data.
 * |        |          |Current PDMA transfer data is the content of ADDR0 ~ ADDR7.
 * |        |          |This is a read only register.
 * @var ADC_T::ADDR1
 * Offset: 0x50-0x5C  ADC Data Register  8 ~ 11
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RSLT      |A/D Conversion Result
 * |        |          |This field contains conversion result of ADC.
 * |        |          |When DMOF bit (ADCR[31]) set to 0, 12-bit ADC conversion result with unsigned format will be
 * |        |          |filled in RSLT (ADDRx[11:0], x=8~11) and zero will be filled in RSLT (ADDRx[15:12], x=8~11).
 * |        |          |When DMOF bit (ADCR[31]) set to 1, 12-bit ADC conversion result with 2'complement format will be
 * |        |          |filled in RSLT(ADDRx[11:0], x=8~11) and signed bits to will be filled in RSLT (ADDRx[15:12],
 * |        |          |x=0~7).
 * |[16]    |OVERRUN   |Overrun Flag
 * |        |          |0 = Data in RSLT (ADDRx[15:0], x=8~11) is recent conversion result.
 * |        |          |1 = Data in RSLT (ADDRx[15:0], x=8~11) is overwritten.
 * |        |          |If converted data in RSLT has not been read before new conversion result is loaded to this
 * |        |          |register, OVERRUN is set to 1 and previous conversion result is gone.
 * |        |          |It is cleared by hardware after ADDR register is read.
 * |        |          |This is a read only bit.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RSLT bits (ADDRx[15:0], x=8~11) is not valid.
 * |        |          |1 = Data in RSLT bits (ADDRx[15:0], x=8~11) is valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared
 * |        |          |by hardware after ADDR register is read.
 * |        |          |This is a read only bit
 */

    __I  uint32_t ADDR[8];       /* Offset: 0x00-0x1C  ADC Data Register 0 ~ 7                                       */
    __IO uint32_t ADCR;          /* Offset: 0x20  ADC Control Register                                               */
    __IO uint32_t ADCHER;        /* Offset: 0x24  ADC Channel Enable Register                                        */
    __IO uint32_t ADCMPR[2];     /* Offset: 0x28  ADC Compare Register                                               */
    __IO uint32_t ADSR;          /* Offset: 0x30  ADC Status Register                                                */
    __I  uint32_t RESERVE0[3];  
    __I  uint32_t ADPDMA;        /* Offset: 0x40  ADC PDMA Current Transfer Data Register                            */
    __I  uint32_t RESERVE1[3];  
    __I  uint32_t ADDR1[4];       /* Offset: 0x50-0x5C  ADC Data Register 8 ~ 11                                     */

} ADC_T;


/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */



/* ADDR Bit Field Definitions */
#define ADC_ADDR_VALID_Pos      17                                /*!< ADC_T::ADDR: VALID Position */
#define ADC_ADDR_VALID_Msk      (1ul << ADC_ADDR_VALID_Pos)       /*!< ADC_T::ADDR: VALID Mask */

#define ADC_ADDR_OVERRUN_Pos    16                                /*!< ADC_T::ADDR: OVERRUN Position */
#define ADC_ADDR_OVERRUN_Msk    (1ul << ADC_ADDR_OVERRUN_Pos)     /*!< ADC_T::ADDR: OVERRUN Mask */

#define ADC_ADDR_RSLT_Pos       0                                 /*!< ADC_T::ADDR: RSLT Position */
#define ADC_ADDR_RSLT_Msk       (0xFFFFul << ADC_ADDR_RSLT_Pos)   /*!< ADC_T::ADDR: RSLT Mask */

/* ADCR Bit Field Definitions */
#define ADC_ADCR_DMOF_Pos       31                                /*!< ADC_T::ADCR: DMOF Position */
#define ADC_ADCR_DMOF_Msk       (1ul << ADC_ADCR_DMOF_Pos)        /*!< ADC_T::ADCR: DMOF Mask */

#define ADC_ADCR_ADST_Pos       11                                /*!< ADC_T::ADCR: ADST Position */
#define ADC_ADCR_ADST_Msk       (1ul << ADC_ADCR_ADST_Pos)        /*!< ADC_T::ADCR: ADST Mask */

#define ADC_ADCR_DIFFEN_Pos     10                                /*!< ADC_T::ADCR: DIFFEN Position */
#define ADC_ADCR_DIFFEN_Msk     (1ul << ADC_ADCR_DIFFEN_Pos)      /*!< ADC_T::ADCR: DIFFEN Mask */

#define ADC_ADCR_PTEN_Pos       9                                 /*!< ADC_T::ADCR: PTEN Position */
#define ADC_ADCR_PTEN_Msk       (1ul << ADC_ADCR_PTEN_Pos)        /*!< ADC_T::ADCR: PTEN Mask */

#define ADC_ADCR_TRGEN_Pos      8                                 /*!< ADC_T::ADCR: TRGEN Position */
#define ADC_ADCR_TRGEN_Msk      (1ul << ADC_ADCR_TRGEN_Pos)       /*!< ADC_T::ADCR: TRGEN Mask */

#define ADC_ADCR_TRGCOND_Pos    6                                 /*!< ADC_T::ADCR: TRGCOND Position */
#define ADC_ADCR_TRGCOND_Msk    (3ul << ADC_ADCR_TRGCOND_Pos)     /*!< ADC_T::ADCR: TRGCOND Mask */

#define ADC_ADCR_TRGS_Pos       4                                 /*!< ADC_T::ADCR: TRGS Position */
#define ADC_ADCR_TRGS_Msk       (3ul << ADC_ADCR_TRGS_Pos)        /*!< ADC_T::ADCR: TRGS Mask */

#define ADC_ADCR_ADMD_Pos       2                                 /*!< ADC_T::ADCR: ADMD Position */
#define ADC_ADCR_ADMD_Msk       (3ul << ADC_ADCR_ADMD_Pos)        /*!< ADC_T::ADCR: ADMD Mask */

#define ADC_ADCR_ADIE_Pos       1                                 /*!< ADC_T::ADCR: ADIE Position */
#define ADC_ADCR_ADIE_Msk       (1ul << ADC_ADCR_ADIE_Pos)        /*!< ADC_T::ADCR: ADIE Mask */

#define ADC_ADCR_ADEN_Pos       0                                 /*!< ADC_T::ADCR: ADEN Position */
#define ADC_ADCR_ADEN_Msk       (1ul << ADC_ADCR_ADEN_Pos)        /*!< ADC_T::ADCR: ADEN Mask */

/* ADCHER Bit Field Definitions */
#define ADC_ADCHER_CHEN1_Pos    10                                /*!< ADC_T::ADCHER: CHEN1 Position */
#define ADC_ADCHER_CHEN1_Msk    (0xFul << ADC_ADCHER_CHEN1_Pos)   /*!< ADC_T::ADCHER: CHEN1 Mask */

#define ADC_ADCHER_PRESEL_Pos   8                                 /*!< ADC_T::ADCHER: PRESEL Position */
#define ADC_ADCHER_PRESEL_Msk   (3ul << ADC_ADCHER_PRESEL_Pos)    /*!< ADC_T::ADCHER: PRESEL Mask */

#define ADC_ADCHER_CHEN_Pos     0                                 /*!< ADC_T::ADCHER: CHEN Position */
#define ADC_ADCHER_CHEN_Msk     (0xFFul << ADC_ADCHER_CHEN_Pos)   /*!< ADC_T::ADCHER: CHEN Mask */

/* ADCMPR Bit Field Definitions */
#define ADC_ADCMPR_CMPD_Pos        16                                    /*!< ADC_T::ADCMPR: CMPD Position */
#define ADC_ADCMPR_CMPD_Msk        (0xFFFul << ADC_ADCMPR_CMPD_Pos)      /*!< ADC_T::ADCMPR: CMPD Mask */

#define ADC_ADCMPR_CMPMATCNT_Pos   8                                     /*!< ADC_T::ADCMPR: CMPMATCNT Position */
#define ADC_ADCMPR_CMPMATCNT_Msk   (0xFul << ADC_ADCMPR_CMPMATCNT_Pos)   /*!< ADC_T::ADCMPR: CMPMATCNT Mask */

#define ADC_ADCMPR_CMPCH_Pos       3                                     /*!< ADC_T::ADCMPR: CMPCH Position */
#define ADC_ADCMPR_CMPCH_Msk       (0xFul << ADC_ADCMPR_CMPCH_Pos)       /*!< ADC_T::ADCMPR: CMPCH Mask */

#define ADC_ADCMPR_CMPCOND_Pos     2                                     /*!< ADC_T::ADCMPR: CMPCOND Position */
#define ADC_ADCMPR_CMPCOND_Msk     (1ul << ADC_ADCMPR_CMPCOND_Pos)       /*!< ADC_T::ADCMPR: CMPCOND Mask */

#define ADC_ADCMPR_CMPIE_Pos       1                                     /*!< ADC_T::ADCMPR: CMPIE Position */
#define ADC_ADCMPR_CMPIE_Msk       (1ul << ADC_ADCMPR_CMPIE_Pos)         /*!< ADC_T::ADCMPR: CMPIE Mask */

#define ADC_ADCMPR_CMPEN_Pos       0                                     /*!< ADC_T::ADCMPR: CMPEN Position */
#define ADC_ADCMPR_CMPEN_Msk       (1ul << ADC_ADCMPR_CMPEN_Pos)         /*!< ADC_T::ADCMPR: CMPEN Mask */

/* ADSR Bit Field Definitions */
#define ADC_ADSR_OVERRUN1_Pos      28                                    /*!< ADC_T::ADSR: OVERRUN1 Position */
#define ADC_ADSR_OVERRUN1_Msk      (0xFul << ADC_ADSR_OVERRUN1_Pos)      /*!< ADC_T::ADSR: OVERRUN1 Mask */

#define ADC_ADSR_VALID1_Pos        24                                    /*!< ADC_T::ADSR: VALID1 Position */
#define ADC_ADSR_VALID1_Msk        (0xFul << ADC_ADSR_VALID1_Pos)        /*!< ADC_T::ADSR: VALID1 Mask */

#define ADC_ADSR_OVERRUN_Pos       16                                    /*!< ADC_T::ADSR: OVERRUN Position */
#define ADC_ADSR_OVERRUN_Msk       (0xFFul << ADC_ADSR_OVERRUN_Pos)      /*!< ADC_T::ADSR: OVERRUN Mask */

#define ADC_ADSR_VALID_Pos         8                                     /*!< ADC_T::ADSR: VALID Position */
#define ADC_ADSR_VALID_Msk         (0xFFul << ADC_ADSR_VALID_Pos)        /*!< ADC_T::ADSR: VALID Mask */

#define ADC_ADSR_CHANNEL_Pos       4                                     /*!< ADC_T::ADSR: CHANNEL Position */
#define ADC_ADSR_CHANNEL_Msk       (0xFul << ADC_ADSR_CHANNEL_Pos)       /*!< ADC_T::ADSR: CHANNEL Mask */

#define ADC_ADSR_BUSY_Pos          3                                     /*!< ADC_T::ADSR: BUSY Position */
#define ADC_ADSR_BUSY_Msk          (1ul << ADC_ADSR_BUSY_Pos)            /*!< ADC_T::ADSR: BUSY Mask */

#define ADC_ADSR_CMPF1_Pos         2                                     /*!< ADC_T::ADSR: CMPF1 Position */
#define ADC_ADSR_CMPF1_Msk         (1ul << ADC_ADSR_CMPF1_Pos)           /*!< ADC_T::ADSR: CMPF1 Mask */

#define ADC_ADSR_CMPF0_Pos         1                                     /*!< ADC_T::ADSR: CMPF0 Position */
#define ADC_ADSR_CMPF0_Msk         (1ul << ADC_ADSR_CMPF0_Pos)           /*!< ADC_T::ADSR: CMPF0 Mask */

#define ADC_ADSR_ADF_Pos           0                                     /*!< ADC_T::ADSR: ADF Position */
#define ADC_ADSR_ADF_Msk           (1ul << ADC_ADSR_ADF_Pos)             /*!< ADC_T::ADSR: ADF Mask */

/* ADPDMA Bit Field Definitions */
#define ADC_ADPDMA_AD_PDMA_Pos     0                                     /*!< ADC_T::ADPDMA: AD_PDMA Position */
#define ADC_ADPDMA_AD_PDMA_Msk     (0x3FFFFul << ADC_ADPDMA_AD_PDMA_Pos) /*!< ADC_T::ADPDMA: AD_PDMA Mask */

/* ADDR Bit 1 Field Definitions */
#define ADC_ADDR1_VALID_Pos      17                                 /*!< ADC_T::ADDR1: VALID Position */
#define ADC_ADDR1_VALID_Msk      (1ul << ADC_ADDR1_VALID_Pos)       /*!< ADC_T::ADDR1: VALID Mask */

#define ADC_ADDR1_OVERRUN_Pos    16                                 /*!< ADC_T::ADDR1: OVERRUN Position */
#define ADC_ADDR1_OVERRUN_Msk    (1ul << ADC_ADDR1_OVERRUN_Pos)     /*!< ADC_T::ADDR1: OVERRUN Mask */

#define ADC_ADDR1_RSLT_Pos       0                                  /*!< ADC_T::ADDR1: RSLT Position */
#define ADC_ADDR1_RSLT_Msk       (0xFFFFul << ADC_ADDR1_RSLT_Pos)   /*!< ADC_T::ADDR1: RSLT Mask */

/*@}*/ /* end of group ADC_CONST */
/*@}*/ /* end of group ADC */
/**@}*/ /* end of REGISTER group */

#endif /* __ADC_REG_H__ */


