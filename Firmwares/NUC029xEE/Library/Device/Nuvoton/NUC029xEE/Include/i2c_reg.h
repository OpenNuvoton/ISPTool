/**************************************************************************//**
 * @file     i2c_reg.h
 * @version  V1.00
 * @brief    I2C register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __I2C_REG_H__
#define __I2C_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */



/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller (I2C)
    Memory Mapped Structure for I2C Controller
@{ */


typedef struct
{


/**
 * @var I2C_T::I2CON
 * Offset: 0x00  I2C Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2]     |AA        |Assert Acknowledge Control
 * |        |          |When AA =1 prior to address or data received, an acknowledged (low level to I2Cn_SDA) will be
 * |        |          |returned during the acknowledge clock pulse on the I2Cn_SCL line when 
 * |        |          |1. A slave is acknowledging the address sent from master, 
 * |        |          |2. The receiver devices are acknowledging the data sent by transmitter.
 * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to I2Cn_SDA) will be
 * |        |          |returned during the acknowledge clock pulse on the I2Cn_SCL line.
 * |[3]     |SI        |I2C Interrupt Flag
 * |        |          |When a new I2C state is present in the I2CSTATUS register, the SI flag is set by hardware, and
 * |        |          |if bit EI (I2CON [7]) is set, the I2C interrupt is requested.
 * |        |          |SI must be cleared by software.
 * |        |          |Clear SI by writing 1 to this bit.
 * |[4]     |STO       |I2C STOP Control
 * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the
 * |        |          |bus condition if a STOP condition is detected this bit will be cleared by hardware
 * |        |          |automatically.
 * |        |          |In a slave mode, setting STO resets I2C hardware to the defined "not addressed" slave mode.
 * |        |          |This means it is NO LONGER in the slave receiver mode to receive data from the master transmit
 * |        |          |device.
 * |[5]     |STA       |I2C START Control
 * |        |          |Setting STA to logic 1 to enter Master mode, the I2C hardware sends a START or repeat START
 * |        |          |condition to bus when the bus is free.
 * |[6]     |ENS1      |I2C Controller Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |Set to enable I2C serial function controller.
 * |        |          |When ENS1=1 the I2C serial function enables.
 * |        |          |The multi-function pin function of I2Cn_SDA and I2Cn_SCL must set to I2C function first.
 * |[7]     |EI        |Enable Interrupt
 * |        |          |0 = I2C interrupt Disabled.
 * |        |          |1 = I2C interrupt Enabled.
 * @var I2C_T::I2CADDR0
 * Offset: 0x04  I2C Slave Address Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = General Call Function Disabled.
 * |        |          |1 = General Call Function Enabled.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in Master mode.
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
 * |        |          |The I2C hardware will react if either of the address is matched.
 * @var I2C_T::I2CDAT
 * Offset: 0x08  I2C Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |I2CDAT    |I2C Data Register
 * |        |          |Bit [7:0] is located with the 8-bit transferred data of I2C serial port.
 * @var I2C_T::I2CSTATUS
 * Offset: 0x0C  I2C Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |I2CSTATUS |I2C Status Register
 * |        |          |The status register of I2C:
 * |        |          |The three least significant bits are always 0.
 * |        |          |The five most significant bits contain the status code.
 * |        |          |There are 26 possible status codes.
 * |        |          |When I2CSTATUS contains F8H, no serial interrupt is requested.
 * |        |          |All other I2CSTATUS values correspond to defined I2C states.
 * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1).
 * |        |          |A valid status code is present in I2CSTATUS one cycle after SI is set by hardware and is still
 * |        |          |present one cycle after SI has been reset by software.
 * |        |          |In addition, states 00H stands for a Bus Error.
 * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the
 * |        |          |formation frame.
 * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an
 * |        |          |acknowledge bit.
 * @var I2C_T::I2CLK
 * Offset: 0x10  I2C Clock Divided Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |I2CLK     |I2C clock divided Register
 * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = (system clock) / (4x (I2CLK+1)).
 * |        |          |Note: The minimum value of I2CLK is 4.
 * @var I2C_T::I2CTOC
 * Offset: 0x14  I2C Time-out Counter Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TIF       |Time-out Flag
 * |        |          |This bit is set by H/W when I2C time-out happened and it can interrupt CPU if I2C interrupt
 * |        |          |enable bit (EI) is set to 1.
 * |        |          |Note: Write 1 to clear this bit.
 * |[1]     |DIV4      |Time-out Counter Input Clock Divided by 4
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |When Enabled, The time-out period is extend 4 times.
 * |[2]     |ENTI      |Time-out Counter Enable/Disable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |When Enabled, the 14-bit time-out counter will start counting when SI is clear.
 * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
 * @var I2C_T::I2CADDR1
 * Offset: 0x18  I2C Slave Address Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = General Call Function Disabled.
 * |        |          |1 = General Call Function Enabled.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in Master mode.
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
 * |        |          |The I2C hardware will react if either of the address is matched.
 * @var I2C_T::I2CADDR2
 * Offset: 0x1C  I2C Slave Address Register2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = General Call Function Disabled.
 * |        |          |1 = General Call Function Enabled.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in Master mode.
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
 * |        |          |The I2C hardware will react if either of the address is matched.
 * @var I2C_T::I2CADDR3
 * Offset: 0x20  I2C Slave Address Register3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = General Call Function Disabled.
 * |        |          |1 = General Call Function Enabled.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in Master mode.
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
 * |        |          |The I2C hardware will react if either of the address is matched.
 * @var I2C_T::I2CADM0
 * Offset: 0x24  I2C Slave Address Mask Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2CADM    |I2C Address Mask Register
 * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address
 * |        |          |register.).
 * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received corresponding
 * |        |          |address bit is don't-care.
 * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact
 * |        |          |the same as address register.
 * @var I2C_T::I2CADM1
 * Offset: 0x28  I2C Slave Address Mask Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2CADM    |I2C Address Mask Register
 * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address
 * |        |          |register.).
 * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received corresponding
 * |        |          |address bit is don't-care.
 * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact
 * |        |          |the same as address register.
 * @var I2C_T::I2CADM2
 * Offset: 0x2C  I2C Slave Address Mask Register2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2CADM    |I2C Address Mask Register
 * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address
 * |        |          |register.).
 * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received corresponding
 * |        |          |address bit is don't-care.
 * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact
 * |        |          |the same as address register.
 * @var I2C_T::I2CADM3
 * Offset: 0x30  I2C Slave Address Mask Register3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2CADM    |I2C Address Mask Register
 * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address
 * |        |          |register.).
 * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received corresponding
 * |        |          |address bit is don't-care.
 * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact
 * |        |          |the same as address register.
 * @var I2C_T::I2CWKUPCON
 * Offset: 0x3C  I2C Wake-up Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WKUPEN    |I2C Wake-up Enable
 * |        |          |0 = I2C wake-up function Disabled.
 * |        |          |1= I2C wake-up function Enabled.
 * @var I2C_T::I2CWKUPSTS
 * Offset: 0x40  I2C Wake-up Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WKUPIF    |I2C Wake-up Flag
 * |        |          |When chip is woken up from Power-down mode by I2C, this bit is set to 1.
 * |        |          |Software can write 1 to clear this bit.
 */

    __IO uint32_t I2CON;         /* Offset: 0x00  I2C Control Register                                               */
    __IO uint32_t I2CADDR0;      /* Offset: 0x04  I2C Slave Address Register0                                        */
    __IO uint32_t I2CDAT;        /* Offset: 0x08  I2C Data Register                                                  */
    __I  uint32_t I2CSTATUS;     /* Offset: 0x0C  I2C Status Register                                                */
    __IO uint32_t I2CLK;         /* Offset: 0x10  I2C Clock Divided Register                                         */
    __IO uint32_t I2CTOC;        /* Offset: 0x14  I2C Time-out Counter Register                                      */
    __IO uint32_t I2CADDR1;      /* Offset: 0x18  I2C Slave Address Register1                                        */
    __IO uint32_t I2CADDR2;      /* Offset: 0x1C  I2C Slave Address Register2                                        */
    __IO uint32_t I2CADDR3;      /* Offset: 0x20  I2C Slave Address Register3                                        */
    __IO uint32_t I2CADM0;       /* Offset: 0x24  I2C Slave Address Mask Register0                                   */
    __IO uint32_t I2CADM1;       /* Offset: 0x28  I2C Slave Address Mask Register1                                   */
    __IO uint32_t I2CADM2;       /* Offset: 0x2C  I2C Slave Address Mask Register2                                   */
    __IO uint32_t I2CADM3;       /* Offset: 0x30  I2C Slave Address Mask Register3                                   */
    __I  uint32_t RESERVED0[2]; 
    __IO uint32_t I2CWKUPCON;    /* Offset: 0x3C  I2C Wake-up Control Register                                       */
    __IO uint32_t I2CWKUPSTS;    /* Offset: 0x40  I2C Wake-up Status Register                                        */

} I2C_T;


/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */



/* I2C I2CON Bit Field Definitions */
#define I2C_I2CON_EI_Pos                        7                                       /*!< I2C_T::I2CON: EI Position */
#define I2C_I2CON_EI_Msk                        (1ul << I2C_I2CON_EI_Pos)               /*!< I2C_T::I2CON: EI Mask */

#define I2C_I2CON_ENS1_Pos                      6                                       /*!< I2C_T::I2CON: ENS1 Position */
#define I2C_I2CON_ENS1_Msk                      (1ul << I2C_I2CON_ENS1_Pos)             /*!< I2C_T::I2CON: ENS1 Mask */

#define I2C_I2CON_STA_Pos                       5                                       /*!< I2C_T::I2CON: STA Position */
#define I2C_I2CON_STA_Msk                       (1ul << I2C_I2CON_STA_Pos)              /*!< I2C_T::I2CON: STA Mask */

#define I2C_I2CON_STO_Pos                       4                                       /*!< I2C_T::I2CON: STO Position */
#define I2C_I2CON_STO_Msk                       (1ul << I2C_I2CON_STO_Pos)              /*!< I2C_T::I2CON: STO Mask */

#define I2C_I2CON_SI_Pos                        3                                       /*!< I2C_T::I2CON: SI Position */
#define I2C_I2CON_SI_Msk                        (1ul << I2C_I2CON_SI_Pos)               /*!< I2C_T::I2CON: SI Mask */

#define I2C_I2CON_AA_Pos                        2                                       /*!< I2C_T::I2CON: AA Position */
#define I2C_I2CON_AA_Msk                        (1ul << I2C_I2CON_AA_Pos)               /*!< I2C_T::I2CON: AA Mask */

/* I2C I2CADDR Bit Field Definitions */
#define I2C_I2CADDR_I2CADDR_Pos                 1                                       /*!< I2C_T::I2CADDR1: I2CADDR Position */
#define I2C_I2CADDR_I2CADDR_Msk                 (0x7Ful << I2C_I2CADDR_I2CADDR_Pos)     /*!< I2C_T::I2CADDR1: I2CADDR Mask */

#define I2C_I2CADDR_GC_Pos                      0                                       /*!< I2C_T::I2CADDR1: GC Position */
#define I2C_I2CADDR_GC_Msk                      (1ul << I2C_I2CADDR_GC_Pos)             /*!< I2C_T::I2CADDR1: GC Mask */

/* I2C I2CDAT Bit Field Definitions */
#define I2C_I2CDAT_I2CDAT_Pos                   0                                       /*!< I2C_T::I2CDAT: I2CDAT Position */
#define I2C_I2CDAT_I2CDAT_Msk                   (0xFFul << I2C_I2CDAT_I2CDAT_Pos)       /*!< I2C_T::I2CDAT: I2CDAT Mask */

/* I2C I2CSTATUS Bit Field Definitions */
#define I2C_I2CSTATUS_I2CSTATUS_Pos             0                                       /*!< I2C_T::I2CSTATUS: I2CSTATUS Position */
#define I2C_I2CSTATUS_I2CSTATUS_Msk             (0xFFul << I2C_I2CSTATUS_I2CSTATUS_Pos) /*!< I2C_T::I2CSTATUS: I2CSTATUS Mask */

/* I2C I2CLK Bit Field Definitions */
#define I2C_I2CLK_I2CLK_Pos                     0                                       /*!< I2C_T::I2CLK: I2CLK Position */
#define I2C_I2CLK_I2CLK_Msk                     (0xFFul << I2C_I2CLK_I2CLK_Pos)         /*!< I2C_T::I2CLK: I2CLK Mask */

/* I2C I2CTOC Bit Field Definitions */
#define I2C_I2CTOC_ENTI_Pos                     2                                       /*!< I2C_T::I2CTOC: ENTI Position */
#define I2C_I2CTOC_ENTI_Msk                     (1ul << I2C_I2CTOC_ENTI_Pos)            /*!< I2C_T::I2CTOC: ENTI Mask */

#define I2C_I2CTOC_DIV4_Pos                     1                                       /*!< I2C_T::I2CTOC: DIV4 Position */
#define I2C_I2CTOC_DIV4_Msk                     (1ul << I2C_I2CTOC_DIV4_Pos)            /*!< I2C_T::I2CTOC: DIV4 Mask */

#define I2C_I2CTOC_TIF_Pos                      0                                       /*!< I2C_T::I2CTOC: TIF Position */
#define I2C_I2CTOC_TIF_Msk                      (1ul << I2C_I2CTOC_TIF_Pos)             /*!< I2C_T::I2CTOC: TIF Mask */

/* I2C I2CADM Bit Field Definitions */
#define I2C_I2CADM_I2CADM_Pos                   1                                       /*!< I2C_T::I2CADM0: I2CADM Position */
#define I2C_I2CADM_I2CADM_Msk                   (0x7Ful << I2C_I2CADM_I2CADM_Pos)       /*!< I2C_T::I2CADM0: I2CADM Mask */

/* I2C I2CWKUPCON Bit Field Definitions */
#define I2C_I2CWKUPCON_WKUPEN_Pos               0                                       /*!< I2C_T::I2CWKUPCON: WKUPEN Position */
#define I2C_I2CWKUPCON_WKUPEN_Msk               (1ul << I2C_I2CWKUPCON_WKUPEN_Pos)      /*!< I2C_T::I2CWKUPCON: WKUPEN Mask */

/* I2C I2CWKUPSTS Bit Field Definitions */
#define I2C_I2CWKUPSTS_WKUPIF_Pos               0                                       /*!< I2C_T::I2CWKUPSTS: WKUPIF Position */
#define I2C_I2CWKUPSTS_WKUPIF_Msk               (1ul << I2C_I2CWKUPSTS_WKUPIF_Pos)      /*!< I2C_T::I2CWKUPSTS: WKUPIF Mask */
/*@}*/ /* end of group I2C_CONST */
/*@}*/ /* end of group I2C */
/**@}*/ /* end of REGISTER group */


#endif /* __I2C_REG_H__ */
