/**************************************************************************//**
 * @file     NuEdu-Basic01_I2C_EEPROM.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/16 9:27a $
 * @brief    NuEdu-Basic01_I2C_EEPROM driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_I2C_EEPROM.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS I2C EEPROM Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS
/*---------------------------------------------------------------------------------------------------------*/
/*  Definitons                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_EEPROM I2C1
#define I2C_EEPROM_IRQn I2C1_IRQn

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr = 0x50;
uint8_t g_au8TxData[3];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;
typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2CHandlerFn = NULL;
/// @endcond

/**
 * @brief       Open GPIO port for I2C interface and enable this I2C controller clock and reset it.
 *
 * @return      None
 */
__INLINE void I2C_PIN_Init(void)
{
    /* Set GPA10,11 multi-function pins for I2C1 SDA and SCL */
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA10_MFP_I2C1_SDA | SYS_PA_H_MFP_PA11_MFP_I2C1_SCL);

    /* Enable I2C1 clock */
    CLK->APBCLK |= CLK_APBCLK_I2C1_EN;

    /* Reset I2C1 */
    SYS->IPRST_CTL2 |=  SYS_IPRST_CTL2_I2C1_RST_Msk;
    SYS->IPRST_CTL2 &= ~SYS_IPRST_CTL2_I2C1_RST_Msk;
}

/**
 * @brief       I2C interrupt handler.
 *              Checks the I2C interrupt flag, clears the corresponding event flag and calls the related handler subroutine.
 *
 * @return      None
 */
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    // clear interrupt flag
    I2C1->INTSTS = I2C_INTSTS_INTSTS_Msk;

    u32Status = I2C_EEPROM->STATUS;

    if (I2C_EEPROM->INTSTS & I2C_INTSTS_TIF_Msk) {
        /* Clear I2C Timeout Flag */
        I2C_EEPROM->INTSTS = I2C_INTSTS_TIF_Msk;
    } else {
        if (s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/**
 * @brief       This function initializes the I2C module, bit-rate = 100 kHz and enable the corresponding interrupt.
 *
 * @param[in]   u8Divider   A divider of I2C clock source.
 *
 * @return      None
 */
void I2C_EEPROM_Init(uint8_t u8Divider)
{
    I2C_PIN_Init();
    /* Enable I2C Controller */
    I2C_EEPROM->CON |= I2C_CON_IPEN_Msk;

    /* I2C clock divider, I2C Bus Clock = PCLK / (4*120) = 100kHz */
    I2C_EEPROM->DIV = u8Divider;

    /* Enable I2C interrupt and set corresponding NVIC bit */
    I2C_EEPROM->CON |= I2C_CON_INTEN_Msk;
    NVIC_EnableIRQ(I2C_EEPROM_IRQn);
}

/**
 * @brief       This function checks the status of I2C, sets the related control bit and data if needed
 *      when this I2C module is master receiver.
 *
 * @param[in]   u32Status   The current value of I2C status register.
 *
 * @return      None
 */
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C_EEPROM->DATA = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_EEPROM->DATA = g_au8TxData[g_u8DataLen++];
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 2) {
            I2C_EEPROM->DATA = g_au8TxData[g_u8DataLen++];
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STA | I2C_SI);
        }
    } else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C_EEPROM->DATA = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
    } else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
    } else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_EEPROM->DATA;
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STO | I2C_SI);
        g_u8EndFlag = 1;
    } else if (u32Status == 0xF8) { /*I2C wave keeps going*/

    } else {
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/**
 * @brief       This function checks the status of I2C, sets the related control bit and data if needed
 *      when this I2C module is master transmitter.
 *
 * @param[in]   u32Status   The current value of I2C status register.
 *
 * @return      None
 */
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted */
        I2C_EEPROM->DATA = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_EEPROM->DATA = g_au8TxData[g_u8DataLen++];
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 3) {
            I2C_EEPROM->DATA = g_au8TxData[g_u8DataLen++];
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    } else if (u32Status == 0xF8) { /*I2C wave keeps going*/

    } else {
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/**
 * @brief       This function do the I2C data writing to EEPROM device.
 *
 * @param[in]   u16Address  An address of EEPROM that will be written.
 *
 * @param[in]   u18Data     The data will be written to EEPROM.
 *
 * @return      None
 */
void I2C_EEPROM_Write(uint16_t u16Address, uint8_t u8Data)
{
    g_au8TxData[0] = u16Address >> 8;
    g_au8TxData[1] = u16Address & 0xFF;
    g_au8TxData[2] = u8Data;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STA);

    /* Wait I2C Tx Finish */
    while (g_u8EndFlag == 0);
}

/**
 * @brief       This function do the I2C data reading from EEPROM device.
 *
 * @param[in]   u16Address  An address of EEPROM that will be read.
 *
 * @return      The data be read out.
 */
uint8_t I2C_EEPROM_Read(uint16_t u16Address)
{
    g_au8TxData[0] = u16Address >> 8;
    g_au8TxData[1] = u16Address & 0xFF;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_STA);

    /* Wait I2C Tx Finish */
    while (g_u8EndFlag == 0);

    return g_u8RxData;
}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



