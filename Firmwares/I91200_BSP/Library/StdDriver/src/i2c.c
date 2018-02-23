/****************************************************************************//**
 * @file     i2c.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/03 2:52p $
 * @brief    I91200 I2C driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_I2C_Driver I2C Driver
  @{
*/

/** @addtogroup I91200_I2C_EXPORTED_FUNCTIONS I2C Exported Functions
  @{
*/

/**
  * @brief This function make I2C module be ready and set the wanted bus clock.
  * @param i2c is the base address of I2C module.
  * @param u32BusClock is the target bus speed of I2C module.
  * @return Actual I2C bus clock frequency.
  */
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32Div;

    u32Div = (uint32_t) (((SystemCoreClock * 10)/(u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    i2c->CLKDIV = u32Div;

    /* Enable I2C */
    i2c->CTL |= I2C_CTL_I2CEN_Msk;

    return ( SystemCoreClock / ((u32Div+1)<<2) );
}

/**
  * @brief  This function closes the I2C module.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_Close(I2C_T *i2c)
{
    /* Reset SPI */
    if((uint32_t)i2c == I2C0_BASE) {
        SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;
    }

    /* Disable I2C */
    i2c->CTL &= ~I2C_CTL_I2CEN_Msk;
}

/**
  * @brief This function clears the timeout flag.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_ClearTimeoutFlag(I2C_T *i2c)
{
    i2c->TOCTL |= I2C_TOCTL_TOIF_Msk;
}

/**
  * @brief This function sets the control bit of the I2C module.
  * @param i2c is the base address of I2C module.
  * @param u8Start sets START bit to I2C module.
  * @param u8Stop sets STOP bit to I2C module.
  * @param u8Si sets SI bit to I2C module.
  * @param u8Ack sets ACK bit to I2C module.
  * @return none
  */
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack)
{
    uint32_t u32Reg = 0;
    uint32_t u32Val = i2c->CTL & ~(I2C_CTL_STA_Msk | I2C_CTL_STO_Msk | I2C_CTL_AA_Msk|I2C_CTL_SI_Msk);

    if (u8Start)
        u32Reg |= I2C_CTL_STA_Msk;
    if (u8Stop)
        u32Reg |= I2C_CTL_STO_Msk;
    if (u8Si)
        u32Reg |= I2C_CTL_SI_Msk;
    if (u8Ack)
        u32Reg |= I2C_CTL_AA_Msk;

    i2c->CTL = u32Val | u32Reg;
}

/**
  * @brief This function disables the interrupt of I2C module.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_DisableInt(I2C_T *i2c)
{
    i2c->CTL &= ~I2C_CTL_INTEN_Msk;
}

/**
  * @brief This function enables the interrupt (EI bit) of I2C module.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_EnableInt(I2C_T *i2c)
{
    i2c->CTL |= I2C_CTL_INTEN_Msk;
}

/**
  * @brief This function returns the real bus clock of I2C module.
  * @param i2c is the base address of I2C module.
  * @return Actual I2C bus clock frequency.
  */
uint32_t I2C_GetBusClockFreq(I2C_T *i2c)
{
    uint32_t u32Divider = i2c->CLKDIV;

    return ( SystemCoreClock / ((u32Divider+1)<<2) );
}

/**
  * @brief This function sets bus frequency of I2C module.
  * @param i2c is the base address of I2C module.
  * @param u32BusClock is the target bus speed of I2C module.
  * @return Actual I2C bus clock frequency.
  */
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32Div;

    u32Div = (uint32_t) (((SystemCoreClock * 10)/(u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    i2c->CLKDIV = u32Div;

    return ( SystemCoreClock / ((u32Div+1)<<2) );
}

/**
  * @brief This function gets the interrupt flag of I2C module.
  * @param i2c is the base address of I2C module.
  * @return Interrupt flag.
  * @retval 0 Flag is not set.
  * @retval 1 Flag is set.
  */
uint32_t I2C_GetIntFlag(I2C_T *i2c)
{
    return ( (i2c->CTL & I2C_CTL_SI_Msk) == I2C_CTL_SI_Msk ? 1:0 );
}

/**
  * @brief This function returns the status of I2C module.
  * @param i2c is the base address of I2C module.
  * @return I2C status
  */
uint32_t I2C_GetStatus(I2C_T *i2c)
{
    return ( i2c->STATUS );
}

/**
  * @brief This function returns the data stored in data register of I2C module.
  * @param i2c is the base address of I2C module.
  * @return I2C data.
  */
uint32_t I2C_GetData(I2C_T *i2c)
{
    return ( i2c->DAT );
}

/**
  * @brief This function writes the data to data register of I2C module.
  * @param i2c is the base address of I2C module.
  * @param u8Data is the data which will be write to data register of I2C module.
  * @return none
  */
void I2C_SetData(I2C_T *i2c, uint8_t u8Data)
{
    i2c->DAT = u8Data;
}

/**
  * @brief Configure slave address and enable GC mode.
  * @param i2c is the base address of I2C module.
  * @param u8SlaveNo is the set number of salve address.
  * @param u8SlaveAddr is the slave address.
  * @param u8GCMode GC mode enable or not. Valid values are:
  *              - \ref I2C_GCMODE_ENABLE
  *              - \ref I2C_GCMODE_DISABLE
  * @return none
  */
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode)
{
    switch (u8SlaveNo) {
    case 0:
        i2c->ADDR0  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    case 1:
        i2c->ADDR1  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    case 2:
        i2c->ADDR2  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    case 3:
        i2c->ADDR3  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    default:
        i2c->ADDR0  = (u8SlaveAddr << 1) | u8GCMode;
    }
}

/**
  * @brief Configure the mask of slave address. The corresponding address bit is "Don't Care".
  * @param i2c is the base address of I2C module.
  * @param u8SlaveNo is the set number of salve address.
  * @param u8SlaveAddrMask is the slave address mask.
  * @return none
  */
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask)
{
    switch (u8SlaveNo) {
    case 0:
        i2c->ADDRMSK0  = u8SlaveAddrMask << 1;
        break;
    case 1:
        i2c->ADDRMSK1  = u8SlaveAddrMask << 1;
        break;
    case 2:
        i2c->ADDRMSK2  = u8SlaveAddrMask << 1;
        break;
    case 3:
        i2c->ADDRMSK3  = u8SlaveAddrMask << 1;
        break;
    default:
        i2c->ADDRMSK0  = u8SlaveAddrMask << 1;
    }
}

/**
  * @brief This function enables timeout function and configures DIV4 function to support long timeout.
  * @param i2c is the base address of I2C module.
  * @param u8LongTimeout Enable timeout counter input clock is divide by 4.
  * @return none
  */
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout)
{
    if(u8LongTimeout)
        i2c->TOCTL |= I2C_TOCTL_TOCDIV4_Msk;
    else
        i2c->TOCTL &= ~I2C_TOCTL_TOCDIV4_Msk;

    i2c->TOCTL |= I2C_TOCTL_TOCEN_Msk;
}

/**
  * @brief This function disables timeout function.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_DisableTimeout(I2C_T *i2c)
{
    i2c->TOCTL &= ~I2C_TOCTL_TOCEN_Msk;
}

/*@}*/ /* end of group I91200_I2C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_I2C_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
