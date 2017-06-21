/**************************************************************************//**
 * @file     i2c.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 15/05/26 4:27p $
 * @brief    MINI51 series I2C driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini51Series.h"

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_I2C_Driver I2C Driver
  @{
*/


/** @addtogroup MINI51_I2C_EXPORTED_FUNCTIONS I2C Exported Functions
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
    i2c->I2CLK = u32Div;

    /* Enable I2C */
    i2c->I2CON |= I2C_I2CON_ENSI_Msk;

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
    SYS->IPRSTC2 |= SYS_IPRSTC2_I2C_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C_RST_Msk;

    /* Disable I2C */
    i2c->I2CON &= ~I2C_I2CON_ENSI_Msk;
}

/**
  * @brief This function clears the timeout flag.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_ClearTimeoutFlag(I2C_T *i2c)
{
    i2c->I2CTOC |= I2C_I2CTOC_TIF_Msk;
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

    if (u8Start)
        u32Reg |= I2C_STA;
    if (u8Stop)
        u32Reg |= I2C_STO;
    if (u8Si)
        u32Reg |= I2C_SI;
    if (u8Ack)
        u32Reg |= I2C_AA;

    i2c->I2CON = (i2c->I2CON & ~0x3C) | u32Reg;
}

/**
  * @brief This function disables the interrupt (EI bit) of I2C module.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_DisableInt(I2C_T *i2c)
{
    i2c->I2CON &= ~I2C_I2CON_EI_Msk;
}

/**
  * @brief This function enables the interrupt (EI bit) of I2C module.
  * @param i2c is the base address of I2C module.
  * @return none
  */
void I2C_EnableInt(I2C_T *i2c)
{
    i2c->I2CON |= I2C_I2CON_EI_Msk;
}

/**
  * @brief This function returns the real bus clock of I2C module.
  * @param i2c is the base address of I2C module.
  * @return Actual I2C bus clock frequency.
  */
uint32_t I2C_GetBusClockFreq(I2C_T *i2c)
{
    uint32_t u32Divider = i2c->I2CLK;

    return ( SystemCoreClock / ((u32Divider+1)<<2) );
}

/**
  * @brief This function enables the interrupt (EI bit) of I2C module.
  * @param i2c is the base address of I2C module.
  * @param u32BusClock is the target bus speed of I2C module.
  * @return Actual I2C bus clock frequency.
  */
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32Div;

    u32Div = (uint32_t) (((SystemCoreClock * 10)/(u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    i2c->I2CLK = u32Div;

    return ( SystemCoreClock / ((u32Div+1)<<2) );
}

/**
  * @brief This function gets the interrupt flag (SI bit) of I2C module.
  * @param i2c is the base address of I2C module.
  * @return Interrupt flag.
  * @retval 0 Flag is not set.
  * @retval 1 Flag is set.
  */
uint32_t I2C_GetIntFlag(I2C_T *i2c)
{
    return ( (i2c->I2CON & I2C_I2CON_SI_Msk) == I2C_I2CON_SI_Msk ? 1:0 );
}

/**
  * @brief This function returns the status of I2C module.
  * @param i2c is the base address of I2C module.
  * @return Status.
  */
uint32_t I2C_GetStatus(I2C_T *i2c)
{
    return ( i2c->I2CSTATUS );
}

/**
  * @brief This function returns the data stored in data register of I2C module.
  * @param i2c is the base address of I2C module.
  * @return Data.
  */
uint32_t I2C_GetData(I2C_T *i2c)
{
    return ( i2c->I2CDAT );
}

/**
  * @brief This function writes the data to data register of I2C module.
  * @param i2c is the base address of I2C module.
  * @param u8Data is the data which will be write to data register of I2C module.
  * @return none
  */
void I2C_SetData(I2C_T *i2c, uint8_t u8Data)
{
    i2c->I2CDAT = u8Data;
}

/**
  * @brief Configure slave address and enable GC mode.
  * @param i2c is the base address of I2C module.
  * @param u8SlaveNo is the set number of salve address.
  * @param u8SlaveAddr is the slave address.
  * @param u8GCMode enable GC mode.
  * @return none
  */
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode)
{
    switch (u8SlaveNo) {
    case 0:
        i2c->I2CADDR0  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    case 1:
        i2c->I2CADDR1  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    case 2:
        i2c->I2CADDR2  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    case 3:
        i2c->I2CADDR3  = (u8SlaveAddr << 1) | u8GCMode;
        break;
    default:
        i2c->I2CADDR0  = (u8SlaveAddr << 1) | u8GCMode;
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
        i2c->I2CADM0  = u8SlaveAddrMask << 1;
        break;
    case 1:
        i2c->I2CADM1  = u8SlaveAddrMask << 1;
        break;
    case 2:
        i2c->I2CADM2  = u8SlaveAddrMask << 1;
        break;
    case 3:
        i2c->I2CADM3  = u8SlaveAddrMask << 1;
        break;
    default:
        i2c->I2CADM0  = u8SlaveAddrMask << 1;
    }
}

/**
  * @brief This function enables timeout function and configures DIV4 function to support long timeout.
  * @param i2c is the base address of I2C module.
  * @param u8LongTimeout Enable timeout counter input clock is divide by 4.
  * @return none.
  */
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout)
{
    if(u8LongTimeout)
        i2c->I2CTOC |= I2C_I2CTOC_DIV4_Msk;
    else
        i2c->I2CTOC &= ~I2C_I2CTOC_DIV4_Msk;

    i2c->I2CTOC |= I2C_I2CTOC_ENTI_Msk;
}

/**
  * @brief This function disables timeout function.
  * @param i2c is the base address of I2C module.
  * @return none.
  */
void I2C_DisableTimeout(I2C_T *i2c)
{
    i2c->I2CTOC &= ~I2C_I2CTOC_ENTI_Msk;
}

/**
  * @brief This function enables the wakeup function of I2C module.
  * @param i2c is the base address of I2C module.
  * @return none.
  */
void I2C_EnableWakeup(I2C_T *i2c)
{
    i2c->I2CON2 |= I2C_I2CON2_WKUPEN_Msk;
}

/**
  * @brief This function disables the wakeup function of I2C module.
  * @param i2c is the base address of I2C module.
  * @return none.
  */
void I2C_DisableWakeup(I2C_T *i2c)
{
    i2c->I2CON2 &= ~I2C_I2CON2_WKUPEN_Msk;
}

/*@}*/ /* end of group MINI51_I2C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_I2C_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
