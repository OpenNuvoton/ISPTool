/**************************************************************************//**
 * @file        spim.h
 * @version     V1.00
 * $Revision:   1$
 * $Date:       15/07/10 5:00p$
 * @brief       ISD9000 series SPIM driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SPIM_H__
#define __SPIM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_SPIM_Driver SPIM Driver
  @{
*/

/** @addtogroup ISD9000_SPIM_EXPORTED_CONSTANTS SPIM Exported Constants
  @{
*/
	
#define SPIM_SS_ACTIVE_HIGH    (SPIM_CTL1_SSACTPOL_Msk)                /*!< unset any spi port \hideinitializer */
#define SPIM_SS_ACTIVE_LOW     (0)                                     /*!< Set SS0 \hideinitializer */
	
#define SPIM_BITMODE_STAN      (0UL << SPIM_CTL0_BITMODE_Pos)          /*!< Standard mode (SPI Interface including DO, DI, HOLD, WP). \hideinitializer */
#define SPIM_BITMODE_DUAL      (1UL << SPIM_CTL0_BITMODE_Pos)          /*!< Dual mode (SPI Interface including D0, D1, HOLD, WP). \hideinitializer */
#define SPIM_BITMODE_QUAD      (2UL << SPIM_CTL0_BITMODE_Pos)          /*!< Quad mode (SPI Interface including D0, D1, D2, D3). \hideinitializer */
	
/* SPIM Interrupt Mask */
#define SPIM_INT_MASK               (0x001)                                 /*!< Interrupt mask. \hideinitializer */


/*@}*/ /* end of group ISD9000_SPIM_EXPORTED_CONSTANTS */

/** @addtogroup ISD9000_SPIM_EXPORTED_FUNCTIONS SPIM Exported Functions
  @{
*/

/**
  * @brief                  Set bit mode for I/O mode.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32BitMode  Bit mode. Valid values include:
  *                         - \ref SPIM_CTL0_BITMODE_STAN
  *                         - \ref SPIM_CTL0_BITMODE_DUAL
  *                         - \ref SPIM_CTL0_BITMODE_QUAD
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_SET_BIT_MODE(spim, u32BitMode) ((spim)->CTL0 = ((spim)->CTL0 & (~(SPIM_CTL0_BITMODE_Msk))) | (u32BitMode))

/**
  * @brief                  Set direction for Quad/Dual I/O mode.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32Dir      Direction. Valid values can be 0 (for input) and 1 (for output).
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_SET_QDIODIR(spim, u32Dir) ((spim)->CTL0 = ((spim)->CTL0 & (~(SPIM_CTL0_QDIODIR_Msk))) | ((u32Dir) << SPIM_CTL0_QDIODIR_Pos))   

/**
  * @brief                      Set the length of suspend interval.
  * @param[in]  spim            Base address of SPIM module.
  * @param[in]  u32SuspCycle    Decides the length of suspend interval which ranges between 0 and 15.
  * @return                     None.
  * \hideinitializer
  */
#define SPIM_SET_SUSPEND_CYCLE(spim, u32SuspCycle) ((spim)->CTL0 = ((spim)->CTL0 & (~SPIM_CTL0_SLEEP_Msk)) | ((u32SuspCycle) << SPIM_CTL0_SLEEP_Pos))

/**
  * @brief                      Set the number of successive transmit/receive transactions in one transfer.
  * @param[in]  spim            Base address of SPIM module.
  * @param[in]  u32BurstNum     Decides the transmit/receive number in one transfer which ranges between 1 and 4.
  * @return                     None.
  * \hideinitializer
  */
#define SPIM_SET_BURST_NUM(spim, u32BurstNum) ((spim)->CTL0 = ((spim)->CTL0 & (~SPIM_CTL0_BURSTNUM_Msk)) | (((u32BurstNum) - 1) << SPIM_CTL0_BURSTNUM_Pos))

/**
  * @brief                  Set the data width of a transmit/receive transaction.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32Width    Data width: 8, 16, 24, 32 bits
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_SET_DATA_WIDTH(spim, u32Width) ((spim)->CTL0 = ((spim)->CTL0 & (~SPIM_CTL0_DWIDTH_Msk)) | (((u32Width) - 1) << SPIM_CTL0_DWIDTH_Pos))

/**
  * @brief              
  * @param[in]  spim    
  * @return             
  * \hideinitializer
  */
#define SPIM_ACTIVE_SS(spim)                ((spim)->CTL1 &= (~SPIM_CTL1_SS_Msk))  

/**
  * @brief              
  * @param[in]  spim   
  * @return             
  * \hideinitializer
  */
#define SPIM_INACTIVE_SS(spim)              ((spim)->CTL1 |= SPIM_CTL1_SS_Msk)                                                                \

/**
  * @brief              
  * @param[in]  spim    
  * @return             
  * \hideinitializer
  */
#define SPIM_SET_SS_LEVEL(spim,u32Level)    ((spim)->CTL1 = ((spim)->CTL1 & (~SPIM_CTL1_SSACTPOL_Msk))|u32Level ) 

/**
  * @brief              Check if SPIM module is busy.
  * @param[in]  spim    Base 	address of SPIM module.
  * @retval 0           Not busy.
  * @retval 1           Busy.
  * \hideinitializer
  */
#define SPIM_IS_BUSY(spim) ((spim->CTL1 & SPIM_CTL1_SPIMEN_Msk) == SPIM_CTL1_SPIMEN_Msk ? 1:0 )

/**
  * @brief              Trigger SPI transfer.
  * @param[in]  spim    Base address of SPIM module.
  * @return             None.
  * \hideinitializer
  */
#define SPIM_TRIGGER(spim) ((spim)->CTL1 |= ((spim)->CTL1 & (~SPIM_CTL1_SPIMEN_Msk)) | SPIM_CTL1_SPIMEN_Msk)

/**
  * @brief                  Write datum to TX0 register.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32TxData   Data which user attempts to transfer through SPI bus.
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_WRITE_TX0(spim, u32TxData) ((spim)->TX0 = u32TxData)

/**
  * @brief                  Write datum to TX1 register.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32TxData   Data which user attempts to transfer through SPI bus.
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_WRITE_TX1(spim, u32TxData) ((spim)->TX1 = u32TxData)

/**
  * @brief                  Write datum to TX2 register.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32TxData   Data which user attempts to transfer through SPI bus.
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_WRITE_TX2(spim, u32TxData) ((spim)->TX2 = u32TxData)

/**
  * @brief                  Write datum to TX3 register.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32TxData   Data which user attempts to transfer through SPI bus.
  * @return                 None.
  * \hideinitializer
  */
#define SPIM_WRITE_TX3(spim, u32TxData) ((spim)->TX3 = u32TxData)

/**
  * @brief                  Get the datum read from RX0 register.
  * @param[in]  spim        Base address of SPIM module.
  * @return                 Datum read from RX0 register.
  * \hideinitializer
  */
#define SPIM_READ_RX0(spim) ((spim)->RX0)

/**
  * @brief                  Get the datum read from RX1 register.
  * @param[in]  spim        Base address of SPIM module.
  * @return                 Datum read from RX1 register.
  * \hideinitializer
  */
#define SPIM_READ_RX1(spim) ((spim)->RX1)

/**
  * @brief                  Get the datum read from RX2 register.
  * @param[in]  spim        Base address of SPIM module.
  * @return                 Datum read from RX2 register.
  * \hideinitializer
  */
#define SPIM_READ_RX2(spim) ((spim)->RX2)

/**
  * @brief                  Get the datum read from RX3 register.
  * @param[in]  spim        Base address of SPIM module.
  * @return                 Datum read from RX3 register.
  * \hideinitializer
  */
#define SPIM_READ_RX3(spim) ((spim)->RX3)

uint32_t SPIM_Open(SPIM_T *spim, uint32_t u32BusClock);
void SPIM_Close(SPIM_T *spim);

uint32_t SPIM_SetBusClock(SPIM_T *spim, uint32_t u32BusClock);
uint32_t SPIM_GetBusClock(SPIM_T *spim);

void SPIM_EnableInt(SPIM_T *spim, uint32_t u32Mask);
void SPIM_DisableInt(SPIM_T *spim, uint32_t u32Mask);

uint32_t SPIM_GetIntFlag(SPIM_T *spim, uint32_t u32Mask);
void SPIM_ClearIntFlag(SPIM_T *spim, uint32_t u32Mask);

/*@}*/ /* end of group ISD9000_SPIM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_SPIM_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPIM_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
