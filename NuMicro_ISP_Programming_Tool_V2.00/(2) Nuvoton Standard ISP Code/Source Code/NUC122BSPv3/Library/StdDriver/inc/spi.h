/**************************************************************************//**
 * @file     spi.h
 * @version  V3.0
 * $Revision: 7 $
 * $Date: 15/07/24 5:39p $
 * @brief    SPI Driver Header File
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPI_Driver SPI Driver
  @{
*/

/** @addtogroup SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0        (SPI_CNTRL_TX_NEG_Msk)                          /*!< CLKP=0; RX_NEG=0; TX_NEG=1 */
#define SPI_MODE_1        (SPI_CNTRL_RX_NEG_Msk)                          /*!< CLKP=0; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_2        (SPI_CNTRL_CLKP_Msk | SPI_CNTRL_RX_NEG_Msk)     /*!< CLKP=1; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_3        (SPI_CNTRL_CLKP_Msk | SPI_CNTRL_TX_NEG_Msk)     /*!< CLKP=1; RX_NEG=0; TX_NEG=1 */

#define SPI_SLAVE         (SPI_CNTRL_SLAVE_Msk)                           /*!< Set as slave */
#define SPI_MASTER        (0x0)                                           /*!< Set as master */

#define SPI_SS0               (1<<SPI_SSR_SSR_Pos)                        /*!< Select SPIn_SS0 */
#define SPI_SS1               (2<<SPI_SSR_SSR_Pos)                        /*!< Select SPIn_SS1 */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSR_SS_LVL_Msk)                        /*!< SS active high */
#define SPI_SS_ACTIVE_LOW     (0x0)                                       /*!< SS active low */

#define SPI_UNIT_INT_MASK     (0x01)                                      /*!< Unit transfer interrupt mask */

#define SPI_BUSY_MASK         (0x01)                                      /*!< Busy status mask */

/*@}*/ /* end of group SPI_EXPORTED_CONSTANTS */


/** @addtogroup SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief      Clear the unit transfer interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to IF bit of SPI_CNTRL register to clear the unit transfer interrupt flag.
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi)   ((spi)->CNTRL |= SPI_CNTRL_IF_Msk)

/**
  * @brief      Get the datum read from RX0 register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     Data in RX0 register.
  * @details    Read SPI_RX0 register to get the received datum.
  */
#define SPI_READ_RX0(spi)   ((spi)->RX[0])

/**
  * @brief      Get the datum read from RX1 register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     Data in RX1 register.
  * @details    Read SPI_RX1 register to get the received datum.
  */
#define SPI_READ_RX1(spi)   ((spi)->RX[1])

/**
  * @brief      Write datum to TX0 register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TxData The datum which user attempt to transfer through SPI bus.
  * @return     None.
  * @details    Write u32TxData to TX0 register.
  */
#define SPI_WRITE_TX0(spi, u32TxData)   ((spi)->TX[0] = (u32TxData))

/**
  * @brief      Write datum to TX1 register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TxData The datum which user attempt to transfer through SPI bus.
  * @return     None.
  * @details    Write u32TxData to TX1 register.
  */
#define SPI_WRITE_TX1(spi, u32TxData)   ((spi)->TX[1] = (u32TxData))

/**
  * @brief      Set SPIn_SS0, SPIn_SS1 pin to high or low state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  ss0 0 = Set SPIn_SS0 to low. 1 = Set SPIn_SS0 to high.
  * @param[in]  ss1 0 = Set SPIn_SS1 to low. 1 = Set SPIn_SS1 to high.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS0/SPIn_SS1 pin to specified high/low state.
  *             Only available in Master mode.
  */
#define SPI_SET_SS_LEVEL(spi, ss0, ss1)   ((spi)->SSR = ((spi)->SSR & ~(SPI_SSR_AUTOSS_Msk|SPI_SSR_SS_LVL_Msk|SPI_SSR_SSR_Msk)) | (((ss1)^1) << 1) | ((ss0)^1))

/**
  * @brief      Set SPIn_SS0 pin to high state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS0 pin to high state. Only available in Master mode.
  */
#define SPI_SET_SS0_HIGH(spi)   ((spi)->SSR = ((spi)->SSR & ~(SPI_SSR_AUTOSS_Msk|SPI_SSR_SS_LVL_Msk|SPI_SS0)))

/**
  * @brief      Set SPIn_SS1 pin to high state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS1 pin to high state. Only available in Master mode.
  */
#define SPI_SET_SS1_HIGH(spi)   ((spi)->SSR = ((spi)->SSR & ~(SPI_SSR_AUTOSS_Msk|SPI_SSR_SS_LVL_Msk|SPI_SS1)))

/**
  * @brief      Set SPIn_SS0 pin to low state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS0 pin to low state. Only available in Master mode.
  */
#define SPI_SET_SS0_LOW(spi)   ((spi)->SSR = ((spi)->SSR & ~(SPI_SSR_AUTOSS_Msk|SPI_SSR_SS_LVL_Msk|SPI_SS0)) | SPI_SS0)

/**
  * @brief      Set SPIn_SS1 pin to low state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS1 pin to low state. Only available in Master mode.
  */
#define SPI_SET_SS1_LOW(spi)   ((spi)->SSR = ((spi)->SSR & ~(SPI_SSR_AUTOSS_Msk|SPI_SSR_SS_LVL_Msk|SPI_SS1)) | SPI_SS1)

/**
  * @brief      Enable Byte Reorder function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None
  * @details    Set REORDER bit of SPI_CNTRL register to enable Byte Reorder function. The Byte Suspend function is enabled as well.
  *             The setting of TX_BIT_LEN must be configured as 0x00 (32 bits/word).
  *             The period of the byte suspend interval depends on the setting of SP_CYCLE.
  */
#define SPI_ENABLE_BYTE_REORDER(spi)   ((spi)->CNTRL = ((spi)->CNTRL & (~SPI_CNTRL_REORDER_Msk)) | (1<<SPI_CNTRL_REORDER_Pos))

/**
  * @brief      Disable Byte Reorder function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear REORDER bit of SPI_CNTRL register to disable Byte Reorder function.
  */
#define SPI_DISABLE_BYTE_REORDER(spi)   ((spi)->CNTRL &= ~SPI_CNTRL_REORDER_Msk)

/**
  * @brief      Set the length of suspend interval.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32SuspCycle Decides the length of suspend interval. It could be 0 ~ 15.
  * @return     None.
  * @details    Set the length of suspend interval according to u32SuspCycle.
  *             The length of suspend interval is ((u32SuspCycle + 2) * the length of one SPI bus clock cycle).
  *             Only available in Master mode.
  */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle)   ((spi)->CNTRL = ((spi)->CNTRL & ~SPI_CNTRL_SP_CYCLE_Msk) | ((u32SuspCycle) << SPI_CNTRL_SP_CYCLE_Pos))

/**
  * @brief      Set the SPI transfer sequence with LSB first.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set LSB bit of SPI_CNTRL register to set the SPI transfer sequence with LSB first.
  */
#define SPI_SET_LSB_FIRST(spi)   ((spi)->CNTRL |= SPI_CNTRL_LSB_Msk)

/**
  * @brief      Set the SPI transfer sequence with MSB first.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear LSB bit of SPI_CNTRL register to set the SPI transfer sequence with MSB first.
  */
#define SPI_SET_MSB_FIRST(spi)   ((spi)->CNTRL &= ~SPI_CNTRL_LSB_Msk)

/**
  * @brief      Set the data width of a SPI transaction.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Width The bit width of transfer data.
  * @return     None.
  * @details    The data width can be 8 ~ 32 bits.
  */
#define SPI_SET_DATA_WIDTH(spi, u32Width)   ((spi)->CNTRL = ((spi)->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (((u32Width)&0x1F) << SPI_CNTRL_TX_BIT_LEN_Pos))

/**
  * @brief      Get the SPI busy state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 SPI controller is not busy.
  * @retval     1 SPI controller is busy.
  * @details    This macro will return the busy state of SPI controller.
  */
#define SPI_IS_BUSY(spi)   ( ((spi)->CNTRL & SPI_CNTRL_GO_BUSY_Msk)>>SPI_CNTRL_GO_BUSY_Pos )

/**
  * @brief      Set the GO_BUSY bit to trigger SPI transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    User can use this macro to trigger the data transfer after all configuration is ready.
  */
#define SPI_TRIGGER(spi)   ((spi)->CNTRL |= SPI_CNTRL_GO_BUSY_Msk)



/* Function prototype declaration */
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);



/**
  * @} End of SPI Device Function Interface
  */

/**
  * @} End of Function Interface
  */

/**
  * @} End of Device_Driver
  */


#endif

