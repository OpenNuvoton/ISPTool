/**************************************************************************//**
 * @file     PDMA.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/01/18 2:52p $
 * @brief    I91200 Series PDMA Controller Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __PDMA_H__
#define __PDMA_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  * @{
  */

/** @addtogroup I91200_PDMA_Driver PDMA Driver
  * @{
  */

/** @addtogroup I91200_PDMA_EXPORTED_CONSTANTS PDMA Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Data Width Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_WIDTH_8        0x00080000UL            /*!<DMA Transfer Width 8-bit */
#define PDMA_WIDTH_16       0x00100000UL            /*!<DMA Transfer Width 16-bit */
#define PDMA_WIDTH_32       0x00000000UL            /*!<DMA Transfer Width 32-bit */ 

/*---------------------------------------------------------------------------------------------------------*/
/*  Address Attribute Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SAR_INC        0x00000000UL            /*!<DMA SAR increment */
#define PDMA_SAR_FIX        0x00000020UL            /*!<DMA SAR fix address */
#define PDMA_SAR_WRA        0x00000030UL            /*!<DMA SAR wrap around */
#define PDMA_DAR_INC        0x00000000UL            /*!<DMA DAR increment */
#define PDMA_DAR_FIX        0x00000080UL            /*!<DMA DAR fix address */
#define PDMA_DAR_WRA        0x000000C0UL            /*!<DMA DAR wrap around */

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMA Transfer Direction Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SRAM_SRAM      0x00000000UL            /*!<DMA Transfer from SRAM to SRAM */
#define PDMA_APB_SRAM       0x00000004UL            /*!<DMA Transfer from APB to SRAM */
#define PDMA_SRAM_APB       0x00000008UL            /*!<DMA Transfer from SRAM to APB */

/*---------------------------------------------------------------------------------------------------------*/
/*  Peripheral Transfer Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SPI0_TX        0x00000000UL            /*!<DMA Connect to SPI0 TX */
#define PDMA_SPI0_RX        0x00000001UL            /*!<DMA Connect to SPI0 RX */
#define PDMA_I2S_TX         0x00000002UL            /*!<DMA Connect to I2S TX */
#define PDMA_I2S_RX         0x00000003UL            /*!<DMA Connect to I2S RX */
#define PDMA_UART0_TX       0x00000004UL            /*!<DMA Connect to UART0 TX */
#define PDMA_UART0_RX       0x00000005UL            /*!<DMA Connect to UART0 RX */
#define PDMA_SDADC          0x00000006UL            /*!<DMA Connect to SDADC */
#define PDMA_DPWM           0x00000007UL            /*!<DMA Connect to DPWM */
#define PDMA_SPI1_TX        0x00000008UL            /*!<DMA Connect to SPI1 TX */
#define PDMA_SPI1_RX        0x00000009UL            /*!<DMA Connect to SPI1 RX */
#define PDMA_UART1_TX       0x0000000AUL            /*!<DMA Connect to UART1 TX */
#define PDMA_UART1_RX       0x0000000BUL            /*!<DMA Connect to UART1 RX */
#define PDMA_SARADC         0x0000000CUL            /*!<DMA Connect to SARADC */
#define PDMA_MEM            0x000000ffUL            /*!<DMA Connect to Memory */

/*---------------------------------------------------------------------------------------------------------*/
/*  Wrap Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_HALF_WRAP_MODE        0x00000004UL            /*!<In this mode, the interrupt can be generated when PDMA transfer is half complete */
#define PDMA_FULL_WRAP_MODE        0x00000001UL            /*!<In this mode, the interrupt can be generated when PDMA transfer is warparound */
#define PDMA_BOTH_WRAP_MODE        0x00000005UL            /*!<Both half and warparound interrupts generated */

/*@}*/ /* end of group I91200_PDMA_EXPORTED_CONSTANTS */

/** @addtogroup I91200_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
 * @brief       Get PDMA Global Interrupt Status
 *
 * @return      Interrupt Status
 *
 * @details     This macro gets the global interrupt status.
 */
#define PDMA_GET_INT_STATUS()   ((uint32_t)(PDMAC->GINTSTS))

/**
 * @brief       Get PDMA Channel Interrupt Status
 *
 * @param[in]   u32Ch   Selected DMA channel
 *
 * @return      Interrupt Status
 *
 * @details     This macro gets the channel interrupt status.
 */
#define PDMA_GET_CH_INT_STS(u32Ch)   (*((__IO uint32_t *)((uint32_t)&PDMA0->INTSTS + (uint32_t)((u32Ch)*0x100))))

/**
 * @brief       Clear PDMA Channel Interrupt Flag
 *
 * @param[in]   u32Ch   Selected DMA channel
 * @param[in]   u32Mask Interrupt Mask
 *
 * @return      None
 *
 * @details     This macro clear the channel interrupt flag.
 */
#define PDMA_CLR_CH_INT_FLAG(u32Ch, u32Mask)   (*((__IO uint32_t *)((uint32_t)&PDMA0->INTSTS + (uint32_t)((u32Ch)*0x100))) = (u32Mask))

/**
 * @brief       Check Channel Status
 *
 * @param[in]   u32Ch    The selected channel
 *
 * @retval      0        The selected channel is idle
 * @retval      1        The selected channel is busy
 *
 * @details     Check the selected channel is busy or not.
 */
#define PDMA_IS_CH_BUSY(u32Ch)    ((*((__IO uint32_t *)((uint32_t)&PDMA0->CTL +(uint32_t)((u32Ch)*0x100)))&PDMA_CTL_TXEN_Msk)? 1 : 0)

/**
 * @brief       Set Source Address
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel source address.
 */
#define PDMA_SET_SRC_ADDR(u32Ch, u32Addr) (*((__IO uint32_t *)((uint32_t)&PDMA0->SADDR + (uint32_t)((u32Ch)*0x100))) = (u32Addr))

/**
 * @brief       Set Destination Address
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel destination address.
 */
#define PDMA_SET_DST_ADDR(u32Ch, u32Addr) (*((__IO uint32_t *)((uint32_t)&PDMA0->DADDR + (uint32_t)((u32Ch)*0x100))) = (u32Addr))

/**
 * @brief       Set Transfer Count
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Count  Transfer Count
 *
 * @return      None
 *
 * @details     This macro set the selected channel transfer count.
 */
#define PDMA_SET_TRANS_CNT(u32Ch, u32Count) {   \
    if (((uint32_t)*((__IO uint32_t *)((uint32_t)&PDMA0->CTL + (uint32_t)((u32Ch)*0x100))) & PDMA_CTL_TXWIDTH_Msk) == PDMA_WIDTH_32)  \
        *((__IO uint32_t *)((uint32_t)&PDMA0->TXCNT + (uint32_t)((u32Ch)*0x100))) = ((u32Count) << 2);  \
    else if (((uint32_t)*((__IO uint32_t *)((uint32_t)&PDMA0->CTL + (uint32_t)((u32Ch)*0x100))) & PDMA_CTL_TXWIDTH_Msk) == PDMA_WIDTH_8)  \
        *((__IO uint32_t *)((uint32_t)&PDMA0->TXCNT + (uint32_t)((u32Ch)*0x100))) = (u32Count); \
    else if (((uint32_t)*((__IO uint32_t *)((uint32_t)&PDMA0->CTL + (uint32_t)((u32Ch)*0x100))) & PDMA_CTL_TXWIDTH_Msk) == PDMA_WIDTH_16) \
        *((__IO uint32_t *)((uint32_t)&PDMA0->TXCNT + (uint32_t)((u32Ch)*0x100))) = ((u32Count) << 1);  \
}

/**
 * @brief       Stop the channel
 *
 * @param[in]   u32Ch     The selected channel
 *
 * @return      None
 *
 * @details     This macro stop the selected channel.
 */
#define PDMA_STOP(u32Ch) (*((__IO uint32_t *)((uint32_t)&PDMA0->CTL + (uint32_t)((u32Ch)*0x100))) &= ~PDMA_CTL_CHEN_Msk)

void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferDirection(uint32_t u32Ch, uint32_t u32Direction);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_SoftwareReset(uint32_t u32Ch);
void PDMA_WrapIntSelect(uint32_t u32Ch, uint32_t u32Mode);


/*@}*/ /* end of group I91200_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_PDMA_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  // __PDMA_H__ 

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
