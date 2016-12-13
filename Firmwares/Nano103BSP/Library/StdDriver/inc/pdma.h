/**************************************************************************//**
 * @file     pdma.h
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/12/28 9:08a $
 * @brief    NANO103 series PDMA driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PDMA_H__
#define __PDMA_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_PDMA_Driver PDMA Driver
  @{
*/

/** @addtogroup NANO103_PDMA_EXPORTED_CONSTANTS PDMA Exported Constants
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
/*  Peripheral Transfer Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SPI0_TX        0x00000000UL            /*!<DMA Connect to SPI0 TX */
#define PDMA_SPI1_TX        0x00000001UL            /*!<DMA Connect to SPI1 TX */
#define PDMA_UART0_TX       0x00000002UL            /*!<DMA Connect to UART0 TX */
#define PDMA_UART1_TX       0x00000003UL            /*!<DMA Connect to UART1 TX */
#define PDMA_SPI3_TX        0x00000005UL            /*!<DMA Connect to SPI3 TX */
#define PDMA_SPI2_TX        0x00000008UL            /*!<DMA Connect to SPI2 TX */
#define PDMA_TMR0           0x00000009UL            /*!<DMA Connect to TMR0 */
#define PDMA_TMR1           0x0000000AUL            /*!<DMA Connect to TMR1 */
#define PDMA_TMR2           0x0000000BUL            /*!<DMA Connect to TMR2 */
#define PDMA_TMR3           0x0000000CUL            /*!<DMA Connect to TMR3 */

#define PDMA_SPI0_RX        0x00000010UL            /*!<DMA Connect to SPI0 RX */
#define PDMA_SPI1_RX        0x00000011UL            /*!<DMA Connect to SPI1 RX */
#define PDMA_UART0_RX       0x00000012UL            /*!<DMA Connect to UART0 RX */
#define PDMA_UART1_RX       0x00000013UL            /*!<DMA Connect to UART1 RX */
#define PDMA_SPI3_RX        0x00000015UL            /*!<DMA Connect to SPI3 RX */
#define PDMA_ADC            0x00000016UL            /*!<DMA Connect to I2S1 RX */
#define PDMA_SPI2_RX        0x00000018UL            /*!<DMA Connect to SPI2 RX */
#define PDMA_MEM            0x0000001FUL            /*!<DMA Connect to Memory */

/*---------------------------------------------------------------------------------------------------------*/
/*  Time-out Counter Clock Source Prescaler Constant Definitions                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_TOC_TPSC_HCLK_DIV_2POW8        0x00000000UL      /*!<DMA time-out clock source is HCLK/2^8  */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW9        0x00000000UL      /*!<DMA time-out clock source is HCLK/2^9  */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW10       0x00000000UL      /*!<DMA time-out clock source is HCLK/2^10 */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW11       0x00000000UL      /*!<DMA time-out clock source is HCLK/2^11 */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW12       0x00000000UL      /*!<DMA time-out clock source is HCLK/2^12 */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW13       0x00000000UL      /*!<DMA time-out clock source is HCLK/2^13 */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW14       0x00000000UL      /*!<DMA time-out clock source is HCLK/2^14 */
#define PDMA_TOC_TPSC_HCLK_DIV_2POW15       0x00000000UL      /*!<DMA time-out clock source is HCLK/2^15 */

/*@}*/ /* end of group NANO103_PDMA_EXPORTED_CONSTANTS */

/** @addtogroup NANO103_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
 * @brief       Get PDMA Interrupt Status
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This macro gets the interrupt status.
 * \hideinitializer
 */
#define PDMA_GET_INT_STATUS()   ((uint32_t)(PDMAGCR->GINTSTS))

/**
 * @brief       Get PDMA Channel Interrupt Status
 *
 * @param[in]   u32Ch   Selected DMA channel
 *
 * @return      Interrupt Status
 *
 * @details     This macro gets the channel interrupt status.
 * \hideinitializer
 */
#define PDMA_GET_CH_INT_STS(u32Ch)   (*((__IO uint32_t *)((uint32_t)&PDMA1->INTSTSn + (uint32_t)((u32Ch-1)*0x100))))

/**
 * @brief       Clear PDMA Channel Interrupt Flag
 *
 * @param[in]   u32Ch   Selected DMA channel
 * @param[in]   u32Mask Interrupt Mask
 *
 * @return      None
 *
 * @details     This macro clear the channel interrupt flag.
 * \hideinitializer
 */
#define PDMA_CLR_CH_INT_FLAG(u32Ch, u32Mask)   (*((__IO uint32_t *)((uint32_t)&PDMA1->INTSTSn + (uint32_t)((u32Ch-1)*0x100))) = (u32Mask))

/**
 * @brief       Check Channel Status
 *
 * @param[in]   u32Ch     The selected channel
 *
 * @return      0 = idle
 * @return      1 = busy
 *
 * @details     Check the selected channel is busy or not.
 * \hideinitializer
 */
#define PDMA_IS_CH_BUSY(u32Ch)    ((*((__IO uint32_t *)((uint32_t)&PDMA1->CTLn +(uint32_t)((u32Ch-1)*0x100))) & PDMA_CH_CTLn_TRIGEN_Msk)? 1 : 0)

/**
 * @brief       Set Source Address
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel source address.
 * \hideinitializer
 */
#define PDMA_SET_SRC_ADDR(u32Ch, u32Addr) (*((__IO uint32_t *)((uint32_t)&PDMA1->SAn + (uint32_t)((u32Ch-1)*0x100))) = (u32Addr))

/**
 * @brief       Set Destination Address
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel destination address.
 * \hideinitializer
 */
#define PDMA_SET_DST_ADDR(u32Ch, u32Addr) (*((__IO uint32_t *)((uint32_t)&PDMA1->DAn + (uint32_t)((u32Ch-1)*0x100))) = (u32Addr))

/**
 * @brief       Set Transfer Count
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Count  Transfer Count
 *
 * @return      None
 *
 * @details     This macro set the selected channel transfer count.
 * \hideinitializer
 */
#define PDMA_SET_TRANS_CNT(u32Ch, u32Count)    \
    (*((__IO uint32_t *)((uint32_t)&PDMA1->CNTn + (uint32_t)((u32Ch-1)*0x100))) = \
    ((*((__IO uint32_t *)((uint32_t)&PDMA1->CNTn + (uint32_t)((u32Ch-1)*0x100))) & ~PDMA_CH_CNTn_TCNT_Msk) | (u32Count & 0xffff))) \


/**
 * @brief       Set Periodic Count
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Count  Periodic FIFO Count
 *
 * @return      None
 *
 * @details     This macro set the selected channel periodic fifo count.
 * \hideinitializer
 */
#define PDMA_SET_PERIODIC_CNT(u32Ch, u32Count)    \
    (*((__IO uint32_t *)((uint32_t)&PDMA1->CNTn + (uint32_t)((u32Ch-1)*0x100))) = \
    ((*((__IO uint32_t *)((uint32_t)&PDMA1->CNTn + (uint32_t)((u32Ch-1)*0x100))) & ~PDMA_CH_TOCn_TPSC_Msk) | ((u32Count & 0xffff) << PDMA_CH_TOCn_TPSC_Pos))) \


/**
 * @brief       Stop the channel
 *
 * @param[in]   u32Ch     The selected channel
 *
 * @return      None
 *
 * @details     This macro stop the selected channel.
 * \hideinitializer
 */
#define PDMA_STOP(u32Ch) (*((__IO uint32_t *)((uint32_t)&PDMA1->CTLn + (uint32_t)((u32Ch-1)*0x100))) &= ~PDMA_CH_CTLn_CHEN_Msk)

void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetTimeOut(uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);

/*@}*/ /* end of group NANO103_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_PDMA_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__PDMA_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
