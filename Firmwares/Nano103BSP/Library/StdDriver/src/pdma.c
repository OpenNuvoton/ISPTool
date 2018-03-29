/**************************************************************************//**
 * @file     pdma.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/12/30 10:58a $
 * @brief    Nano103 series PDMA driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Nano103.h"



/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_PDMA_Driver PDMA Driver
  @{
*/


/** @addtogroup NANO103_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
 * @brief       PDMA Open
 *
 * @param[in]   u32Mask     Channel enable bits.
 *
 * @return      None
 *
 * @details     This function enable the PDMA channels.
 */
void PDMA_Open(uint32_t u32Mask)
{
    PDMAGCR->GCTL |= (u32Mask << 8);
}

/**
 * @brief       PDMA Close
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function disable all PDMA channels.
 */
void PDMA_Close(void)
{
    PDMAGCR->GCTL = 0;
}

/**
 * @brief       Set PDMA Transfer Count
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Width        Data width. \ref PDMA_WIDTH_8, \ref PDMA_WIDTH_16, or \ref PDMA_WIDTH_32
 * @param[in]   u32TransCount   Transfer count
 *
 * @return      None
 *
 * @details     This function set the selected channel data width and transfer count.
 */
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount)
{
    PDMA_CH_T *pdma;
    pdma = (PDMA_CH_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
    pdma->CTLn = (pdma->CTLn & ~PDMA_CH_CTLn_TXWIDTH_Msk) | u32Width;
    pdma->CNTn = (u32TransCount & 0xffff);
}

/**
 * @brief       Set PDMA Transfer Address
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32SrcAddr      Source address
 * @param[in]   u32SrcCtrl      Source control attribute. \ref PDMA_SAR_INC, \ref PDMA_SAR_FIX, or \ref PDMA_SAR_WRA
 * @param[in]   u32DstAddr      destination address
 * @param[in]   u32DstCtrl      destination control attribute. \ref PDMA_DAR_INC, \ref PDMA_DAR_FIX, or \ref PDMA_DAR_WRA
 *
 * @return      None
 *
 * @details     This function set the selected channel source/destination address and attribute.
 */
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl)
{
    PDMA_CH_T *pdma;
    pdma = (PDMA_CH_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->SAn = u32SrcAddr;
    pdma->DAn = u32DstAddr;
    pdma->CTLn = (pdma->CTLn & ~(PDMA_CH_CTLn_SASEL_Msk|PDMA_CH_CTLn_DASEL_Msk)) | (u32SrcCtrl | u32DstCtrl);
}

/**
 * @brief       Set PDMA Transfer Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral.
 *                              \ref PDMA_SPI0_TX, \ref PDMA_SPI1_TX, \ref PDMA_UART0_TX, \ref PDMA_UART1_TX,
 *                              \ref PDMA_SPI2_TX, \ref PDMA_SPI3_TX, \ref PDMA_TMR0, \ref PDMA_TMR1,
 *                              \ref PDMA_TMR2, \ref PDMA_TMR3, \ref PDMA_SPI0_RX, \ref PDMA_SPI1_RX,
 *                              \ref PDMA_UART0_RX, \ref PDMA_UART1_RX, \ref PDMA_SPI2_RX, \ref PDMA_SPI3_RX,
 *                              \ref PDMA_ADC, \ref PDMA_MEM
 * @param[in]   u32ScatterEn    Scatter-gather mode enable
 * @param[in]   u32DescAddr     Scatter-gather descriptor address
 *
 * @return      None
 *
 * @details     This function set the selected channel transfer mode. Include peripheral setting.
 */
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr)
{
    switch (u32Ch)
    {
    case 1:
        PDMAGCR->REQSEL0 = (PDMAGCR->REQSEL0 & ~DMA_GCR_REQSEL0_REQSRC1_Msk) | (u32Peripheral << DMA_GCR_REQSEL0_REQSRC1_Pos);
        break;
    case 2:
        PDMAGCR->REQSEL0 = (PDMAGCR->REQSEL0 & ~DMA_GCR_REQSEL0_REQSRC2_Msk) | (u32Peripheral << DMA_GCR_REQSEL0_REQSRC2_Pos);
        break;
    case 3:
        PDMAGCR->REQSEL0 = (PDMAGCR->REQSEL0 & ~DMA_GCR_REQSEL0_REQSRC3_Msk) | (u32Peripheral << DMA_GCR_REQSEL0_REQSRC3_Pos);
        break;
    case 4:
        PDMAGCR->REQSEL1 = (PDMAGCR->REQSEL1 & ~DMA_GCR_REQSEL1_REQSRC4_Msk) | u32Peripheral;
        break;
    default:
        ;
    }
}

/**
 * @brief       Set PDMA Timeout
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32OnOff        Enable/disable time out function
 * @param[in]   u32TimeOutCnt   Timeout count
 *
 * @return      None
 *
 * @details     This function set the timeout count.
 */
void PDMA_SetTimeOut(uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt)
{
    PDMA_CH_T *pdma;
    pdma = (PDMA_CH_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->TOCn &= ~PDMA_CH_TOCn_TOC_Msk;
    pdma->TOCn |= u32TimeOutCnt;
    pdma->CTLn = (pdma->CTLn & ~PDMA_CH_CTLn_TOUTEN_Msk) | (u32OnOff << PDMA_CH_CTLn_TOUTEN_Pos);

}

/**
 * @brief       Trigger PDMA
 *
 * @param[in]   u32Ch           The selected channel
 *
 * @return      None
 *
 * @details     This function trigger the selected channel.
 */
void PDMA_Trigger(uint32_t u32Ch)
{
    PDMA_CH_T *pdma;
    pdma = (PDMA_CH_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->CTLn |= (PDMA_CH_CTLn_TRIGEN_Msk | PDMA_CH_CTLn_CHEN_Msk);
}

/**
 * @brief       Enable Interrupt
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type
 *
 * @return      None
 *
 * @details     This function enable the selected channel interrupt.
 */
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA_CH_T *pdma;
    pdma = (PDMA_CH_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->INTENn |= u32Mask;
}

/**
 * @brief       Disable Interrupt
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type
 *
 * @return      None
 *
 * @details     This function disable the selected channel interrupt.
 */
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA_CH_T *pdma;
    pdma = (PDMA_CH_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->INTENn &= ~u32Mask;
}


/*@}*/ /* end of group NANO103_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_PDMA_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
