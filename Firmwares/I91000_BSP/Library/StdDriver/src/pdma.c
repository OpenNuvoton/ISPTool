/**************************************************************************//**
 * @file     pdma.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 2:52p $
 * @brief    ISD9000 series PDMA driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  * @{
  */

/** @addtogroup ISD9000_PDMA_Driver PDMA Driver
  * @{
  */

/** @addtogroup ISD9000_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
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
    PDMA_GCR->GLOCTL |= ( (1<<u32Mask) << 8);
}

/**
 * @brief       PDMA Close
 *
 * @return      None
 *
 * @details     This function disable all PDMA channels.
 */
void PDMA_Close(void)
{
    PDMA_GCR->GLOCTL = 0;
}

/**
 * @brief       Set PDMA Transfer Count
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Width        Data width. Valid values are
 *                - \ref PDMA_WIDTH_8
 *                - \ref PDMA_WIDTH_16
 *                - \ref PDMA_WIDTH_32 
 * @param[in]   u32TransCount   Transfer count
 *
 * @return      None
 *
 * @details     This function set the selected channel data width and transfer count.
 */
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
    pdma->DSCT_CTL = (pdma->DSCT_CTL & ~PDMA_DSCT_CTL_TXWIDTH_Msk) | u32Width;
    switch(u32Width)
    {
    case PDMA_WIDTH_32:
        pdma->TXBCCH = (u32TransCount << 2);
        break;

    case PDMA_WIDTH_8:
        pdma->TXBCCH = u32TransCount;
        break;

    case PDMA_WIDTH_16:
        pdma->TXBCCH = (u32TransCount << 1);
        break;

    default:
        ;
    }
}

/**
 * @brief       Set PDMA Transfer Address
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32SrcAddr      Source address
 * @param[in]   u32SrcCtrl      Source control attribute. Valid values are
 *                - \ref PDMA_SAR_INC
 *                - \ref PDMA_SAR_FIX
 *                - \ref PDMA_SAR_WRA 
 * @param[in]   u32DstAddr      destination address
 * @param[in]   u32DstCtrl      destination control attribute. Valid values are
 *                - \ref PDMA_DAR_INC
 *                - \ref PDMA_DAR_FIX
 *                - \ref PDMA_DAR_WRA 
 *
 * @return      None
 *
 * @details     This function set the selected channel source/destination address and attribute.
 */
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->DSCT_ENDSA = u32SrcAddr;
    pdma->DSCT_ENDDA = u32DstAddr;
    pdma->DSCT_CTL = (pdma->DSCT_CTL & ~(PDMA_DSCT_CTL_SASEL_Msk | PDMA_DSCT_CTL_DASEL_Msk)) | (u32SrcCtrl | u32DstCtrl);
}

/**
 * @brief       Set PDMA Transfer Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. Valid values are
 *                - \ref PDMA_SPI0_TX
 *                - \ref PDMA_SPI0_RX
 *                - \ref PDMA_SPIM_TX
 *                - \ref PDMA_SPIM_RX
 *                - \ref PDMA_ADC 
 *                - \ref PDMA_DPWM 
 *                - \ref PDMA_MEM
 *
 * @return      None
 *
 * @details     This function set the selected channel transfer mode. Include peripheral setting.
 */
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral)
{
    uint32_t u32Index = 0;
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
    
    u32Ch = 1<<u32Ch;
    
    for(u32Index = 0; u32Index < 6; u32Index++)
    {
        PDMA_GCR->SVCSEL &= ~(u32Ch << (u32Index * 4));
    }
    
    switch(u32Peripheral)
    {
    case PDMA_SPI0_RX:
        PDMA_GCR->SVCSEL |= (u32Ch<<PDMA_SVCSEL_SPIRXSEL_Pos);
        break;
    case PDMA_SPI0_TX:
        PDMA_GCR->SVCSEL |= (u32Ch<<PDMA_SVCSEL_SPITXSEL_Pos);
        break;

    case PDMA_SPIM_RX:
        PDMA_GCR->SVCSEL |= (u32Ch<<PDMA_SVCSEL_SPIMRXSEL_Pos);
        break;
    case PDMA_SPIM_TX:
        PDMA_GCR->SVCSEL |= (u32Ch<<PDMA_SVCSEL_SPIMTXSEL_Pos);
        break;

    case PDMA_ADC:
        PDMA_GCR->SVCSEL |= (u32Ch<<PDMA_SVCSEL_ADCRXSEL_Pos);
        break;

    case PDMA_DPWM:
        PDMA_GCR->SVCSEL |= (u32Ch<<PDMA_SVCSEL_DPWMTXSEL_Pos);
        break;

    default:/* select PDMA channel as memory to memory */
        pdma->DSCT_CTL = (pdma->DSCT_CTL & (~PDMA_DSCT_CTL_MODESEL_Msk) ) | PDMA_SRAM_SRAM ;
    }
}

/**
 * @brief       Set PDMA Transfer Direction
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Direction    Transfer direction
 *                - \ref PDMA_APB_SRAM
 *                - \ref PDMA_SRAM_APB 
 *
 * @return      None
 *
 * @details     This function select the PDMA transfer direction.
 */
void PDMA_SetTransferDirection(uint32_t u32Ch, uint32_t u32Direction)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->DSCT_CTL = (pdma->DSCT_CTL & (~PDMA_DSCT_CTL_MODESEL_Msk) ) | u32Direction ;
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
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->DSCT_CTL |= (PDMA_DSCT_CTL_TXEN_Msk | PDMA_DSCT_CTL_CHEN_Msk);
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
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->INTENCH |= u32Mask;
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
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->INTENCH &= ~u32Mask;
}

/**
 * @brief       PDMA Software Engine Reset
 *
 * @param[in]   u32Ch           The selected channel
 *
 * @return      None
 *
 * @details     This function will do PDMA software reset.
 */
void PDMA_SoftwareReset(uint32_t u32Ch)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->DSCT_CTL = (pdma->DSCT_CTL & (~PDMA_DSCT_CTL_SWRST_Msk) ) | PDMA_DSCT_CTL_SWRST_Msk ;
}

#define PDMA_HALF_WRAP_MODE        0x00000004UL            /*!<DMA Connect to SPI0 TX */
#define PDMA_FULL_WRAP_MODE        0x00000001UL            /*!<DMA Connect to SPI0 RX */
#define PDMA_BOTH_WRAP_MODE        0x00000005UL            /*!<DMA Connect to SPIM TX */

/**
 * @brief       Select PDMA Wrap Interrupt Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. Valid values are
 *                - \ref PDMA_HALF_WRAP_MODE
 *                - \ref PDMA_FULL_WRAP_MODE
 *                - \ref PDMA_BOTH_WRAP_MODE
 *
 * @return      None
 *
 * @details     This function select the PDMA wrap interrupt mode.
 */
void PDMA_WrapIntSelect(uint32_t u32Ch, uint32_t u32Mode)
{
    PDMA_T *pdma;
    pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->DSCT_CTL = (pdma->DSCT_CTL & (~PDMA_DSCT_CTL_WAINTSEL_Msk) ) | (u32Mode << PDMA_DSCT_CTL_WAINTSEL_Pos) ;
}

/*@}*/ /* end of group ISD9000_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_PDMA_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
