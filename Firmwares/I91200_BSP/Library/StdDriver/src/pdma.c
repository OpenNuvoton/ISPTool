/**************************************************************************//**
 * @file     pdma.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date:  17/01/18 2:52p  $
 * @brief    I91200 series PDMA driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"
//#include "PDMA.h"
/** @addtogroup I91200_Device_Driver I91200 Device Driver
  * @{
  */

/** @addtogroup I91200_PDMA_Driver PDMA Driver
  * @{
  */

/** @addtogroup I91200_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
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
   PDMAC->GCTL|= (u32Mask << 8);
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
   PDMAC->GCTL = 0;
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
    pdma->CTL = (pdma->CTL & ~PDMA_CTL_TXWIDTH_Msk) | u32Width;
    switch(u32Width)
    {
    case PDMA_WIDTH_32:
        pdma->TXCNT  = (u32TransCount << 2);
        break;

    case PDMA_WIDTH_8:
        pdma->TXCNT = u32TransCount;
        break;

    case PDMA_WIDTH_16:
        pdma->TXCNT = (u32TransCount << 1);
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

    pdma->SADDR= u32SrcAddr;
    pdma->DADDR = u32DstAddr;
    pdma->CTL = (pdma->CTL & ~(PDMA_CTL_SASEL_Msk | PDMA_CTL_DASEL_Msk)) | (u32SrcCtrl | u32DstCtrl);
}

/**
 * @brief       Set PDMA Transfer Direction
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Direction    Transfer direction
 *                - \ref PDMA_SRAM_SRAM
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
	
    pdma->CTL = (pdma->CTL & (~PDMA_CTL_MODESEL_Msk) ) | u32Direction ;
}

/**
 * @brief       Set PDMA Transfer Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. Valid values are
 *                - \ref PDMA_SPI0_TX
 *                - \ref PDMA_SPI0_RX
 *                - \ref PDMA_I2S_TX
 *                - \ref PDMA_I2S_RX
 *                - \ref PDMA_UART0_TX
 *                - \ref PDMA_UART0_RX 
 *                - \ref PDMA_SDADC 
 *                - \ref PDMA_DPWM 
 *                - \ref PDMA_SPI1_TX
 *                - \ref PDMA_SPI1_RX
 *                - \ref PDMA_UART1_TX
 *                - \ref PDMA_UART1_RX 
 *                - \ref PDMA_SARADC 
 *                - \ref PDMA_MEM
 * @return      None
 *
 * @details     This function set the selected channel transfer mode. Include peripheral setting.
 */
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral)
{
    // This function doesn't handle channel management
	uint32_t u32Index = 0;
    switch(u32Peripheral)
    {
    case PDMA_SPI0_RX:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_SPI0RXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_SPI0RXSEL_Pos);
        break;
    case PDMA_SPI0_TX:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0& ~PDMA_SVCSEL0_SPI0TXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_SPI0TXSEL_Pos);
        break;
    case PDMA_UART0_RX:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_UART0RXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_UART0RXSEL_Pos);
        break;
    case PDMA_UART0_TX:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_UART0XSEL_Msk) | (u32Ch << PDMA_SVCSEL0_UART0XSEL_Pos);
        break;
    case PDMA_I2S_RX:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_I2SRXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_I2SRXSEL_Pos);
        break;
    case PDMA_I2S_TX:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_I2STXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_I2STXSEL_Pos);
        break;
    case PDMA_SDADC:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_SDADCRXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_SDADCRXSEL_Pos);
        break;
    case PDMA_DPWM:
        PDMAC->SVCSEL0 = (PDMAC->SVCSEL0 & ~PDMA_SVCSEL0_DPWMTXSEL_Msk) | (u32Ch << PDMA_SVCSEL0_DPWMTXSEL_Pos);
        break;
	case PDMA_SPI1_RX:
        PDMAC->SVCSEL1 = (PDMAC->SVCSEL1 & ~PDMA_SVCSEL1_SPI1RXSEL_Msk) | (u32Ch << PDMA_SVCSEL1_SPI1RXSEL_Pos);
        break;
    case PDMA_SPI1_TX:
        PDMAC->SVCSEL1 = (PDMAC->SVCSEL1 & ~PDMA_SVCSEL1_SPI1TXSEL_Msk) | (u32Ch << PDMA_SVCSEL1_SPI1TXSEL_Pos);
        break;
	 case PDMA_UART1_RX:
        PDMAC->SVCSEL1 = (PDMAC->SVCSEL1 & ~PDMA_SVCSEL1_UART1RXSEL_Msk) | (u32Ch << PDMA_SVCSEL1_UART1RXSEL_Pos);
        break;
    case PDMA_UART1_TX:
        PDMAC->SVCSEL1 = (PDMAC->SVCSEL1 & ~PDMA_SVCSEL1_UART1TXSEL_Msk) | (u32Ch << PDMA_SVCSEL1_UART1TXSEL_Pos);
        break;
	case PDMA_SARADC:
        PDMAC->SVCSEL1 = (PDMAC->SVCSEL1 & ~PDMA_SVCSEL1_SARADCRXSEL_Msk) | (u32Ch << PDMA_SVCSEL1_SARADCRXSEL_Pos);
        break;
    default:/* select PDMA channel as memory to memory */
        for(u32Index = 0; u32Index < 8; u32Index++)
        {
            if((PDMAC->SVCSEL0 & (0xF << (u32Index * 4))) == (u32Ch << (u32Index * 4)))
                PDMAC->SVCSEL0 |= 0xF << (u32Index * 4);
			if((PDMAC->SVCSEL1 & (0xF << (u32Index * 4))) == (u32Ch << (u32Index * 4)))
                PDMAC->SVCSEL1 |= 0xF << (u32Index * 4);
        }
    }
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

    pdma->CTL |= (PDMA_CTL_TXEN_Msk | PDMA_CTL_CHEN_Msk);
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

    pdma->INTEN |= u32Mask;
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

    pdma->INTEN &= ~u32Mask;
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
	
    pdma->CTL = (pdma->CTL & (~PDMA_CTL_SWRST_Msk) ) | PDMA_CTL_SWRST_Msk ;
}

/**
 * @brief       Select PDMA Wrap Interrupt Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mode   		The selected peripheral. Valid values are
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
	
    pdma->CTL = (pdma->CTL & (~PDMA_CTL_WAINTSEL_Msk) ) | (u32Mode << PDMA_CTL_WAINTSEL_Pos) ;
}

/*@}*/ /* end of group I91200_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_PDMA_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
