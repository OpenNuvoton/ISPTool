/**************************************************************************//**
 * @file        spim.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       15/11/26 5:00p$
 * @brief       ISD9000 SPIM driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_SPIM_Driver SPIM Driver
  @{
*/

/** @addtogroup ISD9000_SPIM_EXPORTED_FUNCTIONS SPIM Exported Functions
  @{
*/

/**
  * @brief                      This function makes SPIM module be ready to transfer.
  * @param[in]  spim            Base address of SPIM module.
  * @param[in]  u32BusClock     Expected frequency of SPI bus clock in Hz.
  * @return                     Actual frequency of SPI peripheral clock.
  * @note   If u32BusClock = 0, divider setting will be set to the maximum value.
  * @note   If u32BusClock >= SPI peripheral clock source, divider setting will be set to 0.
  * @note   If u32BusClock != 0, actual frequency will be the nearest to and not greater than u32BusClock.
  * @note   This function doesn't change system clock setting.
  */
uint32_t SPIM_Open(SPIM_T *spim, uint32_t u32BusClock)
{
    SYS->IPRST1 |= SYS_IPRST1_SPIMRST_Msk;            // Reset IP.
    SYS->IPRST1 &= ~SYS_IPRST1_SPIMRST_Msk;
	
    return SPIM_SetBusClock(spim, u32BusClock);
}

/**
  * @brief                      Disable SPIM module.
  * @param[in]  spim            Base address of SPIM module.
  * @return                     None.
  * @details                    This function will reset SPIM module.
  */
void SPIM_Close(SPIM_T *spim)
{
    SYS->IPRST1 |= SYS_IPRST1_SPIMRST_Msk;       // Reset IP.
    SYS->IPRST1 &= ~SYS_IPRST1_SPIMRST_Msk;
}

/**
  * @brief                  Set SPI bus clock.
  * @param[in]  spim        Base address of SPIM module.
  * @param[in]  u32BusClock Expected frequency of SPI bus clock in Hz.
  * @return                 Actual frequency of SPI bus clock.
  * @note                   If u32BusClock = 0, divider will be set to the maximum value.
  * @note                   If u32BusClock >= SPI peripheral clock source, divider will be set to 0.
  * @note                   If u32BusClock != 0, actual frequency will be the nearest to and not greater than u32BusClock.
  * @note                   This function doesn't change system clock setting.
  */
uint32_t SPIM_SetBusClock(SPIM_T *spim, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc, u32Div = 0;

	u32ClkSrc = CLK_GetHCLKFreq();
	
    if(u32BusClock != 0 ) {
       if (u32BusClock == u32ClkSrc)  
			  u32Div = 0;
			 else {	 
			  u32Div = (u32ClkSrc / u32BusClock) / 2;
        if(u32Div > 0xffff)
				   u32Div = 0xffff;
			 }
    }
    spim->CTL1 = (spim->CTL1 & (~SPIM_CTL1_DIVIDER_Msk)) | (u32Div << SPIM_CTL1_DIVIDER_Pos);
   
	return SPIM_GetBusClock(spim);
}

/**
  * @brief              Get the actual frequency of SPI bus clock.
  * @param[in]  spim    Base address of SPIM module.
  * @return             Actual SPI bus clock frequency in Hz.
  */
uint32_t SPIM_GetBusClock(SPIM_T *spim)
{
    uint32_t u32Divider = ((spim->CTL1 & SPIM_CTL1_DIVIDER_Msk) >> SPIM_CTL1_DIVIDER_Pos);
    return ((u32Divider)?(CLK_GetHCLKFreq()/(u32Divider*2)):CLK_GetHCLKFreq());
}

/**
  * @brief              Enable interrupt function.
  * @param[in]  spim    Base address of SPIM module.
  * @param[in]  u32Mask Combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled.
  *                     Valid values include:
  *                     - \ref SPIM_INT_MASK
  * @return             None.
  * @details            Enable SPIM related interrupts specified by u32Mask parameter.
  */
void SPIM_EnableInt(SPIM_T *spim, uint32_t u32Mask)
{
    if ((u32Mask & SPIM_INT_MASK) == SPIM_INT_MASK)
        spim->CTL0 |= SPIM_CTL0_IEN_Msk;
}

/**
  * @brief              Disable interrupt function.
  * @param[in]  spim    Base address of SPIM module.
  * @param[in]  u32Mask Combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled.
  *                     Valid values include:
  *                     - \ref SPIM_INT_MASK
  * @return             None.
  * @details            Disable SPIM related interrupts specified by u32Mask parameter.
  */
void SPIM_DisableInt(SPIM_T *spim, uint32_t u32Mask)
{
    if ((u32Mask & SPIM_INT_MASK) == SPIM_INT_MASK)
        spim->CTL0 &= ~SPIM_CTL0_IEN_Msk;
}

/**
  * @brief              Get interrupt flag.
  * @param[in]  spim    Base address of SPIM module.
  * @param[in]  u32Mask Combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read.
  *                     Valid values include:
  *                     - \ref SPIM_INT_MASK
  * @return             Interrupt flags of selected sources.
  * @details            Get SPIM related interrupt flags specified by u32Mask parameter.
  */
uint32_t SPIM_GetIntFlag(SPIM_T *spim, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0;

    if ((u32Mask & SPIM_INT_MASK) && (spim->CTL0 & SPIM_CTL0_IF_Msk)) {
        u32IntFlag |= SPIM_INT_MASK;
    }
    return u32IntFlag;
}

/**
  * @brief              Clear interrupt flag.
  * @param[in]  spim    Base address of SPIM module.
  * @param[in]  u32Mask Combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be cleared.
  *                     Valid values include:
  *                     - \ref SPIM_INT_MASK
  * @return             None.
  * @details            Clear SPIM related interrupt flags specified by u32Mask parameter.
  */
void SPIM_ClearIntFlag(SPIM_T *spim, uint32_t u32Mask)
{
    if (u32Mask & SPIM_INT_MASK)
        spim->CTL0 |= SPIM_CTL0_IF_Msk;
}

/*@}*/ /* end of group ISD9000_SPIM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_SPIM_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
