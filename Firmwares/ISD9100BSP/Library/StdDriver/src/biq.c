/**************************************************************************//**
 * @file     BIQ.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/04 11:27a $
 * @brief    ISD9100 BIQ driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9100.h"
/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_BIQ_Driver BIQ Driver
  @{
*/


/** @addtogroup ISD9100_BIQ_EXPORTED_FUNCTIONS BIQ Exported Functions
  @{
*/

/**
  * @brief     Set BIQ coefficients
  * @param     u32BiqCoeff[] BIQ coefficient array for programming the 3 stage BIQ filter. There are 5
  *            coefficeints in each stage. Totally 15 coefficients are in 3.16 format.   
  * @return    None
  */
void BIQ_SetCoeff(uint32_t u32BiqCoeff[15])
{
	volatile uint32_t *p32CoeffAdd;
	uint8_t u8i;
    
    p32CoeffAdd = &BIQ->COEFF0;
    /* Reset BIQ and reload coefficient */
    BIQ->CTL &= (~BIQ_CTL_PRGCOEFF_Msk);
    BIQ->CTL |= (BIQ_CTL_PRGCOEFF_Msk);
    
    /* The default coefficients will be downloaded to the coefficient ram automatically in 32 internal system clocks.*/
	for(u8i=0;u8i<=32;u8i++);
		
	BIQ->CTL |= BIQ_CTL_PRGCOEFF_Msk;
		
	for(u8i=0;u8i<15;u8i++)
		*(p32CoeffAdd+u8i) = u32BiqCoeff[u8i];
    
    BIQ->CTL &= (~BIQ_CTL_PRGCOEFF_Msk);
}

/**
  * @brief     Reset BIQ 
  * @return    None
  * @details  Biquad is released from reset by setting BIQ_CTL.DLCOEFF=1. After 32 clock cycles, 
  *           processor can setup other Biquad parameters or re-program coefficients before enabling filter
  */
void BIQ_Reset(void)
{
	uint16_t u8i;
	
	BIQ->CTL |= BIQ_CTL_DLCOEFF_Msk;
	
	for(u8i=0;u8i<=32;u8i++);
}

/*@}*/ /* end of group ISD9100_BIQ_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_BIQ_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
