/**************************************************************************//**
 * @file     BIQ.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/08/22 11:27a $
 * @brief    I91200 BIQ driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "Platform.h"
/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_BIQ_Driver BIQ Driver
  @{
*/


/** @addtogroup I91200_BIQ_EXPORTED_FUNCTIONS BIQ Exported Functions
  @{
*/

/**
  * @brief     Set BIQ coefficients
  * @param     u32BiqCoeff[] BIQ coefficient array for programming the 3 stage BIQ filter. There are 5
  *            coefficeints in each stage. Totally 15 coefficients are in 3.16 format.   
  * @return    None
  */
void BIQ_SetCoeff(uint32_t u32BiqCoeff[30])
{
	volatile uint32_t *p32CoeffAdd;
	uint8_t u8i;
    
    p32CoeffAdd = &BIQ->COEFF0;
    /* Reset BIQ and reload coefficient */
    BIQ->CTL &= (~BIQ_CTL_PRGCOEFF_Msk);
    BIQ->CTL |= (BIQ_CTL_PRGCOEFF_Msk);
    
    /* The default coefficients will be downloaded to the coefficient ram automatically in 32 internal system clocks.*/
	for(u8i=0;u8i<=32;u8i++);
		
	//BIQ->CTL |= BIQ_CTL_PRGCOEFF_Msk;
		
	for(u8i=0;u8i<30;u8i++)
		*(p32CoeffAdd+u8i) = u32BiqCoeff[u8i];
    
    BIQ->CTL &= (~BIQ_CTL_PRGCOEFF_Msk);
}

/**
  * @brief     Reset BIQ 
  * @return    None
  * @details   Biquad is released from reset by setting BIQ_CTL.DLCOEFF=1. After 32 clock cycles, 
  *            processor can setup other Biquad parameters or re-program coefficients before enabling filter
  */
void BIQ_Reset(void)
{
	uint16_t u8i;
	
	BIQ->CTL &= ~BIQ_CTL_DLCOEFF_Msk;
	
	for(u8i=0;u8i<=32;u8i++);
	
}

/**
  * @brief     Load default coefficients
  * @return    None
  * @details   The default coefficients will be downloaded to the coefficient ram automatically in 32 internal system clocks.  
  *            processor can setup other Biquad parameters or re-program coefficients before enabling filter
  */
void BIQ_LoadDefaultCoeff(void)
{
	uint16_t u8i;
	
	BIQ->CTL |= BIQ_CTL_DLCOEFF_Msk;
	
	for(u8i=0;u8i<=32;u8i++);
}

/*@}*/ /* end of group I91200_BIQ_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_BIQ_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
