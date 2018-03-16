/**************************************************************************//**
 * @file     DPWM.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/07/04 07:27p $
 * @brief    ISD9100 DPWM driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9100.h"
/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_DPWM_Driver DPWM Driver
  @{
*/


/** @addtogroup ISD9100_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

/**
  * @brief      DPWM open function
  * @return     None
  * @details    
  */
void DPWM_Open(void)
{

}

/**
  * @brief      DPWM close function
  * @return     None
  * @details   
  */
void DPWM_Close(void)
{   
      
}

/**
  * @brief      Write to DPWM FIFO for transmit
  * @param      pi16Stream is pointer of input data stream for transmit.
  * @param      u32count is transmit sample count.  
  * @return     None
  * @details    If all channels of PDMA are occupied, programmer can call this 
  *             function to write FIFO by CPU. But programmer needs to 
  *             handle suitable time for writing.  
  */
void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32count)
{
	uint32_t u32countRemain;
	
	u32countRemain = u32count;

	while(u32countRemain > 0)	
	{
		while(DPWM->STS == 1); //FIFO is full

		DPWM->DATA = *pi16Stream++;
		u32countRemain --;	
	}
}

/**
  * @brief      Set the sample Rate of data 
  * @param      u32SampleRate is sample Rate of data.
  * @return     Real sample rate.
  */
uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32Div;
	
	SystemCoreClockUpdate();
	u32Div = (SystemCoreClock/64)/u32SampleRate;
	
	if(u32Div<1)
		u32Div = 1;
	if (u32Div>255)
		u32Div = 255;
	DPWM->ZOHDIV = u32Div;
	return (SystemCoreClock /(DPWM->ZOHDIV*64));
}

/**
  * @brief      Get the sample Rate of data 
  * @return     Real sample rate.
  */
uint32_t DPWM_GetSampleRate(void)
{
	 SystemCoreClockUpdate();
   return (SystemCoreClock /(DPWM->ZOHDIV*64));
}


/*@}*/ /* end of group ISD9100_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_DPWM_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

