#include "ML51.h"

/**
  * @brief  Configure the specified ACMP module
  * @param[in]  u8ACMPNum Acmp module 0 or 1
  *                  - \ref ACMP0
  *                  - \ref ACMP1	
  * @param[in]  u8ChNum Comparator positive input selection.  Including:.
  *                  - \ref ACMP_CTL_POSSEL_P0 (P2.5)
  *                  - \ref ACMP_CTL_POSSEL_P1 (P2.3)
  *                  - \ref ACMP_CTL_POSSEL_P2 (P2.1)
  *                  - \ref ACMP_CTL_POSSEL_P3 (P3.0)
  * @param[in]  u8NegSrc Comparator negative input selection.  Including:
  *                  - \ref ACMP_CTL_NEGSEL_PIN_N0
  *                  - \ref ACMP_CTL_NEGSEL_CRV
  *                  - \ref ACMP_CTL_NEGSEL_VBG
  *                  - \ref ACMP_CTL_NEGSEL_PIN_N1	
	* @param[in]  u8CVRSource The ACMP Comparator Reference Voltage Select
  *                  - \ref ACMP_CTL_CRV_VREF
  *                  - \ref ACMP_CTL_CRV_VDD
  *                  - \ref 0                          if not use CRV this param fill with 0. 
  * @param[in]  u8ACMPOut The ACMP output enable
  *                  - \ref ACMP_CTL_ACMP0_OUTPUT_ENABLE      
  *                  - \ref ACMP_CTL_ACMP0_OUTPUT_DISABLE					
  *                  - \ref ACMP_CTL_ACMP1_OUTPUT_ENABLE					
  *                  - \ref ACMP_CTL_ACMP0_OUTPUT_DISABLE
  * @param[in]  u8HysteresisEn The hysteresis function option. Including:
  *                  - \ref ACMP_CTL_HYSTERESIS_ENABLE
  *                  - \ref ACMP_CTL_HYSTERESIS_DISABLE
  * @return     None
  * @example    Configure hysteresis function, select the source of negative input and enable analog comparator.
  */		
void ACMP_Open(unsigned char u8ACMPNum, unsigned char u8PosSrc, unsigned char u8NegSrc, unsigned char u8CVRSource, unsigned char u8ACMPOut, unsigned char u8HysteresisEn)
{
		if(u8NegSrc==ACMP_CTL_NEGSEL_CRV)
			SFRS=0x02;ACMPCR2|=(u8CVRSource|ACMP_CTL_CRV_ENABLE);
		switch (u8ACMPNum)
		{
			case ACMP0: SFRS=0x00;ACMPCR0=0x00;ACMPCR0|=((u8PosSrc<<6)|(u8NegSrc<<4)|u8HysteresisEn);SFRS=0x02;ACMPCR2|=u8ACMPOut;SFRS=0x00;ACMPCR0|=0x01; break;
			case ACMP1: SFRS=0x00;ACMPCR1=0x00;ACMPCR1|=((u8PosSrc<<6)|(u8NegSrc<<4)|u8HysteresisEn);SFRS=0x02;ACMPCR2|=u8ACMPOut;SFRS=0x00;ACMPCR0|=0x01; break;
		}
}

/**
  * @brief  Configure the ACMP module interrupt enable
  * @param[in]  u8ACMPNum Acmp module 0 or 1
  *                  - \ref ACMP0
  *                  - \ref ACMP1	
	* @param[in]  u8ACMPWake The ACMP wakeup function option. Including:
  *                  - \ref ACMP_CTL_WAKEUP_ENABLE
  *                  - \ref ACMP_CTL_WAKEUP_DISABLE
  * @param[in]  u8ACMPINT The interrupt function option. Including:
  *                  - \ref ACMP_CTL_INT_ENABLE
  *                  - \ref ACMP_CTL_INT_DISABLE
  * @return     None
  * @example    Configure hysteresis function, select the source of negative input and enable analog comparator.
  */	
void ACMP_INTEnable(unsigned char u8ACMPNum, unsigned char u8ACMPWakeEn,unsigned char u8ACMPINTEn)
{
			switch (u8ACMPNum)
		{
			case ACMP0:SFRS=0x00;ACMPCR0|=(u8ACMPWakeEn|u8ACMPINTEn); break;
			case ACMP1:SFRS=0x00;ACMPCR1|=(u8ACMPWakeEn|u8ACMPINTEn); break;
		}
		ACMPSR = 0;
}

/**
  * @brief  Configure ACMP Negtive input by CRV value
  * @param[in]  u8ACMPNum Acmp module 0 or 1
  *                  - \ref ACMP0
  *                  - \ref ACMP1
  * @param[in]  u8ACMPValue CRV value 
  *                  - \ref 0~7
  *             CRV output value = CRV source voltage*(2/12+u8ACMPValue/12).
*/
void ACMP_CRVValue(unsigned char u8ACMPNum, unsigned char u8ACMPValue)
{
	switch(u8ACMPNum)
	{
		case ACMP0: ACMPVREF|=(u8ACMPValue&0x07); break;
		case ACMP1: ACMPVREF|=((u8ACMPValue<<4)&0x70); break;
	}
}

