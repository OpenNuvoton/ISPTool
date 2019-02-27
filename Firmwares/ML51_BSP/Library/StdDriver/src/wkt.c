/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


#include "ML51.h"

/**
 * @brief       Wakeup time setting 
 * @param       u8WKTCLKSouce clock source select define (LIRC / LXT)
 * @param       u16WKTDIV WKT counter divider select  (1/4/16/64/256/512/1024/2048)
 * @param       u8WKTRLData reload counter value(Value < 256) 
 * @return      none
 * @details     timer0/1 max_unit=10ms; timer2 max_unit=1s; timer3 max_unit=100ms; all min_unit = 10us;
 * @note        Once timer1 or timer3 use as baudrate generator, please not define as timer delay.
 * @example			WKT_Open (LIRC,256,100);
 */

void WKT_Open(unsigned char  u8WKTCLKSouce, unsigned int u16WKTDIV, unsigned char u8WKTRLData)
{
		SFRS = 0;
		switch (u8WKTCLKSouce)
		{
			case LIRC: WKCON &= 0xDF; break;
			case LXT: set_CKEN_ELXTEN;									      //step3: Enable LIRC.
				        while((CKSWT|CLR_BIT6)==CLR_BIT6);			//step4: check clock source status and wait for ready
			          WKCON |= 0x20; break;
		}
		switch (u16WKTDIV)
		{
			case 1: WKCON &= 0xF8; break;
			case 4: WKCON &= 0xF8; WKCON |= 0x01; break;
			case 16: WKCON &= 0xF8; WKCON |= 0x02; break;
			case 64: WKCON &= 0xF8; WKCON |= 0x03; break;
			case 256: WKCON &= 0xF8; WKCON |= 0x04; break;
			case 512: WKCON &= 0xF8; WKCON |= 0x05; break;
			case 1024: WKCON &= 0xF8; WKCON |= 0x06; break;
			case 2048: WKCON &= 0xF8; WKCON |= 0x07; break;
		}
		RWK = u8WKTRLData;
		set_WKCON_WKTR;
}

/**
 * @brief       Wakeup time interrupt Enable/disable 
 * @return      WKT_Current_Value
 * @example			WKT_Interrupt(Enable);
 */
void WKT_Interrupt(unsigned char u8WKTINT)
{
		switch (u8WKTINT)
		{
				case Disable: clr_EIE1_EWKT;
				case Enable: set_EIE1_EWKT;
		}
}

/**
 * @brief       Wakeup time setting 
 * @return      WKT_Current_Value
 * @example			temp = WKT_Current_Value();
 */
unsigned char WKT_Current_Value()
{
		unsigned char c;
		SFRS = 0;
		c = CWK;
		return (c);
}

//****************************************************************************************************************	
//**** WKT close   
//**** 1. Disable WKT 
//**** 2. clear WKT reload counter 
//**** 3. return WKT current counter value
void WKT_Close()
{
		clr_WKCON_WKTR;
}