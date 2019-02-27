/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//***********************************************************************************************************

#include "ML51.h"

void LowPower_LIRC_UART2_4800_init(void)
{
		set_SC0CR0_SCEN;         /* Enable SC module */
		set_SC0CR1_UARTEN;       /* set as UART mode */
		set_SC0CR1_CLKKEEP;      
		clr_SC0CR0_CONSEL;       /* clr CONSEL and AUTOCEN*/
		clr_SC0CR0_AUTOCEN;
		SC0ETURD0 = 7;          /* define baud rate low byte    38400/8=4800  */
		SC0ETURD1 &= 0x00;       /* define baud rate high byte */
		set_SC0CR1_PBOFF; 		   //parity bit disable
		SC0CR1&=0XCF;				     //datalegth 8bit
		set_SC0CR0_NSB;			     //stop bit = 1bit
		clr_SC0CR0_RXOFF;
}

void LowPower_UART2_Send_Data( unsigned char c)
{
		clr_SC0CR0_TXOFF;
		SC0DR = c;
		while((SC0TSR|CLR_BIT3)==CLR_BIT3);
		clr_SC0CR0_TXOFF;
}