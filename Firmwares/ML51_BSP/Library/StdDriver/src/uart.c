/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2018
//***********************************************************************************************************

#include "ML51.h"

/**
 * @brief       UART no interrupt enable setting 
 * @param       UART0~3, baudrate value
 * @return      none
 * @details     none
 * @note        max baud rate = 1.5MHz when Fsys = 24MHz
 */
//****************************************************************************************************************	
//**** UART Enable Setting  
//**** 1. Define Fsys value(value)
//**** 2. Select UART port(UART0_Timer1 / UART0_Timer3 / UART1_Timer3 / UART2/UART3) 
//**** 3. Define baudrate (value)
//**** For example: UART_Open(24000000,UART0_Timer1,115200)
void UART_Open(unsigned long u32SysClock, unsigned char u8UARTPort,unsigned long u32Baudrate)
{
	switch(u8UARTPort)
	{
		case UART0_Timer1:
					SFRS = 0x00;
					SCON = 0x50;     				/*UART0 Mode1,REN=1,TI=1 */
					TMOD |= 0x20;    				/*Timer1 Mode1*/
					set_PCON_SMOD;        	/*UART0 Double Rate Enable*/
					set_CKCON_T1M;
					clr_T3CON_BRCK;        	/*Serial port 0 baud rate clock source = Timer1*/
					TH1 = 256 - (u32SysClock/16/u32Baudrate);
					set_TCON_TR1;
			break;
			case UART0_Timer3:
					SFRS = 0x00;
					SCON = 0x50;    				/*UART0 Mode1,REN=1,TI=1*/
					set_PCON_SMOD;        	/*UART0 Double Rate Enable*/
					T3CON &= 0xF8;   				/*T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)*/
					set_T3CON_BRCK;        	/*UART0 baud rate clock source = Timer3*/
					RH3    = HIBYTE(65536 - (u32SysClock/16/u32Baudrate));
					RL3    = LOBYTE(65536 - (u32SysClock/16/u32Baudrate));
					set_T3CON_TR3;          /*Trigger Timer3*/
			break;
			case UART1:
					SFRS = 0x00;
					S1CON = 0x50;   	      /*UART1 Mode1,REN_1=1 */
					T3CON = 0x88;   	      /*T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1), UART1 in MODE 1*/
					RH3    = HIBYTE(65536 - (u32SysClock/16/u32Baudrate));
					RL3    = LOBYTE(65536 - (u32SysClock/16/u32Baudrate));
					set_T3CON_TR3;         		//Trigger Timer3                                                             
			break; 
			case UART2:
					set_SC0CR0_SCEN;         /* Enable SC module */
					set_SC0CR1_UARTEN;       /* set as UART mode */
					set_SC0CR1_CLKKEEP;      
					clr_SC0CR0_CONSEL;       /* clr CONSEL and AUTOCEN*/
					clr_SC0CR0_AUTOCEN;
					SC0ETURD0 = LOBYTE(u32SysClock/8/u32Baudrate-1);  /* define baud rate low byte */
					SC0ETURD1 &= 0xF0;                                /* define baud rate high byte */
					SC0ETURD1 |= (HIBYTE(u32SysClock/8/u32Baudrate-1))&0x0F; 
					set_SC0CR1_PBOFF; 		//parity bit disable
					SC0CR1&=0XCF;				//datalegth 8bit
					set_SC0CR0_NSB;			//stop bit = 1bit
			break;	
			case UART3:
					set_SC0CR1_UARTEN;
					clr_SC0CR0_CONSEL;
					clr_SC0CR0_AUTOCEN;
					SC0ETURD0 = LOBYTE(u32SysClock/u32Baudrate-1);
					SC0ETURD1 = HIBYTE(u32SysClock/u32Baudrate-1)&0x0F;
					set_SC1CR1_PBOFF; 		//parity bit disable
					SC1CR1&=0XCF;					//datalegth 8bit
					set_SC1CR0_NSB;				//stop bit = 1bit
			break;		
	}
}
//****************************************************************************************************************	
//**** UART Receive data without interrupt  
//**** For example: UART_Open(UART0_Timer1,1200)
unsigned char UART_Receive_Data(unsigned char UARTPort)
{
    unsigned char c;
		SFRS = 0x00;
		switch (UARTPort)
		{
			case UART0:
				while (!RI);
				c = SBUF;
				RI = 0;
			break;
			case UART1:
				while (!RI_1);
				c = SBUF1;
				RI_1 = 0;
			break;
			case UART2:
				clr_SC0CR0_RXOFF;
				while((SC0TSR&SET_BIT1)==SET_BIT1);
				c = SC0DR;
			break;
			case UART3:
				clr_SC1CR0_RXOFF;
				while((SC1TSR&&SET_BIT1)==SET_BIT1);
				c = SC1DR;
			break;			
		}
		return (c);
}
//****************************************************************************************************************	
//**** UART transfer data without interrupt  
//**** For example: UART_Send_Data(UART0,0x55)
void UART_Send_Data(unsigned char UARTPort, unsigned char c)
{
		SFRS = 0x00;
		switch (UARTPort)
		{
			case UART0:
				SFRS = 0;
				TI = 0;
				SBUF = c;
				while(TI==0);
				TI = 0;
			break;
			case UART1:
				SFRS = 0;
				TI_1 = 0;
				SBUF1 = c;
				while(TI_1==0);
				TI_1 = 0;
			break;
			case UART2:
				clr_SC0CR0_TXOFF;
				SC0DR = c;
				while((SC0TSR|CLR_BIT3)==CLR_BIT3);
				clr_SC0CR0_TXOFF;
			break;
			case UART3:
				clr_SC1CR0_TXOFF;
				SFRS=2;
				SC1DR = c;
				while((SC1TSR|CLR_BIT3)==CLR_BIT3);
				clr_SC1CR0_TXOFF;
			break;
		}
}


/**
 * @brief       UART interrupt enable setting 
 * @param       u8UARTPort: UART0/UART1/UART2/UART3 baudrate value
 * @param       u8UARTINTStatus: Disable/Enable
 * @return      none
 * @details     none
 * @note        max baud rate = 1.5MHz when Fsys = 24MHz
 */
void UART_Interrupt_Enable(unsigned char u8UARTPort, unsigned char u8UARTINTStatus)
{
		switch (u8UARTPort)
		{
				case UART0: 
					switch(u8UARTINTStatus)
					{
						case Disable: clr_IE_ES; break;
						case Enable:  set_IE_ES; break;
					}
				break;
				case UART1:
					switch(u8UARTINTStatus)
					{
						case Disable: clr_EIE1_ES1; break;
						case Enable:  set_EIE1_ES1; break;
					}
				break;
				case UART2:
					switch(u8UARTINTStatus)
				  {
					  case Disable: clr_SC0IE_TBEIEN;
					                clr_SC0IE_RDAIEN;
													break;
					  case Enable: set_SC0IE_TBEIEN;
						             set_SC0IE_RDAIEN;
													break;
					}
				break;
				case UART3:
			   switch (u8UARTINTStatus)
					{
						case Disable: clr_SC1IE_TBEIEN;
							            clr_SC1IE_RDAIEN;
													break;
						case Enable: set_SC1IE_TBEIEN;
						             set_SC1IE_RDAIEN;
													break;
					}
				break;
		}
}

void UART0_LIRC_Baudrate2400_Open(void)
{
		SFRS = 0x00;
		SCON = 0x50;     				//UART0 Mode1,REN=1,TI=1
		TMOD |= 0x20;    				//Timer1 Mode1
		set_PCON_SMOD;        	//UART0 Double Rate Enable
		clr_CKCON_T1M;
		clr_T3CON_BRCK;        	//Serial port 0 baud rate clock source = Timer1
		TH1 = 255;
		set_TCON_TR1;
//		set_SCON_TI;					 //For printf function must setting TI = 1
}


