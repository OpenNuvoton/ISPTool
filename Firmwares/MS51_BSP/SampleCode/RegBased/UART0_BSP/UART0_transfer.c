/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Nov/6/2017
//***********************************************************************************************************

//***********************************************************************************************************
//  File Function: N76E003 UART-0 ISP demo code UART transfer
//***********************************************************************************************************

#include "MS51.h"
#include "uart0_transfer.h"

  xdata volatile uint8_t uart_rcvbuf[64]; 
  xdata volatile uint8_t uart_txbuf[64];
  data volatile uint8_t bufhead;
  data volatile uint16_t flash_address; 
  data volatile uint16_t AP_size;
  data volatile uint8_t g_timer1Counter;
  data volatile uint8_t count; 
  data volatile uint16_t g_timer0Counter;
  data volatile uint32_t g_checksum;
  data volatile uint32_t g_totalchecksum;
  bit volatile bUartDataReady;
  bit volatile g_timer0Over;
  bit volatile g_timer1Over;
  bit volatile g_progarmflag;

unsigned char PID_highB,PID_lowB,DID_highB,DID_lowB,CONF0,CONF1,CONF2,CONF4;
unsigned char recv_CONF0,recv_CONF1,recv_CONF2,recv_CONF4;
bit BIT_TMP;


void READ_ID(void)
{
    set_CHPCON_IAPEN;
    IAPCN = READ_DID;
    IAPAH = 0x00;
    IAPAL = 0x00;
    set_IAPTRG_IAPGO;
    DID_lowB = IAPFD;
    IAPAL = 0x01;
    set_IAPTRG_IAPGO;
    DID_highB = IAPFD;
    IAPAL = 0x02;
    set_IAPTRG_IAPGO;
    PID_lowB = IAPFD;
    IAPAL = 0x03;
    set_IAPTRG_IAPGO;
    PID_highB = IAPFD;
}
void READ_CONFIG(void)
{
    IAPCN = BYTE_READ_CONFIG;
    IAPAL = 0x00;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    CONF0 = IAPFD;
    IAPAL = 0x01;
    set_IAPTRG_IAPGO;
    CONF1 = IAPFD;
    IAPAL = 0x02;
    set_IAPTRG_IAPGO;
    CONF2 = IAPFD;
    IAPAL = 0x04;
    set_IAPTRG_IAPGO;
    CONF4 = IAPFD;
    clr_CHPCON_IAPEN;
}

void UART0_ini_115200(void)
{
		P06_QUASI_MODE;																	
		P07_QUASI_MODE;
		SFRS = 0x00;
		SCON = 0x50;    				/*UART0 Mode1,REN=1,TI=1*/
		set_PCON_SMOD;        	/*UART0 Double Rate Enable*/
		T3CON &= 0xF8;   				/*T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)*/
		set_T3CON_BRCK;        	/*UART0 baud rate clock source = Timer3*/
		RH3    = HIBYTE(65536 - 13);
		RL3    = LOBYTE(65536 - 13);
		set_T3CON_TR3;          /*Trigger Timer3*/
		ES=1;
		EA=1;
}

void Send_64byte_To_UART0(void)
{
   for(count=0;count<64;count++)
  {
     TI = 0;  
    SBUF = uart_txbuf[count];
    while(TI==0);
  }
}

void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}


void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

 void TM0_ini(void)
{    
  TH0=TL0=0;    //interrupt timer 140us
  set_TCON_TR0;      //Start timer0
  set_IPH_PSH;       // Serial port 0 interrupt level2
  set_IE_ET0;
}

void Serial_ISR (void) interrupt 4 
{
    if (RI == 1)
    {   
      uart_rcvbuf[bufhead++]=  SBUF;    
      clr_RI;                                         // Clear RI (Receive Interrupt).
    }
    if (TI == 1)
    {       
        clr_TI;                                         // Clear TI (Transmit Interrupt).
    }
    if(bufhead ==1)
    {
      g_timer1Over=0;
      g_timer1Counter=90; //for check uart timeout using
    }
  if(bufhead == 64)
    {
      
      bUartDataReady = TRUE;
      g_timer1Counter=0;
      g_timer1Over=0;
      bufhead = 0;
    }    
}

void Timer0_ISR (void) interrupt 1
{
if(g_timer0Counter)
  {
  g_timer0Counter--;
    if(!g_timer0Counter)
    {
    g_timer0Over=1;
    }
  }
  
  if(g_timer1Counter)
  {
  g_timer1Counter--;
    if(!g_timer1Counter)
    {
    g_timer1Over=1;
    }
  }
} 

