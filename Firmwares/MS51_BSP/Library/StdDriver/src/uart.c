/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/*  Website: http://www.nuvoton.com                                                                        */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/
#include "MS51.h"

/*MS51 new version buadrate */

void UART_Open(unsigned long u32SysClock, unsigned char u8UARTPort,unsigned long u32Baudrate)
{
  switch(u8UARTPort)
  {
    case UART0_Timer1:
          P06_PUSHPULL_MODE;    //Setting UART pin as Quasi mode for transmit
          P07_INPUT_MODE;    //Setting UART pin as Quasi mode for transmit  
          SCON = 0x50;       //UART0 Mode1,REN=1,TI=1
          TMOD |= 0x20;      //Timer1 Mode1
          set_PCON_SMOD;          //UART0 Double Rate Enable
          set_CKCON_T1M;
          clr_T3CON_BRCK;          //Serial port 0 baud rate clock source = Timer1
          TH1 = 256 - (u32SysClock/16/u32Baudrate);
          set_TCON_TR1;
      break;
      
      case UART0_Timer3:
          P06_PUSHPULL_MODE;    //Setting UART pin as Quasi mode for transmit
          P07_INPUT_MODE;    //Setting UART pin as Quasi mode for transmit  
          SCON = 0x50;     //UART0 Mode1,REN=1,TI=1
          set_PCON_SMOD;        //UART0 Double Rate Enable
          T3CON &= 0xF8;   //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
          set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3
          RH3    = HIBYTE(65536 - (u32SysClock/16/u32Baudrate));  
          RL3    = LOBYTE(65536 - (u32SysClock/16/u32Baudrate));  
          set_T3CON_TR3;         //Trigger Timer3
      break;
      
      case UART1_Timer3:
          P16_PUSHPULL_MODE;  //Setting UART pin as Quasi mode for transmit
          P02_INPUT_MODE;  //Setting RXD_1 pin as Input mode for transmit   
          SCON_1 = 0x50;     //UART1 Mode1,REN_1=1,TI_1=1
          T3CON = 0x80;     //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1), UART1 in MODE 1
          RH3    = HIBYTE(65536 - (u32SysClock/16/u32Baudrate));  
          RL3    = LOBYTE(65536 - (u32SysClock/16/u32Baudrate));     
          set_T3CON_TR3;             //Trigger Timer3                                                             
      break; 
  }
}

unsigned char Receive_Data(unsigned char UARTPort)
{
    UINT8 c;
    switch (UARTPort)
    {
      case UART0:
        while (!RI);
        c = SBUF;
        RI = 0;
      break;
      case UART1:
        while (!RI_1);
        c = SBUF_1;
        RI_1 = 0;
      break;
    }
    return (c);
}

void UART_Send_Data(UINT8 UARTPort, UINT8 c)
{
    switch (UARTPort)
    {
      case UART0:
        TI = 0;
        SBUF = c;
        while(TI==0);
      break;
      case UART1:
        TI_1 = 0;
        SBUF_1 = c;
        while(TI_1==0);
      break;
    }
}

