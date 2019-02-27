/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//***********************************************************************************************************

#include "MS51.h"

 
/**
  * @brief Timer0 delay setting 
  * @param[in] u32SYSCLK define Fsys clock value. for example 24000000, use the real Fsys value.
  *                       - \ref 24000000 (use HIRC 24MHz)
  *                       - \ref 16000000 (use HIRC 16MHz)
  * @param[in] u8CNT define count time.
  * @param[in] u6DLYUnit define delay time base is us. From 1~10000, the maxima value please not over 10000.
  *                       - \ref 1000 (1ms)
  *                       - \ref 100 (100us)
  * @return  None
  * @note    If use LIRC or LXT as Fsys need adjust this marco.
  * @exmaple :  Timer0_Delay(16000000,200,1000);
*/
void Timer0_Delay(unsigned long u32SYSCLK, unsigned int u16CNT, unsigned int u16DLYUnit)
{
    clr_CKCON_T0M;                                  //T0M=0, Timer0 Clock = Fsys/12
    TMOD |= 0x01;                                   //Timer0 is 16-bit mode
    while (u16CNT != 0)
    {
      TL0 = LOBYTE(65535-((u32SYSCLK/1000000)*u16DLYUnit/12));
      TH0 = HIBYTE(65535-((u32SYSCLK/1000000)*u16DLYUnit/12));
      set_TCON_TR0;                                    //Start Timer0
      while (TF0 != 1);                       //Check Timer0 Time-Out Flag
      clr_TCON_TF0;
      u16CNT --;
    }
    clr_TCON_TR0;                                     //Stop Timer0
}

/**
  * @brief Timer1 delay setting 
  * @param[in] u32SYSCLK define Fsys clock value. for example 24000000, use the real Fsys value.
  *                       - \ref 24000000 (use HIRC)
  *                       - \ref 22118400 (use HXT)
  * @param[in] u8CNT define count time.
  * @param[in] u6DLYUnit define delay time base is us. From 1~10000, the maxima value please not over 10000.
  *                       - \ref 1000 (1ms)
  *                       - \ref 100 (100us)
  * @return  None
  * @note    If use LIRC or LXT as Fsys need adjust this marco.
  * @exmaple :  Timer1_Delay(24000000,5,1000);
*/
void Timer1_Delay(unsigned long u32SYSCLK, unsigned int u16CNT, unsigned int u16DLYUnit)
{
    clr_CKCON_T1M;                              //T1M=0, Timer1 Clock = Fsys/12
    TMOD |= 0x10;                                //Timer1 is 16-bit mode
    while (u16CNT != 0)
    {
      TL1 = LOBYTE(65536-(u16DLYUnit/12*(u32SYSCLK/1000000)));
      TH1 = HIBYTE(65536-(u16DLYUnit/12*(u32SYSCLK/1000000)));
      set_TCON_TR1;                                //Start Timer1
      while (TF1 != 1);                       //Check Timer0 Time-Out Flag
      clr_TCON_TF1;
      u16CNT --;
    }
    clr_TCON_TR1;                             //Stop Timer1
}

/**
  * @brief Timer2 delay setting 
  * @param[in] u32SYSCLK define Fsys clock value. for example 24000000, use the real Fsys value.
  *                       - \ref 24000000 (use HIRC)
  *                       - \ref 22118400 (use HXT for example)
  * @param[in] u16TMDIV define timer2 clock devider value from 1\4\16\32\64\128\256\512 detail check datasheet T2MOD.
  *                       - \ref 512
  * @param[in] u16CNT define total count times.
  * @param[in] u6DLYUnit define delay time base is us. From 1~1000000, please consider the value with devider.
  *                       - \ref 1000 (1ms)
  *                       - \ref 100 (100us)
  * @return  None
  * @note    If use LIRC or LXT as Fsys need adjust this marco.
  * @exmaple :  Timer2_Delay(24000000,128,5,1000);
*/
void Timer2_Delay(unsigned long u32SYSCLK,unsigned int u16TMDIV, unsigned int u16CNT, unsigned long u32DLYUnit)
{
    SFRS = 0x00;
    switch (u16TMDIV)
    {
      case 1:T2MOD&=0x8F; break;
      case 4:T2MOD&=0x8F;T2MOD|=0x10; break;
      case 16:T2MOD&=0x8F;T2MOD|=0x20; break;
      case 32:T2MOD&=0x8F;T2MOD|=0x30; break;
      case 64:T2MOD&=0x8F;T2MOD|=0x40; break;
      case 128:T2MOD&=0x8F;T2MOD|=0x50; break;
      case 256:T2MOD&=0x8F;T2MOD|=0x60; break;
      case 512:T2MOD&=0x8F;T2MOD|=0x70; break;
    }
    clr_T2CON_CMRL2;                                  //Timer 2 as auto-reload mode
    set_T2MOD_LDEN;
    set_T2MOD_CMPCR;                                  //Timer 2 value is auto-cleared as 0000H when a compare match occurs.
    while (u16CNT != 0)
    {
      TL2 = LOBYTE(65536-((u32SYSCLK/1000000)*u32DLYUnit/u16TMDIV));
      TH2 = HIBYTE(65536-((u32SYSCLK/1000000)*u32DLYUnit/u16TMDIV));
      set_T2CON_TR2;                                    //Start Timer2
      while (TF2!=1);            //Check Timer2 Time-Out Flag
      clr_T2CON_TF2;
      u16CNT --;
    }
    clr_T2CON_TR2;                                    //Stop Timer2
}
/**
  * @brief Timer3 delay setting 
  * @param[in] u32SYSCLK define Fsys clock value. for example 24000000, use the real Fsys value.
  *                       - \ref 24000000 (use HIRC)
  *                       - \ref 22118400 (use HXT for example)
  * @param[in] u8TMDIV define timer2 clock devider value from 1\2\4\8\16\32\64\128 detail check datasheet T3MOD.
  *                       - \ref 512
  * @param[in] u16CNT define total count times.
  * @param[in] u32DLYUnit define delay time base is us. From 1~1000000, please consider the value with devider.
  *                       - \ref 1000 (1ms)
  *                       - \ref 100 (100us)
  * @return  None
  * @note    If use LIRC or LXT as Fsys need adjust this marco.
  * @exmaple :  Timer3_Delay(24000000,16,5,1000);
*/
void Timer3_Delay(unsigned long u32SYSCLK,unsigned char u8TMDIV, unsigned int u16CNT, unsigned long u32DLYUnit)
{
    SFRS = 0x00;
    switch (u8TMDIV)
    {
      case 1:T3CON&=0xF8; break;
      case 2:T3CON&=0xF8;T3CON|=0x01; break;
      case 4:T3CON&=0xF8;T3CON|=0x02; break;
      case 8:T3CON&=0xF8;T3CON|=0x03; break;
      case 16:T3CON&=0xF8;T3CON|=0x04; break;
      case 32:T3CON&=0xF8;T3CON|=0x05; break;
      case 64:T3CON&=0xF8;T3CON|=0x06; break;
      case 128:T3CON&=0xF8;T3CON|=0x07; break;
    }
    while (u16CNT != 0)
    {
      clr_T3CON_TF3;
      RL3 = LOBYTE(65536-((u32SYSCLK/1000000)*u32DLYUnit/u8TMDIV));
      RH3 = HIBYTE(65536-((u32SYSCLK/1000000)*u32DLYUnit/u8TMDIV));
      set_T3CON_TR3;                                    //Trigger Timer3
      while ((T3CON|CLR_BIT4)==CLR_BIT4);    //Check Timer3 Time-Out Flag
      clr_T3CON_TF3;
      u16CNT --;
    }
    clr_T3CON_TR3;                                    //Stop Timer3
}


//****************************************************************************************************************  
//**** Timer Interrupt enable setting  
//**** 1. Delay value
//**** 2. Define unit
//**** For example: Timer3_Delay(5,UNIT_100US) = Delay 100us
void Timer_Interrupt_Enable(unsigned char u8TM)
{
    switch(u8TM)
    {
      case TIMER0: set_IE_ET0;break;
      case TIMER1: set_IE_ET1;break;
      case TIMER2: set_EIE_ET2;break;
      case TIMER3: set_EIE1_ET3;break;
    }
}


