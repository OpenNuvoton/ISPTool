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

//****************************************************************************************************************	
//**** GPIO interrupt enable setting step
//**** 1. Select pin interrupt channel PIT0~PIT7
//**** 2. Select trig status HIGH,LEVEL / LOW,LEVEL / RISING,EDGE / FALLING,EDGE / BOTH,EDGE /
//**** 3. Define interrupt port (PORT, BIT)
//**** For example: GPIO_EnableInt(PIT0,BOTH,EDGE,Port1,0) means P1.0 falling edge trig pin intterrupt channel 0 (PIT0)
void GPIO_EnableInt(unsigned char u8PIT, unsigned char u8IntStatus,unsigned char u8IntMode, unsigned char u8Port, unsigned char u8Bit)
{
		switch ((u8IntMode<<4)+u8PIT)
		{
			case 0x00:  	clr_PICON_PIT0;	break;
			case 0x01:  	clr_PICON_PIT1;	break;
			case 0x02:  	clr_PICON_PIT2;	break;
			case 0x03:  	clr_PICON_PIT3;	break;
			case 0x04:  	clr_PICON_PIT4;	break;
			case 0x05:  	clr_PICON_PIT5;	break;
			case 0x06:  	clr_PICON_PIT6;	break;
			case 0x07:  	clr_PICON_PIT7;	break;
			
			case 0x10:  	set_PICON_PIT0;	break;
			case 0x11:  	set_PICON_PIT1;	break;
			case 0x12:  	set_PICON_PIT2;	break;
			case 0x13:  	set_PICON_PIT3;	break;
			case 0x14:  	set_PICON_PIT4;	break;
			case 0x15:  	set_PICON_PIT5;	break;
			case 0x16:  	set_PICON_PIT6;	break;
			case 0x17:  	set_PICON_PIT7;	break;
			default: break;
		}
		switch((u8IntStatus<<4)+u8PIT)
		{
			case 0x00: set_PINEN_PINEN0;clr_PIPEN_PIPEN0; break;
			case 0x01: set_PINEN_PINEN1;clr_PIPEN_PIPEN1; break;
			case 0x02: set_PINEN_PINEN2;clr_PIPEN_PIPEN2; break;
			case 0x03: set_PINEN_PINEN3;clr_PIPEN_PIPEN3; break;
			case 0x04: set_PINEN_PINEN4;clr_PIPEN_PIPEN4; break;
			case 0x05: set_PINEN_PINEN5;clr_PIPEN_PIPEN5; break;
			case 0x06: set_PINEN_PINEN6;clr_PIPEN_PIPEN6; break;
			case 0x07: set_PINEN_PINEN7;clr_PIPEN_PIPEN7; break; 
			
			case 0x10: clr_PINEN_PINEN0;set_PIPEN_PIPEN0; break;
			case 0x11: clr_PINEN_PINEN1;set_PIPEN_PIPEN1; break;
			case 0x12: clr_PINEN_PINEN2;set_PIPEN_PIPEN2; break;
			case 0x13: clr_PINEN_PINEN3;set_PIPEN_PIPEN3; break;
			case 0x14: clr_PINEN_PINEN4;set_PIPEN_PIPEN4; break;
			case 0x15: clr_PINEN_PINEN5;set_PIPEN_PIPEN5; break;
			case 0x16: clr_PINEN_PINEN6;set_PIPEN_PIPEN6; break;
			case 0x17: clr_PINEN_PINEN7;set_PIPEN_PIPEN7; break;  
			
			case 0x20: set_PINEN_PINEN0;set_PIPEN_PIPEN0; break; 
			case 0x21: set_PINEN_PINEN1;set_PIPEN_PIPEN1; break; 
			case 0x22: set_PINEN_PINEN2;set_PIPEN_PIPEN2; break; 
			case 0x23: set_PINEN_PINEN3;set_PIPEN_PIPEN3; break; 
			case 0x24: set_PINEN_PINEN4;set_PIPEN_PIPEN4; break; 
			case 0x25: set_PINEN_PINEN5;set_PIPEN_PIPEN5; break; 
			case 0x26: set_PINEN_PINEN6;set_PIPEN_PIPEN6; break; 
			case 0x27: set_PINEN_PINEN7;set_PIPEN_PIPEN7; break; 
 
			default: break;			
		}                                            
		switch(u8PIT)                                
		{                                            
			  case PIT0:PIPS0=0x00;PIPS0=(u8Port<<4)+(u8Bit&0x0F);break;      
        case PIT1:PIPS1=0x00;PIPS1=(u8Port<<4)+(u8Bit&0x0F);break;                    
        case PIT2:PIPS2=0x00;PIPS2=(u8Port<<4)+(u8Bit&0x0F);break;  
        case PIT3:PIPS3=0x00;PIPS3=(u8Port<<4)+(u8Bit&0x0F);break;   
        case PIT4:PIPS4=0x00;PIPS4=(u8Port<<4)+(u8Bit&0x0F);break;   
        case PIT5:PIPS5=0x00;PIPS5=(u8Port<<4)+(u8Bit&0x0F);break;
				case PIT6:PIPS6=0x00;PIPS6=(u8Port<<4)+(u8Bit&0x0F);break;
				case PIT7:PIPS7=0x00;PIPS7=(u8Port<<4)+(u8Bit&0x0F);break;   
				default: break;
		}
		set_EIE0_EPI;				//Pin interrupt enable bit
}
