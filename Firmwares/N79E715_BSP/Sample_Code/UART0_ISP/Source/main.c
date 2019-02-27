/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2015 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2015
//***********************************************************************************************************

//***********************************************************************************************************
//  File Function: N76E885 UART-0 Mode2 demo code
//***********************************************************************************************************

#include <stdio.h>
#include "N79E715.h"
#include "Typedef.h"
#include "Define.h"
#include "SFR_Macro.h"
#include "UART0_transfer.h"
#include "ISP_USER.h"

unsigned char temp0;
unsigned char idata temp1  _at_ 0x90;
unsigned char xdata temp2;



void Package_checksum(void)
{
	g_checksum=0;
   for(count=0;count<64;count++)
	{
    g_checksum =g_checksum+ uart_rcvbuf[count];    
	}
	uart_txbuf[0]=g_checksum&0xff;
	uart_txbuf[1]=(g_checksum>>8)&0xff;
	uart_txbuf[4]=uart_rcvbuf[4]+1;
	uart_txbuf[5]=uart_rcvbuf[5];
	if(uart_txbuf[4]==0x00)
	uart_txbuf[5]++;
	
}

 
/************************************************************************************************************
*    Main function 
************************************************************************************************************/
void main (void)
{
   //initial GPIO for quasi mode.  
	 P0M1 = 0x00;
   P0M2 = 0x00;
   P1M1 = 0x00;
   P1M2 = 0x00;
   P2M1 = 0x00;
   P2M2 = 0x00;
   P3M1 = 0x00;
   P3M2 = 0x00;
	 //uart initial 
	
		UART0_ini();
		TM0_ini();

		g_timer0Over=0;
		g_timer0Counter=5000;
		g_progarmflag=0;

while(1)
{
	    	if(bUartDataReady == TRUE)
				{
					EA=0; //DISABLE ALL INTERRUPT									

					if(g_progarmflag==1)
					{
						for(count=8;count<64;count++)
						{
							ISPCN = BYTE_PROGRAM_AP;					//program byte
							ISPAL = flash_address&0xff;
							ISPAH = (flash_address>>8)&0xff;
							ISPFD=uart_rcvbuf[count];
							TA=0xAA;		//trigger IAP
							TA=0x55;
							ISPTRG |= SET_BIT0 ;   

					
							ISPCN = BYTE_READ_AP;							//program byte verify
							TA=0xAA;			//trigger IAP
							TA=0x55;
							ISPTRG |= SET_BIT0 ;  
							if(ISPFD!=uart_rcvbuf[count])
							while(1);													
							if (CHPCON==0x43)							//if error flag set, program error stop ISP
							while(1);
							
							g_totalchecksum=g_totalchecksum+uart_rcvbuf[count];
							flash_address++;
	
							if(flash_address==AP_size)
							{
								g_progarmflag=0;
								 goto END_2;					
							}
						} 
END_2:	        			
						Package_checksum();
						uart_txbuf[8]=g_totalchecksum&0xff;
						uart_txbuf[9]=(g_totalchecksum>>8)&0xff;
						Send_64byte_To_UART0();
    				
					}
						
					switch(uart_rcvbuf[0])
					{								
						case CMD_CONNECT:
						case CMD_SYNC_PACKNO:
						{
							Package_checksum();
							Send_64byte_To_UART0();		
							g_timer0Counter=0; //clear timer 0 for no reset
							g_timer0Over=0;
							
							TA=0xAA;						//set_IAPEN;
							TA=0x55;
							CHPCON|=SET_BIT0;
						
						break;
						}
												
						case CMD_GET_FWVER:						
						{
							Package_checksum();
							uart_txbuf[8]=FW_VERSION;	
							Send_64byte_To_UART0();	
						break;
						}
            
						case CMD_RUN_APROM:						
						{
						//Package_checksum();
						//uart_txbuf[8]=FW_VERSION;	
						//Send_64byte_To_UART0();	
							goto _APROM;
						break;
						}
		
						//please tool must define devices value
						case CMD_GET_DEVICEID:						
						{
							Package_checksum();
							uart_txbuf[8]=0x00;	
							uart_txbuf[8]=0x5a;	
							uart_txbuf[8]=0x00;	
							uart_txbuf[8]=0x20;	
							Send_64byte_To_UART0();	
						break;
						}
	
						case CMD_READ_CONFIG:						
						{
							Package_checksum();
							uart_txbuf[8]=0x7f;	
							uart_txbuf[9]=0xff;	
							uart_txbuf[10]=0xff;	
							uart_txbuf[11]=0xff;	
							uart_txbuf[12]=0xff;	
							uart_txbuf[13]=0xff;	
							uart_txbuf[14]=0xff;						
							uart_txbuf[15]=0xff;
							Send_64byte_To_UART0();	
						break;
						}
						
						case CMD_UPDATE_APROM:						
						{
						
							TA=0xAA;	//set_IAPEN;		
							TA=0x55;
							CHPCON |= SET_BIT0 ;	

							ISPFD = 0xFF;    			//Erase must set ISPFD = 0xFF
							ISPCN = PAGE_ERASE_AP;
							
							for(flash_address=0x0000;flash_address<APROM_SIZE/PAGE_SIZE;flash_address++)
							{        
								ISPAL = LOBYTE(flash_address*PAGE_SIZE);
								ISPAH = HIBYTE(flash_address*PAGE_SIZE);
								TA=0xAA;			//trigger IAP
								TA=0x55;
								ISPTRG |= SET_BIT0 ;
							}						
							
							g_totalchecksum=0;
							flash_address=0;
							AP_size=0;
							AP_size=uart_rcvbuf[12];
							AP_size|=(uart_rcvbuf[13]<<8);	
							g_progarmflag=1;

							for(count=16;count<64;count++)
							{
								ISPCN = BYTE_PROGRAM_AP;
								ISPAL = flash_address&0xff;
								ISPAH = (flash_address>>8)&0xff;
								ISPFD=uart_rcvbuf[count];
								//trigger IAP
								TA = 0xAA;
								TA = 0x55;
								ISPTRG |= SET_BIT0;                                 
			
								ISPCN = BYTE_READ_AP;								//program byte verify
								//trigger IAP
								TA=0xAA;
								TA=0x55;
								ISPTRG |= SET_BIT0 ;                               
								if(ISPFD!=uart_rcvbuf[count])
								while(1);					
								if (CHPCON==0x43)								//if error flag set, program error stop ISP
								while(1);
								
								g_totalchecksum=g_totalchecksum+uart_rcvbuf[count];
								flash_address++;
								
								if(flash_address==AP_size)
								{
									g_progarmflag=0;
									 goto END_1;					
								}
							}
END_1:	        			
							Package_checksum();
							uart_txbuf[8]=g_totalchecksum&0xff;
							uart_txbuf[9]=(g_totalchecksum>>8)&0xff;
							Send_64byte_To_UART0();	
							break;
						}
						
					}	
			    bUartDataReady = FALSE;
				  bufhead = 0;						
					EA=1;
			}
			//For connect timer out	

      if(g_timer0Over==1)
			{		
				goto _APROM;
			}

			
	    //for uart time out or buffer error
     	if(g_timer1Over==1)
			{			 
			 if((bufhead<64)&&(bufhead>0)||(bufhead>64))
			   {
				 bufhead=0;				 
			   }
			}	
	
	
}   



_APROM:

		TA = 0xAA;
		TA = 0x55;
		CHPCON = 0x00;                  //set boot from AP
		TA = 0xAA;
		TA = 0x55;
		CHPCON = 0x80;                   //software reset enable

	  /* Trap the CPU */
		while(1);	
}


