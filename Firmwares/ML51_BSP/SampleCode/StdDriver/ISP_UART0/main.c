/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


//***********************************************************************************************************
//  File Function: ML51 UART0 ISP demo code
//***********************************************************************************************************
#include "ML51.H"

/************************************************************************************************************
*    Main function 
************************************************************************************************************/
void main (void)
{
   //uart initial for ISP programmer GUI, always use 115200 baudrate
  UART0_ini_115200();
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
              IAPCN = BYTE_PROGRAM_AP;          //program byte
              IAPAL = flash_address&0xff;
              IAPAH = (flash_address>>8)&0xff;
              IAPFD=uart_rcvbuf[count];
              set_IAPTRG_IAPGO;
          
              IAPCN = BYTE_READ_AP;              //program byte verify
              set_IAPTRG_IAPGO;
              if(IAPFD!=uart_rcvbuf[count])
              while(1);                          
              if (CHPCON==0x43)              //if error flag set, program error stop ISP
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
              goto _APROM;
            break;
            }
    
            //please for ISP programmer GUI, ID always use following rule to transmit.
            case CMD_GET_DEVICEID:            
            {
              READ_ID();
              Package_checksum();
              uart_txbuf[8]=DID_lowB;  
              uart_txbuf[9]=DID_highB;  
              uart_txbuf[10]=PID_lowB;  
              uart_txbuf[11]=PID_highB;  
              Send_64byte_To_UART0();  
            break;
            }
            case CMD_ERASE_ALL:
            {
              set_CHPCON_IAPEN;
              set_IAPUEN_APUEN;
              IAPFD = 0xFF;          //Erase must set IAPFD = 0xFF
              IAPCN = PAGE_ERASE_AP;
              
              for(flash_address=0x0000;flash_address<APROM_SIZE/PAGE_SIZE;flash_address++)
              {        
                IAPAL = LOBYTE(flash_address*PAGE_SIZE);
                IAPAH = HIBYTE(flash_address*PAGE_SIZE);
                set_IAPTRG_IAPGO;
              }            
              
              Package_checksum();
              Send_64byte_To_UART0();  
              break;
            }
            case CMD_READ_CONFIG:            
            {
              READ_CONFIG();
              Package_checksum();
              uart_txbuf[8]=CONF0;  
              uart_txbuf[9]=CONF1;  
              uart_txbuf[10]=CONF2;  
              uart_txbuf[11]=0xff;  
              uart_txbuf[12]=CONF4;  
              uart_txbuf[13]=0xff;  
              uart_txbuf[14]=0xff;            
              uart_txbuf[15]=0xff;
              Send_64byte_To_UART0();  
            break;
            }
            
            case CMD_UPDATE_CONFIG:
            {
              recv_CONF0 = uart_rcvbuf[8];
              recv_CONF1 = uart_rcvbuf[9];
              recv_CONF2 = uart_rcvbuf[10];
              recv_CONF4 = uart_rcvbuf[12];
/*Erase CONFIG */              
              set_CHPCON_IAPEN;
              set_IAPUEN_CFUEN;
              IAPCN = PAGE_ERASE_CONFIG;
              IAPAL = 0x00;
              IAPAH = 0x00;
              IAPFD = 0xFF;
              set_IAPTRG_IAPGO;
/*Program CONFIG*/  
              IAPCN = BYTE_PROGRAM_CONFIG;
              IAPAL = 0x00;
              IAPAH = 0x00;
              IAPFD = recv_CONF0;
              set_IAPTRG_IAPGO;
              IAPFD = recv_CONF1;
              IAPAL = 0x01;
              set_IAPTRG_IAPGO;
              IAPAL = 0x02;
              IAPFD = recv_CONF2;
              set_IAPTRG_IAPGO;
              IAPAL = 0x04;
              IAPFD = recv_CONF4;
              set_IAPTRG_IAPGO;
              clr_IAPUEN_CFUEN;
/*Read new CONFIG*/  
              READ_CONFIG();
              
              Package_checksum();
              uart_txbuf[8]=CONF0;  
              uart_txbuf[9]=CONF1;  
              uart_txbuf[10]=CONF2;  
              uart_txbuf[11]=0xff;  
              uart_txbuf[12]=CONF4;  
              uart_txbuf[13]=0xff;  
              uart_txbuf[14]=0xff;            
              uart_txbuf[15]=0xff;
              Send_64byte_To_UART0();  
              break;
            }
            
            case CMD_UPDATE_APROM:            
            {
              set_CHPCON_IAPEN;
              set_IAPUEN_APUEN;
              IAPFD = 0xFF;          //Erase must set IAPFD = 0xFF
              IAPCN = PAGE_ERASE_AP;
              
              for(flash_address=0x0000;flash_address<APROM_SIZE/PAGE_SIZE;flash_address++)
              {        
                IAPAL = LOBYTE(flash_address*PAGE_SIZE);
                IAPAH = HIBYTE(flash_address*PAGE_SIZE);
                set_IAPTRG_IAPGO;
              }            
              
              g_totalchecksum=0;
              flash_address=0;
              AP_size=0;
              AP_size=uart_rcvbuf[12];
              AP_size|=(uart_rcvbuf[13]<<8);  
              g_progarmflag=1;

              for(count=16;count<64;count++)
              {
                IAPCN = BYTE_PROGRAM_AP;
                IAPAL = flash_address&0xff;
                IAPAH = (flash_address>>8)&0xff;
                IAPFD=uart_rcvbuf[count];
                clr_CHPCON_IAPFF;
                set_IAPTRG_IAPGO;                              
      
                IAPCN = BYTE_READ_AP;                //program byte verify
                set_IAPTRG_IAPGO;                          
                if(IAPFD!=uart_rcvbuf[count])
                while(1);          
                if (CHPCON==0x43)                //if error flag set, program error stop ISP
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
    SFRS = 0;
    TA = 0xAA;
    TA = 0x55;
    CHPCON = 0x00;                  //set boot from AP
    TA = 0xAA;
    TA = 0x55;
    CHPCON = 0x80;    

    /* Trap the CPU */
    while(1);  
}


