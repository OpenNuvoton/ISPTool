/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/09/02 3:49p $
 * @brief    NuEdu Basic01 UART printf Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "diskio.h"
#include "NUC230_240.h"
#include "ISP_BSP.h"

#include "HID_Transfer_and_MSC.h"
#include "cmd.h"
#include "ISP_UART0.h"
#include "massstorage.h"
extern void ISP_Procoess(void);
void RTC_IRQHandler(void)
{
	  unsigned char Buff[256];
	
	 S_RTC_TIME_DATA_T  sReadRTC;
    /* To check if RTC Tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1) {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();
		    RTC_GetDateAndTime(&sReadRTC);	
			memset(Buff, '\0', sizeof(Buff)); 
sprintf((char *)Buff,"Time: %d/%02d/%02d %02d:%02d:%02d",
         sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

ILI9341_LCD_PutString_line(6,0,Buff,Red,Yellow);
    }
}


	//#define APROM_OFFSET 16*1024

uint32_t UI_IsKeyPressed()
{
	uint32_t bKeyPressed = ((PB14 == 0) ? TRUE : FALSE);
	return bKeyPressed;
}
	unsigned int i;
int main()
{			
  unsigned char Buff[256];

	TEST_PIN_INIT();
	#if 0
		
 IO_TEST_MODE();
	#endif
	 	
  ISP_BSP_SYSTEM_CLOCK_INITIAL();		
	
	
  Initial_UART2_Debug();
  dbg_printf("this is debug printer!!\n\r");
	LCD_initial();
	dsp_single_color(Yellow);
#if 1
  Open_ACMP1();
if(Get_ACMP1()==1)  //no extnal power;
{
	ILI9341_LCD_PutString_line(0,0,(unsigned char *)"ISP Board Power",Red,Yellow);
	PWC_C_ENABLE; //enable power
}
else
{
	ILI9341_LCD_PutString_line(0,0,(unsigned char *)"External Power in",Red,Yellow);
	PWC_C_DISABLE; //enable power	
}
#endif
	Open_ADC0();
	TVCC_voltage_new=0;
  memset(Buff, '\0', sizeof(Buff)); 
  sprintf((char *)Buff, "System Voltage %.2f V", TVCC_voltage_new);
  ILI9341_LCD_PutString_line(1,0,Buff,Red,Yellow);
#if 0
  Initial_UART0_RS485(); //for rs485 and rs232 isp. PB0=>RXD, PB1=>TXD, PB2=>RTS
  Open_SPI2();//for target chip use spi2
  OPEN_CAN0();//for target chip use can0	
  I2C0_INIT();//for target chip use I2C0
  INIT_UART_1WIRE();
#endif


  Initial_Key_Input();
//for spi loop test
#if 0
while(1)
{
		for(i=0;i<64;i++)
		{
		rcvbuf[i]=0;
		sendbuf[i]=(64-i);
		}
  SPI_MasterSendDataT1();
	SPI_MasterRcvDataT1();
		for(i=0;i<64;i++)
		{
				if(rcvbuf[i]!=(64-i))
			while(1);
			}
				CLK_SysTickDelay(20000);
}
#endif
	RTC_Init();
	RTC_SET();

	Open_SPI_Flash();
	if(SpiFlash_ReadMidDid()!=0xef14)
	{
	ILI9341_LCD_PutString_line(2,0,(unsigned char *)"SPI FLASH ID FALSE!!",Red,Yellow);
	}
	else
  {
	ILI9341_LCD_PutString_line(2,0,(unsigned char *)"SPI FLASH ID PASS!!",Red,Yellow);
	}

if(Check_SDCARD()!=0)
{
ILI9341_LCD_PutString_line(3,0,(unsigned char *)"The SDCARD can't access",Red,Yellow);
}
else
{
ILI9341_LCD_PutString_line(3,0,(unsigned char *)"The SDCARD access Pass",Red,Yellow);
}

#if 1
memset(Buff, '\0', sizeof(Buff)); 
file_totallen=GET_FILE_SIZE();
sprintf((char *)Buff, "File Size :%d Byte",file_totallen);
ILI9341_LCD_PutString_line(4,0,Buff,Red,Yellow);

memset(Buff, '\0', sizeof(Buff)); 
file_checksum=GET_FILE_CHECKSUM();
sprintf((char *)Buff, "File Checksum: 0x%x", file_checksum);
ILI9341_LCD_PutString_line(5,0,Buff,Red,Yellow);
#endif

 
//write_log("test1");
//write_log("test2");

		Initial_USB();		  										
while(1)
{
	  ISP_Procoess();
		MSC_ProcessCmd();
		//OFFLINE PROCESS
		if (UI_IsKeyPressed() == TRUE)
		{
			USBD_ENABLE_INT(0);
			if (UI_IsKeyPressed() == TRUE)
			{
				ILI9341_LCD_PutString_line(7,0,"Start offline programmer",Red,Yellow);
						LCD_Fill(0,16*8,320,240,Yellow);
             CmdUpdateAprom(FALSE);			         //offline spi write
			}
			/* Enable USB-related interrupts. */
			USBD_ENABLE_INT(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
		}
		
		#if 0
		//loop to detect power 
		if(Get_ACMP1()==1)  //no extnal power;
{
	ILI9341_LCD_PutString_line(0,0,"ISP Board Power",Red,Yellow);
	PWC_C_ENABLE; //enable power
}
else
{
	ILI9341_LCD_PutString_line(0,0,"External Power in",Red,Yellow);
	PWC_C_DISABLE; //enable power	
}
#endif
  //loop to show power 
	//TVCC_voltage=Get_ADC0_Value()*(0.000805*2.0);
  Poll_ADC0();
if (TVCC_voltage_new!=TVCC_voltage_old)
 {
	TVCC_voltage_new=TVCC_voltage_new*(0.000805*2.0);
  
  memset(Buff, '\0', sizeof(Buff)); 
  sprintf((char *)Buff, "System Voltage %.2f V", TVCC_voltage_new);
  ILI9341_LCD_PutString_line(1,0,Buff,Red,Yellow);
	 TVCC_voltage_old=TVCC_voltage_new;
  }
}

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
