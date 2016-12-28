#include <stdio.h>
#include <string.h>
#include "NUC230_240.h"
#include "cmd.h"
#include "ISP_BSP.h"
#include "ff.h"
#if 1
extern  FIL file1;
extern __align(4) uint8_t rcvbuf[PACKET_SIZE];
extern __align(4) uint8_t sendbuf[PACKET_SIZE];
__align(4) uint8_t aprom_buf[512];

//extern BOOL RcvData(void);
//extern BOOL SendData(void);
unsigned int g_packno = 1;
unsigned short gcksum;


void WordsCpy(void *dest, void *src, int32_t size)
{
    uint8_t *pu8Src, *pu8Dest;
    int32_t i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;
    
    for(i=0;i<size;i++)
        pu8Dest[i] = pu8Src[i]; 
}

uint16_t Checksum (unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c=0, i=0; i < len; i++) {
        c += buf[i];
    }
    return (c);
}

BOOL SendData(void)
{
	BOOL Result;

	gcksum = Checksum(sendbuf, PACKET_SIZE);
#if SPI_BUS
	Result = SPI_MasterSendDataT1();
#endif
	
#if UART_BUS	
  Result = UART_MasterSendDataT1();
#endif
	
	#if UART1WIRE
	UART1WIRTE_TX_64(sendbuf);
	Result=TRUE;
	#endif
	return Result;
}

BOOL RcvData(unsigned int count)
{
	BOOL Result;
	unsigned short lcksum,i;
	uint8_t *pBuf;
#if SPI_BUS
	CLK_SysTickDelay(50000);//50ms	
  Result = SPI_MasterRcvDataT1();
	#endif
#if UART_BUS	
	Result = UART_MasterRcvDataT2(count);
	#endif
	#if UART1WIRE
	Result = UART1WIRE_RX_64_no_timeout(rcvbuf);
	#endif
	if (Result == FALSE)
		return Result;

	pBuf = rcvbuf;
	WordsCpy(&lcksum, pBuf, 2);
	pBuf += 4;

	if (inpw(pBuf) != g_packno)
	{
		dbg_printf("g_packno=%d rcv %d\n", g_packno, inpw(pBuf));
		Result = FALSE;
	}
	else
	{
		if (lcksum != gcksum)
		{
			dbg_printf("gcksum=%x lcksum=%x\n", gcksum, lcksum);
			Result = FALSE;
		}
		g_packno++;

	}
	return Result;
}

BOOL CmdSyncPackno(int flag)
{
	BOOL Result;
	unsigned long cmdData;
	
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_SYNC_PACKNO;//CMD_UPDATE_APROM
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	WordsCpy(sendbuf+8, &g_packno, 4);
	g_packno++;
	
	Result = SendData();
	if(Result == FALSE)
		return Result;

	Result = RcvData(1);
	
	return Result;
}

BOOL CmdFWVersion(int flag, unsigned int *fwver)
{
	BOOL Result;
	unsigned long cmdData;
	unsigned int lfwver;
	
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_FWVER;
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	g_packno++;
	
	Result = SendData();
	if(Result == FALSE)
		return Result;

	Result = RcvData(1);
	if(Result)
	{
		WordsCpy(&lfwver, rcvbuf+8, 4);
		*fwver = lfwver;
	}
	
	return Result;
}


BOOL CmdGetDeviceID(int flag, unsigned int *devid)
{
	BOOL Result;
	unsigned long cmdData;
	unsigned int ldevid;
	
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_DEVICEID;
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	g_packno++;
	
	Result = SendData();
	if(Result == FALSE)
		return Result;

	Result = RcvData(1);
	if(Result)
	{
		WordsCpy(&ldevid, rcvbuf+8, 4);
		*devid = ldevid;
	}
	
	return Result;
}

BOOL CmdGetConfig(int flag, unsigned int *config)
{
	BOOL Result;
	unsigned long cmdData;
	unsigned int lconfig[2];
	
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_READ_CONFIG;
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	g_packno++;
	
	Result = SendData();
	if(Result == FALSE)
		return Result;

	Result = RcvData(1);
	if(Result)
	{
		WordsCpy(&lconfig[0], rcvbuf+8, 4);
		WordsCpy(&lconfig[1], rcvbuf+12, 4);
		config[0] = lconfig[0];
		config[1] = lconfig[1];
	}
	
	return Result;
}

//uint32_t def_config[2] = {0xFFFFFF7F, 0x0001F000};
//CmdUpdateConfig(FALSE, def_config)
BOOL CmdUpdateConfig(int flag, uint32_t *conf)
{
	BOOL Result;
	unsigned long cmdData;
	
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_UPDATE_CONFIG;
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	WordsCpy(sendbuf+8, conf, 8);
	g_packno++;
	
	Result = SendData();
	if(Result == FALSE)
		return Result;

	Result = RcvData(2);
	
	return Result;
}

//for the commands
//CMD_RUN_APROM
//CMD_RUN_LDROM
//CMD_RESET
//CMD_ERASE_ALL
//CMD_GET_FLASHMODE
//CMD_WRITE_CHECKSUM
BOOL CmdRunCmd(uint32_t cmd, uint32_t *data)
{
	BOOL Result;
	uint32_t cmdData,i;
	
	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = cmd;
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	if(cmd == CMD_WRITE_CHECKSUM)
	{
		WordsCpy(sendbuf+8, &data[0], 4);
		WordsCpy(sendbuf+12, &data[1], 4);
	}
	g_packno++;
	
	Result = SendData();
	if(Result == FALSE)
		return Result;
	
	if((cmd == CMD_ERASE_ALL) || (cmd == CMD_GET_FLASHMODE) 
			|| (cmd == CMD_WRITE_CHECKSUM))
	{
		if(cmd == CMD_WRITE_CHECKSUM)
			//SysTimerDelay(400000);//0.2s
			for (i = 0; i<200; i++)
				CLK_SysTickDelay(1000);
		Result = RcvData(2);
		if(Result)
		{
			if(cmd == CMD_GET_FLASHMODE)
			{
				WordsCpy(&cmdData, rcvbuf+8, 4);
				*data = cmdData;
			}
		}
		
	}
	else if ((cmd == CMD_RUN_APROM) || (cmd == CMD_RUN_LDROM)
		|| (cmd == CMD_RESET))
		//SysTimerDelay(1000000);//0.5s
	{
		for (i = 0; i < 500; i++)
			CLK_SysTickDelay(1000);
	}
	return Result;
}
volatile unsigned int file_totallen;
volatile unsigned int file_checksum;

BOOL CmdUpdateAprom(int flag)
{
	BOOL Result;
	unsigned int devid, config[2], i, mode,j;
	unsigned long cmdData, startaddr;
	unsigned short get_cksum;
	unsigned char Buff[256];
	unsigned int s1;
	//for uart auto detect;
#if UART_AUTO_DETECT
	for (i = 0; i < 64;i++)
		rcvbuf[i]=0;
	rcvbuf[0] = 0xae;
	rcvbuf[4]=0x01;
	auto_detect_command();
	ILI9341_LCD_PutString_line(7,0,"Auto Detect Connect",Red,Yellow);
	#endif
	
	
	g_packno = 1;
	
	Result = CmdSyncPackno(flag);
	if(Result == FALSE)
	{
		dbg_printf("send Sync Packno cmd fail\n");
		goto out;
	}
	Result = CmdRunCmd(CMD_GET_FLASHMODE, &mode);
	if(mode != LDROM_MODE)
	{
		dbg_printf("change to LDROM ");
		CmdRunCmd(CMD_RUN_LDROM, NULL);
		Result = CmdSyncPackno(flag);
		if(Result == FALSE)
		{
			dbg_printf("ldrom Sync Packno cmd fail\n");
			goto out;
		}
	
		Result = CmdRunCmd(CMD_GET_FLASHMODE, &mode);
		if(mode != LDROM_MODE)
		{
			dbg_printf("fail\n");
			goto out;
		}
		else
			dbg_printf("ok\n");
	}
	
	CmdGetDeviceID(flag, &devid);
	dbg_printf("DeviceID: 0x%x\n", devid);
	memset(Buff, '\0', sizeof(Buff)); 
  sprintf((char *)Buff, "DeviceID: 0x%x", devid);
  ILI9341_LCD_PutString_line(8,0,Buff,Red,Yellow);
	
	CmdGetConfig(flag, config);
	dbg_printf("config0: 0x%x\n", config[0]);
	dbg_printf("config1: 0x%x\n", config[1]);
	memset(Buff, '\0', sizeof(Buff)); 
  sprintf((char *)Buff,"config0: 0x%x\n", config[0]);
  ILI9341_LCD_PutString_line(9,0,Buff,Red,Yellow);
		memset(Buff, '\0', sizeof(Buff)); 
  sprintf((char *)Buff,"config1: 0x%x\n", config[1]);
  ILI9341_LCD_PutString_line(10,0,Buff,Red,Yellow);
	
	
	
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_UPDATE_APROM;//CMD_UPDATE_APROM
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	g_packno++;
	//start address
	startaddr = 0;
	WordsCpy(sendbuf+8, &startaddr, 4);
	WordsCpy(sendbuf+12, &file_totallen, 4);
	f_open(&file1,BIN_FILE,FA_OPEN_EXISTING | FA_READ);
	//f_read(&file1,&sendbuf[16],48,&s1);
	f_read(&file1,aprom_buf,48,&s1);
	WordsCpy(sendbuf+16, aprom_buf, 48);
	//send CMD_UPDATE_APROM
	Result = SendData();
	if(Result == FALSE)
		goto out;
	ILI9341_LCD_PutString_line(11,0,"Erase",Red,Yellow);
	
	//for erase time delay using, other bus need it.

	
	#if SPI_BUS
	for (i = 0; i<12000; i++)
		CLK_SysTickDelay(1000);
	#endif
	dbg_printf("rcv data2\n");
	
	Result = RcvData(20);
	if(Result == FALSE)
		goto out;
	
	for ( i = 48; i < file_totallen; i = i + 56)
	{
	if(((i-48)%448)==0)
	f_read(&file1,aprom_buf,448,&s1);	
		
		  dbg_printf("i=%d \n\r",i);
			memset(Buff, '\0', sizeof(Buff)); 
	sprintf((char *)Buff,"Programm: %.0f %%",(float)(((float)i/(float)file_totallen)*100.0));
		  ILI9341_LCD_PutString_line(11,0,Buff,Red,Yellow);
		//clear buffer
		for (j = 0; j < 64; j++)
		{
			sendbuf[j] = 0;
		}
		//WordsCpy(sendbuf+0, &cmdData, 4);
		WordsCpy(sendbuf+4, &g_packno, 4);
		g_packno++;
		if ((file_totallen - i) > 56)
		{			
			//f_read(&file1,&sendbuf[8],56,&s1);
			WordsCpy(sendbuf+8, &aprom_buf[((i-48)%448)], 56);
			//read check  package
			Result = SendData();
			if(Result == FALSE)
				goto out;
      Result = RcvData(2);
			if(Result == FALSE)
				goto out;			
		}
		else
		{
		  //f_read(&file1,&sendbuf[8],file_totallen - i,&s1);
			WordsCpy(sendbuf+8, &aprom_buf[((i-48)%448)], file_totallen - i);
            //read target chip checksum
      Result = SendData();
			if(Result == FALSE)
				goto out;
      Result = RcvData(2);
			if(Result == FALSE)			
			  goto out;	
			
			WordsCpy(&get_cksum, rcvbuf+8, 2);
	   if((file_checksum&0xffff) != get_cksum)	
		 {			 
			 Result = FALSE;
			  goto out;
		 }
		}
	}
	
	
	#if 0
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_UPDATE_APROM;//CMD_UPDATE_APROM
	WordsCpy(sendbuf+0, &cmdData, 4);
	WordsCpy(sendbuf+4, &g_packno, 4);
	g_packno++;
	//start address
	startaddr = 0;
	WordsCpy(sendbuf+8, &startaddr, 4);
	WordsCpy(sendbuf+12, &file_totallen, 4);
	f_open(&file1,BIN_FILE,FA_OPEN_EXISTING | FA_READ);
	f_read(&file1,&sendbuf[16],48,&s1);
	//send CMD_UPDATE_APROM
	Result = SendData();
	if(Result == FALSE)
		goto out;
	ILI9341_LCD_PutString_line(11,0,"Erase",Red,Yellow);
	//for erase time using, other bus need it.
	#if 0
	for (i = 0; i<12000; i++)
		CLK_SysTickDelay(1000);
	#endif
	dbg_printf("rcv data2\n");
	
	Result = RcvData();
	if(Result == FALSE)
		goto out;
	
	for ( i = 48; i < file_totallen; i = i + 56)
	{
		dbg_printf("i=%d \n\r",i);
			memset(Buff, '\0', sizeof(Buff)); 
      sprintf((char *)Buff,"Programm %d",(i/file_totallen)*100);
		  ILI9341_LCD_PutString_line(11,0,Buff,Red,Yellow);
		//clear buffer
		for (j = 0; j < 64; j++)
		{
			sendbuf[j] = 0;
		}
		//WordsCpy(sendbuf+0, &cmdData, 4);
		WordsCpy(sendbuf+4, &g_packno, 4);
		g_packno++;
		if ((file_totallen - i) > 56)
		{			
			f_read(&file1,&sendbuf[8],56,&s1);
			//read check  package
			Result = SendData();
			if(Result == FALSE)
				goto out;
      Result = RcvData();
			if(Result == FALSE)
				goto out;			
		}
		else
		{
		  f_read(&file1,&sendbuf[8],file_totallen - i,&s1);
            //read target chip checksum
      Result = SendData();
			if(Result == FALSE)
				goto out;
      Result = RcvData();
			if(Result == FALSE)			
			  goto out;	
			
			WordsCpy(&get_cksum, rcvbuf+8, 2);
	   if((file_checksum&0xffff) != get_cksum)	
		 {			 
			 Result = FALSE;
			  goto out;
		 }
		}
	}
	#endif
	

out:
	f_close(&file1);
if(Result==TRUE)	
	ILI9341_LCD_PutString_line(11,0,"Programmer Pass!!",Red,Yellow);
else
	ILI9341_LCD_PutString_line(11,0,"Programmer FALSE!!",Red,Blue);
	return Result;
}

#endif

