#ifndef ISP_USER_H
#define ISP_USER_H

#include "NUC505Series.h"
#include <stdio.h>
#include <stdint.h>

#define PAGE_SIZE	256
#define APROM_BASE      0x00004000UL
#define FLASH_SIZE      0x00200000UL


#define FW_VERSION					0x32

#define	USE_USB					0
#define SUPPORT_WRITECKSUM		0
#define USING_AUTODETECT 		1	//using autodetect for UART download
#define USING_RS485 			0	//default using GPF15 act as Rx/Tx switch for RS485 tranceiver

#define CMD_UPDATE_APROM			0x000000A0
#define CMD_UPDATE_CONFIG			0x000000A1
#define CMD_READ_CONFIG				0x000000A2
#define CMD_ERASE_ALL				0x000000A3
#define CMD_SYNC_PACKNO				0x000000A4
#define CMD_GET_FWVER				0x000000A6
#define CMD_SET_APPINFO     		0x000000A7
#define CMD_GET_APPINFO     		0x000000A8
#define CMD_RUN_APROM				0x000000AB
#define CMD_RUN_LDROM				0x000000AC
#define CMD_RESET					0x000000AD
#define CMD_CONNECT					0x000000AE
#define CMD_DISCONNECT				0x000000AF

#define CMD_GET_DEVICEID			0x000000B1

#define CMD_UPDATE_DATAFLASH 		0x000000C3
#define CMD_WRITE_CHECKSUM 	 		0x000000C9
#define CMD_GET_FLASHMODE 	 		0x000000CA

#define CMDBUFSIZE 	 		64



#define CMD_RESEND_PACKET       	0x000000FF

#define	V6M_AIRCR_VECTKEY_DATA		0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ		0x00000004UL

#define DISCONNECTED				0
#define CONNECTING					1
#define CONNECTED					2

#define BOOL		uint8_t

extern int ParseCmd(unsigned char *buffer, uint8_t len, uint8_t bUSB);
//extern void Reset_AfterREVMP(void);  //YT modify for MTP

extern __align(4) uint8_t g_u8RcvBuf[];
extern __align(4) uint8_t g_u8SendBuf[];

#define Config0         0x00300000UL
#define Config1         0x00300004UL


#endif	// #ifndef ISP_USER_H
