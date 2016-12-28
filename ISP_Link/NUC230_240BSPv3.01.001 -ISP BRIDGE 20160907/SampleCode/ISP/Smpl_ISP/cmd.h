#ifndef __CMD_H__
#define __CMD_H__
#define CMD_UPDATE_APROM	0x000000A0
#define CMD_UPDATE_CONFIG	0x000000A1
#define CMD_READ_CONFIG		0x000000A2
#define CMD_ERASE_ALL		0x000000A3
#define CMD_SYNC_PACKNO		0x000000A4
#define CMD_GET_FWVER		0x000000A6
#define CMD_APROM_SIZE		0x000000AA
#define CMD_RUN_APROM		0x000000AB
#define CMD_RUN_LDROM		0x000000AC
#define CMD_RESET			0x000000AD

#define CMD_GET_DEVICEID	0x000000B1

#define CMD_PROGRAM_WOERASE 	0x000000C2
#define CMD_PROGRAM_WERASE 	 	0x000000C3
#define CMD_READ_CHECKSUM 	 	0x000000C8
#define CMD_WRITE_CHECKSUM 	 	0x000000C9
#define CMD_GET_FLASHMODE 	 	0x000000CA

#define APROM_MODE	1
#define LDROM_MODE	2

#define BOOL  uint8_t
#define PAGE_SIZE                      0x00000200     /* Page size */

#define PACKET_SIZE	64//32
#define FILE_BUFFER	128

//extern uint8_t imageBegin, imageEnd;
//extern uint8_t rcvbuf[PACKET_SIZE];
//extern uint8_t sendbuf[PACKET_SIZE];
extern unsigned int g_packno;
extern unsigned short gcksum;

//BOOL SendData(void);
//BOOL RcvData(void);
//void SysTimerDelay(uint32_t us);//unit=0.5us
uint16_t Checksum (unsigned char *buf, int len);
void WordsCpy(void *dest, void *src, int32_t size);

BOOL CmdSyncPackno(int flag);
BOOL CmdGetCheckSum(int flag, int start, int len, unsigned short *cksum);
BOOL CmdGetDeviceID(int flag, unsigned int *devid);
BOOL CmdGetConfig(int flag, unsigned int *config);
BOOL CmdPutApromSize(int flag, unsigned int apsize);
BOOL CmdEraseAllChip(int flag);
BOOL CmdUpdateAprom(int flag);

#endif//__CMD_H__
