#include "StdAfx.h"
#include "ISP_COMMAND.h"
#include "HID.h"
#include <time.h>
#include <io.h> 
#include <fcntl.h>
#define connect_sleep_time 50
#define dbg_printf printf
//#define dbg_printf(...) 
//#define package_printf printf
#define package_printf(...) 
#define USB_VID 0x0416
#define USB_PID 0x5020
//#define Time_Out_Value 1000
//#define Time_Out_Value 5000 //uart
#define Time_Out_Value 5000
#define Package_Size 64
CHidCmd pUSB;
DWORD Length;
unsigned char buffer[Package_Size]={0};
unsigned int PacketNumber;
unsigned int Address,Size;
struct CPartNumItem
{
	char PartNumber[255];
	unsigned int APROM;
	unsigned int LDROM;
	unsigned int DATAFLASH;
	unsigned int PID;		
};

static CPartNumItem g_PartNumItems[] ={

	"NUC101YD2BN",	64,	4, 4,    0x10010143,
	"NUC101YD1BN",	64,	4, 4,    0x10010144,
	"NUC101YD0BN",	64,	4, 4,    0x10010145,
	"NUC101YC2BN",	32,	4, 4,    0x10010146,
	"NUC101YC1BN",	32,	4, 4,    0x10010147,
	"NUC101YC0BN",	32,	4, 4,    0x10010148,
	"NUC101YB2BN",	16,	4, 4,    0x10010149,
	"NUC101YB1BN",	16,	4, 4,    0x10010150,
	"NUC101YB0BN",	16,	4, 4,    0x10010151,
	"NUC101YA2BN",	8,	4, 4,    0x10010152,
	"NUC101YA1BN",	8,	4, 4,    0x10010153,
	"NUC101YA0BN",	8,	4, 4,    0x10010154,
	"NUC101LD2BN",	64,	4, 4,    0x10010104,
	"NUC101LD1BN",	64,	4, 4,    0x10010105,
	"NUC101LD0BN",	64,	4, 4,    0x10010127,
	"NUC101LC2BN",	32,	4, 4,    0x10010107,
	"NUC101LC1BN",	32,	4, 4,    0x10010108,
	"NUC101LC0BN",	32,	4, 4,    0x10010128,
	"NUC101LB2BN",	16,	4, 4,    0x10010129,
	"NUC101LB1BN",	16,	4, 4,    0x10010130,
	"NUC101LB0BN",	16,	4, 4,    0x10010131,
	"NUC101LA2BN",	8,	4, 4,    0x10010132,
	"NUC101LA1BN",	8,	4, 4,    0x10010133,
	"NUC101LA0BN",	8,	4, 4,    0x10010134,
	"NUC101RD2BN",	64,	4, 4,    0x10010113,
	"NUC101RD1BN",	64,	4, 4,    0x10010114,
	"NUC101RD0BN",	64,	4, 4,    0x10010135,
	"NUC101RC2BN",	32,	4, 4,    0x10010116,
	"NUC101RC1BN",	32,	4, 4,    0x10010117,
	"NUC101RC0BN",	32,	4, 4,    0x10010136,
	"NUC101RB2BN",	16,	4, 4,    0x10010137,
	"NUC101RB1BN",	16,	4, 4,    0x10010138,
	"NUC101RB0BN",	16,	4, 4,    0x10010139,
	"NUC101RA2BN",	8,	4, 4,    0x10010140,
	"NUC101RA1BN",	8,	4, 4,    0x10010141,	
	"NUC101RA0BN",	8,	4, 4,    0x10010142,
	"NUC100LD2BN",	64,	4, 4,    0x10010004,
	"NUC100LD1BN",	64,	4, 4,    0x10010005,
	"NUC100LD0BN",	64,	4, 4,    0x10010027,
	"NUC100LC2BN",	32,	4, 4,    0x10010007,
	"NUC100LC1BN",	32,	4, 4,    0x10010008,
	"NUC100LC0BN",	32,	4, 4,    0x10010028,
	"NUC100LB2BN",	16,	4, 4,    0x10010029,
	"NUC100LB1BN",	16,	4, 4,    0x10010030,
	"NUC100LB0BN",	16,	4, 4,    0x10010031,
	"NUC100LA2BN",	8,	4, 4,    0x10010032,
	"NUC100LA1BN",	8,	4, 4,    0x10010033,
	"NUC100LA0BN",	8,	4, 4,    0x10010034,
	"NUC100RD2BN",	64,	4, 4,    0x10010013,	
	"NUC100RD1BN",	64,	4, 4,    0x10010014,
	"NUC100RD0BN",	64,	4, 4,    0x10010035,	
	"NUC100RC2BN",	32,	4, 4,    0x10010016,	
	"NUC100RC1BN",	32,	4, 4,    0x10010017,	
	"NUC100RC0BN",	32,	4, 4,    0x10010036,	
	"NUC100RB2BN",	16,	4, 4,    0x10010037,	
	"NUC100RB1BN",	16,	4, 4,    0x10010038,	
	"NUC100RB0BN",	16,	4, 4,    0x10010039,	
	"NUC100RA2BN",	8,	4, 4,    0x10010040,	
	"NUC100RA1BN",	8,	4, 4,    0x10010041,	
	"NUC100RA0BN",	8,	4, 4,    0x10010042,	
	"NUC120LD2BN",	64,	4, 4,    0x10012004,	
	"NUC120LD1BN",	64,	4, 4,    0x10012005,	
	"NUC120LD0BN",	64,	4, 4,    0x10012027,	
	"NUC120LC2BN",	32,	4, 4,    0x10012007,	
	"NUC120LC1BN",	32,	4, 4,    0x10012008,	
	"NUC120LC0BN",	32,	4, 4,    0x10012028,	
	"NUC120LB2BN",	16,	4, 4,    0x10012029,
	"NUC120LB1BN",	16,	4, 4,    0x10012030,	
	"NUC120LB0BN",	16,	4, 4,    0x10012031,
	"NUC120LA2BN",	8,	4, 4,    0x10012032,	
	"NUC120LA1BN",	8,	4, 4,    0x10012033,	
	"NUC120LA0BN",	8,	4, 4,    0x10012034,
	"NUC120RD2BN",	64,	4, 4,    0x10012013,	
	"NUC120RD1BN",	64,	4, 4,    0x10012014,	
	"NUC120RD0BN",	64,	4, 4,    0x10012035,	
	"NUC120RC2BN",	32,	4, 4,    0x10012016,	
	"NUC120RC1BN",	32,	4, 4,    0x10012017,	
	"NUC120RC0BN",	32,	4, 4,    0x10012036,	
	"NUC120RB2BN",	16,	4, 4,    0x10012037,	
	"NUC120RB1BN",	16,	4, 4,    0x10012038,	
	"NUC120RB0BN",	16,	4, 4,    0x10012039,	
	"NUC120RA2BN",	8,	4, 4,    0x10012040,	
	"NUC120RA1BN",	8,	4, 4,    0x10012041,	
	"NUC120RA0BN",	8,	4, 4,    0x10012042,	
	"NUC130LD2BN",	64,	4, 4,    0x10013004,	
	"NUC130LD1BN",	64,	4, 4,    0x10013005,	
	"NUC130LD0BN",	64,	4, 4,    0x10013027,	
	"NUC130LC2BN",	32,	4, 4,    0x10013007,	
	"NUC130LC1BN",	32,	4, 4,    0x10013008,	
	"NUC130LC0BN",	32,	4, 4,    0x10013028,	
	"NUC130LB2BN",	16,	4, 4,    0x10013029,	
	"NUC130LB1BN",	16,	4, 4,    0x10013030,	
	"NUC130LB0BN",	16,	4, 4,    0x10013031,	
	"NUC130LA2BN",	8,	4, 4,    0x10013032,	
	"NUC130LA1BN",	8,	4, 4,    0x10013033,	
	"NUC130LA0BN",	8,	4, 4,    0x10013034,	
	"NUC130RD2BN",	64,	4, 4,    0x10013013,	
	"NUC130RD1BN",	64,	4, 4,    0x10013014,	
	"NUC130RD0BN",	64,	4, 4,    0x10013035,	
	"NUC130RC2BN",	32,	4, 4,    0x10013016,	
	"NUC130RC1BN",	32,	4, 4,    0x10013017,	
	"NUC130RC0BN",	32,	4, 4,    0x10013036,	
	"NUC130RB2BN",	16,	4, 4,    0x10013037,	
	"NUC130RB1BN",	16,	4, 4,    0x10013038,	
	"NUC130RB0BN",	16,	4, 4,    0x10013039,	
	"NUC130RA2BN",	8,	4, 4,    0x10013040,	
	"NUC130RA1BN",	8,	4, 4,    0x10013041,	
	"NUC130RA0BN",	8,	4, 4,    0x10013042,	
	"NUC140LD2BN",	64,	4, 4,    0x10014004,	
	"NUC140LD1BN",	64,	4, 4,    0x10014005,	
	"NUC140LD0BN",	64,	4, 4,    0x10014027,	
	"NUC140LC2BN",	32,	4, 4,    0x10014007,	
	"NUC140LC1BN",	32,	4, 4,    0x10014008,	
	"NUC140LC0BN",	32,	4, 4,    0x10014028,	
	"NUC140LB2BN",	16,	4, 4,    0x10014029,	
	"NUC140LB1BN",	16,	4, 4,    0x10014030,	
	"NUC140LB0BN",	16,	4, 4,    0x10014031,	
	"NUC140LA2BN",	8,	4, 4,    0x10014032,	
	"NUC140LA1BN",	8,	4, 4,    0x10014033,	
	"NUC140LA0BN",	8,	4, 4,    0x10014034,	
	"NUC140RD2BN",	64,	4, 4,    0x10014013,	
	"NUC140RD1BN",	64,	4, 4,    0x10014014,	
	"NUC140RD0BN",	64,	4, 4,    0x10014035,	
	"NUC140RC2BN",	32,	4, 4,    0x10014016,	
	"NUC140RC1BN",	32,	4, 4,    0x10014017,	
	"NUC140RC0BN",	32,	4, 4,    0x10014036,	
	"NUC140RB2BN",	16,	4, 4,    0x10014037,	
	"NUC140RB1BN",	16,	4, 4,    0x10014038,	
	"NUC140RB0BN",	16,	4, 4,    0x10014039,	
	"NUC140RA2BN",	8,	4, 4,    0x10014040,	
	"NUC140RA1BN",	8,	4, 4,    0x10014041,	
	"NUC140RA0BN",	8,	4, 4,    0x10014042,


	"NUC100LC1AN",	32,	4, 4,    0x00010008,	
	"NUC100LD1AN",	64,	4, 4,    0x00010005,	
	"NUC100LD2AN",	64,	4, 4,    0x00010004,	
	"NUC100LD3AN",	64,	4, 4,    0x00010003,	
	"NUC100LE3AN",	128,4, 0,    0x00010000,	
	"NUC100RC1AN",	32,	4, 4,    0x00010017,	
	"NUC100RD1AN",	64,	4, 4,    0x00010014,	
	"NUC100RD2AN",	64,	4, 4,    0x00010013,	
	"NUC100RD3AN",	64,	4, 4,    0x00010012,	
	"NUC100RE3AN",	128,4, 0,    0x00010009,
	"NUC100VD2AN",	64,	4, 4,    0x00010022,
	"NUC100VD3AN",	64,	4, 4,    0x00010021,
	"NUC100VE3AN",	128,4, 0,    0x00010018,
	"NUC101LC1AN",	32,	4, 4,    0x00010108,
	"NUC101LE3AN",	128,4, 0,    0x00010100,
	"NUC120LC1AN",	32,	4, 4,    0x00012008,
	"NUC120LD1AN",	64,	4, 4,    0x00012005,
	"NUC120LD2AN",	64,	4, 4,    0x00012004,
	"NUC120LD3AN",	64,	4, 4,    0x00012003,
	"NUC120LE3AN",	128,4, 0,    0x00012000,
	"NUC120RC1AN",	32,	4, 4,    0x00012017,
	"NUC120RD1AN",	64,	4, 4,    0x00012014,
	"NUC120RD2AN",	64,	4, 4,    0x00012013,
	"NUC120RD3AN",	64,	4, 4,    0x00012012,
	"NUC120RE3AN",	128,4, 0,    0x00012009,
	"NUC120VD2AN",	64,	4, 4,    0x00012022,
	"NUC120VD3AN",	64,	4, 4,    0x00012021,
	"NUC120VE3AN",	128,4, 0,    0x00012018,

	"NUC122ZD2AN",	64,	4, 4,    0x00012231,
	"NUC122ZC1AN",	32,	4, 4,    0x00012235,
	"NUC122LD2AN",	64,	4, 4,    0x00012204,
	"NUC122LC1AN",	32,	4, 4,    0x00012208,
	"NUC122RD2AN",	64, 4, 4,    0x00012213,
	"NUC122RC1AN",	32,	4, 4,    0x00012217,

	"NUC130LD2AN",	64,	4, 4,    0x00013004,
	"NUC130LD3AN",	64,	4, 4,    0x00013003,
	"NCU130LE3AN",	128,4, 0,    0x00013000,
	"NUC130RD2AN",	64,	4, 4,    0x00013013,
	"NUC130RD3AN",	64,	4, 4,    0x00013012,
	"NUC130RE3AN",	128,4, 0,    0x00013009,
	"NUC130VD2AN",	64,	4, 4,    0x00013022,
	"NUC130VD3AN",	64,	4, 4,    0x00013021,	
	"NUC130VE3AN",	128,4, 0,    0x00013018,
	"NUC140LD2AN",	64,	4, 4,    0x00014004,	
	"NUC140LD3AN",	64,	4, 4,    0x00014003,	
	"NUC140LE3AN",	128,4, 0,    0x00014000,	
	"NUC140RD2AN",	64,	4, 4,    0x00014013,	
	"NUC140RD3AN",	64,	4, 4,    0x00014012,	
	"NUC140RE3AN",	128,4, 0,    0x00014009,	
	"NUC140VD2AN",	64,	4, 4,    0x00014022,	
	"NUC140VD3AN",	64,	4, 4,    0x00014021,	
	"NUC140VE3AN",	128,4, 0,    0x00014018,

	"NUC130LC1CN",	32,	4, 4,    0x20013008,
	"NUC130LD2CN",	64,	4, 4,    0x20013004,
	"NUC130LE3CN",	128,4, 0,    0x20013000,
	"NUC130RC1CN",	32,	4, 4,    0x20013017,
	"NUC130RD2CN",	64,	4, 4,    0x20013013,
	"NUC130RE3CN",	128,4, 0,    0x20013009,
	"NUC130VE3CN",	128,4, 0,    0x20013018,

	"NUC140LC1CN",	32,	4, 4,    0x20014008,
	"NUC140LD2CN",	64,	4, 4,    0x20014004,
	"NUC140LE3CN",	128,4, 0,    0x20014000,
	"NUC140RC1CN",	32,	4, 4,    0x20014017,
	"NUC140RD2CN",	64,	4, 4,    0x20014013,
	"NUC140RE3CN",	128,4, 0,    0x20014009,
	"NUC140VE3CN",	128,4, 0,    0x20014018,

	"NUC12SRE3AN",	128,4, 0,    0x00012009,

	"M052LAN", 8, 4,4, 0x00005200, 
	"M052ZAN", 8, 4,4, 0x00005203, 
	"M054LAN", 16,4,4, 0x00005400, 
	"M054ZAN",  16,4,4,0x00005403, 
	"M058LAN",  32,4,4,0x00005800, 
	"M058ZAN",  32,4,4, 0x00005803, 
	"M0516LAN",  64,4,4,0x00005A00, 
	"M0516ZAN",  64,4,4,0x00005A03, 


	"M052LBN", 8, 4,4, 0x10005200, 
	"M052ZBN", 8, 4,4, 0x10005203, 
	"M052TBN", 8, 4,4, 0x10005204, 
	"M052XBN", 8, 4,4, 0x10005205, 
	"M054LBN", 16,4,4, 0x10005400, 
	"M054ZBN",  16,4,4,0x10005403,
	"M054TBN", 16,4,4, 0x10005404, 
	"M054XBN",  16,4,4,0x10005405,  
	"M058LBN",  32,4,4,0x10005800, 
	"M058ZBN",  32,4,4,0x10005803,
	"M058TBN",  32,4,4,0x10005804, 
	"M058XBN",  32,4,4,0x10005805, 
	"M0516LBN",  64,4,4,0x10005A00, 
	"M0516ZBN",  64,4,4,0x10005A03,
	"M0516TBN",  64,4,4,0x10005A04, 
	"M0516XBN",  64,4,4,0x10005A05,

	"N572F064",  64,0,0,0xe572f064,
	"N572F064",  64,0,0,0xf064ff81, 
	"N572F064",  64,0,0,0xf064ff85, 
	"N572F065",  64,0,0,0xe572f065, 
	"N572F065",  64,0,0,0xf065ff85, 

	"MINI51LAN", 4, 2,0, 0x00205100,
	"MINI51QAN", 4, 2,0, 0x00205101, 
	"MINI51ZAN", 4, 2,0, 0x00205103, 
	"MINI51TAN", 4, 2,0, 0x00205104,
	"MINI52LAN", 8, 2,0, 0x00205200,
	"MINI52QAN", 8, 2,0, 0x00205201,
	"MINI52ZAN", 8, 2,0, 0x00205203, 
	"MINI52TAN", 8, 2,0, 0x00205204, 
	"MINI54LAN", 16,2,0, 0x00205400, 
	"MINI54QAN", 16,2,0, 0x00205401, 
	"MINI54ZAN", 16,2,0, 0x00205403,
	"MINI54TAN", 16,2,0, 0x00205404,
	"NUC123SC2AN1",32,4,4, 0x00012305,
    "NUC123SD4AN0",64,4,4, 0x00012315,
    "NUC123LC2AN1",32,4,4, 0x00012325,
    "NUC123LD4AN0",64,4,4, 0x00012335,
    "NUC123ZC2AN1",32,4,4, 0x00012345,
    "NUC123ZD4AN0",64,4,4, 0x00012355,  
	"NUC240VE3AE",128,4,0, 0x10024018,
};


ISP_STATE ISP_COMMAND::OPEN_USBPORT(void)
{
	if(!pUSB.OpenDevice(USB_VID, USB_PID))
	{
		printf("connect false\n\r");
		return RES_CONNECT_FALSE;
	}
	else 
		USB_OPEN_FLAG=1;
		return RES_CONNECT;
}


#if 0
ISP_STATE ISP_COMMAND::OPEN_COMPORT(_TCHAR* temp)
{
		COMMTIMEOUTS CommTimeOuts ; //定义超时结构，并填写该结构
		DCB dcb;                    //定义数据控制块结构 
		
		memset(&CommTimeOuts, 0, sizeof(CommTimeOuts));
		memset(&dcb, 0, sizeof(dcb));
		CommTimeOuts.ReadIntervalTimeout = 20;//ms
		CommTimeOuts.ReadTotalTimeoutMultiplier=1;
		CommTimeOuts.ReadTotalTimeoutConstant=1;
//m_bPortReady = false;
//m_sComPort = "//";
		char comport_path[100] = "\\\\.\\";
		strcat(comport_path,temp);
m_hCom = CreateFile(comport_path, 
		GENERIC_READ | GENERIC_WRITE,
		0, // exclusive access
		NULL, // no security
		OPEN_EXISTING,
		0, // no overlapped I/O
		NULL); // null template

//PurgeComm( m_hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ); 
if(SetupComm(m_hCom, 128, 128)==true&&GetCommState(m_hCom, &m_dcb)==true)
{
m_dcb.BaudRate = 115200;
m_dcb.ByteSize = 8;
m_dcb.fBinary = TRUE ;
m_dcb.Parity = NOPARITY;
m_dcb.StopBits = ONESTOPBIT;
		m_dcb.fBinary = TRUE ;
			m_dcb.fParity = FALSE; 
				m_dcb.fDtrControl = 0;
SetCommState(m_hCom, &m_dcb);
SetCommMask(m_hCom, EV_RXCHAR|EV_TXEMPTY );
SetupComm(m_hCom, 1024, 128);
SetCommTimeouts( m_hCom, &CommTimeOuts );
/*
GetCommTimeouts (m_hCom, &m_CommTimeouts);
m_CommTimeouts.ReadIntervalTimeout = 20;
m_CommTimeouts.ReadTotalTimeoutConstant = 0;
m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
m_CommTimeouts.WriteTotalTimeoutConstant = 0;
m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
SetCommTimeouts (m_hCom, &m_CommTimeouts);
*/
PurgeComm( m_hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ); 
	COM_OPEN_FLAG=1;		
	return RES_CONNECT;
}
return RES_CONNECT;

}


ISP_STATE ISP_COMMAND::OPEN_COMPORT(void)
{
		COMMTIMEOUTS CommTimeOuts ; //定义超时结构，并填写该结构
		DCB dcb;                    //定义数据控制块结构 
		
		memset(&CommTimeOuts, 0, sizeof(CommTimeOuts));
		memset(&dcb, 0, sizeof(dcb));
		CommTimeOuts.ReadIntervalTimeout = 20;//ms
		CommTimeOuts.ReadTotalTimeoutMultiplier=1;
		CommTimeOuts.ReadTotalTimeoutConstant=1;
//m_bPortReady = false;
//m_sComPort = "//";
m_hCom = CreateFile("\\\\.\\com18", 
		GENERIC_READ | GENERIC_WRITE,
		0, // exclusive access
		NULL, // no security
		OPEN_EXISTING,
		0, // no overlapped I/O
		NULL); // null template

//PurgeComm( m_hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ); 
if(SetupComm(m_hCom, 128, 128)==true&&GetCommState(m_hCom, &m_dcb)==true)
{
m_dcb.BaudRate = 115200;
m_dcb.ByteSize = 8;
m_dcb.fBinary = TRUE ;
m_dcb.Parity = NOPARITY;
m_dcb.StopBits = ONESTOPBIT;
		m_dcb.fBinary = TRUE ;
			m_dcb.fParity = FALSE; 
				m_dcb.fDtrControl = 0;
SetCommState(m_hCom, &m_dcb);
SetCommMask(m_hCom, EV_RXCHAR|EV_TXEMPTY );
SetupComm(m_hCom, 1024, 128);
SetCommTimeouts( m_hCom, &CommTimeOuts );
/*
GetCommTimeouts (m_hCom, &m_CommTimeouts);
m_CommTimeouts.ReadIntervalTimeout = 20;
m_CommTimeouts.ReadTotalTimeoutConstant = 0;
m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
m_CommTimeouts.WriteTotalTimeoutConstant = 0;
m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
SetCommTimeouts (m_hCom, &m_CommTimeouts);
*/
PurgeComm( m_hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ); 
	COM_OPEN_FLAG=1;		
	return RES_CONNECT;
}
return RES_CONNECT;

}


ISP_STATE ISP_COMMAND::CLOSE_UART_PORT()
{
if(COM_OPEN_FLAG==1)
	CloseHandle(m_hCom);
	return RES_DISCONNECT;

} 
#endif

ISP_STATE ISP_COMMAND::CLOSE_USBPORT(void)
{
if(USB_OPEN_FLAG==1)
	pUSB.CloseDevice();
	return RES_DISCONNECT;
}




ISP_STATE ISP_COMMAND::CHECK_UART_LINK(void)
{
unsigned char cmd[Package_Size] = {0xae,0,0,0,
(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
while(1)
{
WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);
ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;
}
return RES_DETECT_MCU;
}

ISP_STATE ISP_COMMAND::SN_PACKAGE_UART(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd1[Package_Size] = {0xa4,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff),
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	WriteFile(m_hCom,cmd1,Package_Size,&iBytesWritten,NULL);		
	start_time = clock(); /* mircosecond */ 
	while(1)
	{
		ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;
		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return RES_TIME_OUT;
	}
	PacketNumber+=2;
	return RES_SN_OK;
}


unsigned int ISP_COMMAND::READFW_VERSION_UART(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd[Package_Size] = {0xa6,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);					
	start_time = clock(); /* mircosecond */  
	while(1)
	{
		ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;

		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return 0;

	}
	dbg_printf("fw version:0x%x\n\r",buffer[8]);
	dbg_printf("\n\r");
	PacketNumber+=2;
	return (buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000));
}



unsigned int ISP_COMMAND::READFW_VERSION(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd[Package_Size] = {0xa6,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);
	Sleep(connect_sleep_time);
	start_time = clock(); /* mircosecond */  
	while(1)
	{
		pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;

		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return 0;

	}
	dbg_printf("fw version:0x%x\n\r",buffer[8]);
	dbg_printf("\n\r");
	PacketNumber+=2;
	return (buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000));
}

#if 0
unsigned int ISP_COMMAND::SPI_READFW_VERSION(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd[Package_Size] = { 0xa6, 0, 0, 0xae,
		(PacketNumber & 0xff), ((PacketNumber >> 8) & 0xff), ((PacketNumber >> 16) & 0xff), ((PacketNumber >> 24) & 0xff) };
	pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);
	start_time = clock(); /* mircosecond */
	while (1)
	{
		pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
		dbg_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;

		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return 0;

	}
	dbg_printf("fw version:0x%x\n\r",buffer[8]);
	dbg_printf("\n\r");
	PacketNumber+=2;
	return (buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000));
}
#endif


void ISP_COMMAND::RUN_TO_APROM(void)
{
	unsigned char cmd[Package_Size] = {0xab,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);					
	PacketNumber+=2;
}


void ISP_COMMAND::RUN_TO_APROM_UART(void)
{
	unsigned char cmd[Package_Size] = {0xab,0,0,0,
	(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);						
	PacketNumber+=2;
}

ISP_STATE ISP_COMMAND::READ_PID(void)
{
	clock_t start_time, end_time;
	float total_time = 0; 
	unsigned char cmd[Package_Size] = {0xB1,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};	                    
	pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);					
	start_time = clock(); /* mircosecond */ 
	Sleep(connect_sleep_time);
	while(1)
	{
		pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;

		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return RES_TIME_OUT;
	}
	dbg_printf("pid: 0x%x\n\r",(buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000)));
	dbg_printf("\n\r");
	PacketNumber+=2;
	//return (buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000));
	unsigned int temp_PDID=buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000);
	CPartNumItem temp;

	int i=0,j=(sizeof(g_PartNumItems)/sizeof(g_PartNumItems[0]));
	while(1){
		temp=g_PartNumItems[i];
		if(temp_PDID==temp.PID)
		{
			printf("Part number: %s\n\r",g_PartNumItems[i].PartNumber);
			printf("APROM SIZE: %dKB\n\r",g_PartNumItems[i].APROM);
			printf("LDROM SIZE: %dKB\n\r",g_PartNumItems[i].LDROM);
			printf("DATAFLASH SIZE: %dKB\n\r",g_PartNumItems[i].DATAFLASH);
			APROM_SIZE=g_PartNumItems[i].APROM;
			LDROM_SIZE=g_PartNumItems[i].LDROM;
			DATAFLASH_SIZE=g_PartNumItems[i].DATAFLASH;		
			break;
		}
		i++;
		if(i == j)
			return RES_FALSE;		
	}

   return RES_PASS;
}

ISP_STATE ISP_COMMAND::READ_PID_UART(void)
{
	clock_t start_time, end_time;
	float total_time = 0; 
	unsigned char cmd[Package_Size] = {0xB1,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};	                    
	WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);						
	start_time = clock(); /* mircosecond */ 
	while(1)
	{
		ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;

		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return RES_TIME_OUT;
	}
	dbg_printf("pid: 0x%x\n\r",(buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000)));
	dbg_printf("\n\r");
	PacketNumber+=2;
	//return (buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000));
	unsigned int temp_PDID=buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000);
	CPartNumItem temp;

	int i=0,j=(sizeof(g_PartNumItems)/sizeof(g_PartNumItems[0]));
	while(1){
		temp=g_PartNumItems[i];
		if(temp_PDID==temp.PID)
		{
			printf("Part number: %s\n\r",g_PartNumItems[i].PartNumber);
			printf("APROM SIZE: %dKB\n\r",g_PartNumItems[i].APROM);
			printf("LDROM SIZE: %dKB\n\r",g_PartNumItems[i].LDROM);
			printf("DATAFLASH SIZE: %dKB\n\r",g_PartNumItems[i].DATAFLASH);
			APROM_SIZE=g_PartNumItems[i].APROM;
			LDROM_SIZE=g_PartNumItems[i].LDROM;
			DATAFLASH_SIZE=g_PartNumItems[i].DATAFLASH;		
			break;
		}
		i++;
		if(i == j)
			return RES_FALSE;		
	}

   return RES_PASS;
}

ISP_STATE ISP_COMMAND::USB_TO_UART_AUTO_DETECT(void)
{
	unsigned char cmd[Package_Size] = { 0xae, 0, 0, 0,
		(PacketNumber & 0xff), ((PacketNumber >> 8) & 0xff), ((PacketNumber >> 16) & 0xff), ((PacketNumber >> 24) & 0xff) };
	while (1)
	{
		pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);
		pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
		//WriteFile(m_hCom, cmd, Package_Size, &iBytesWritten, NULL);
		//ReadFile(m_hCom, &buffer, Package_Size, &iBytesRead, NULL);
		dbg_printf("package: 0x%x\n\r", buffer[4]);
		if ((buffer[4] | ((buffer[5] << 8) & 0xff00) | ((buffer[6] << 16) & 0xff0000) | ((buffer[7] << 24) & 0xff000000)) == (PacketNumber + 1))
			break;
	}
	PacketNumber += 2;
	return RES_DETECT_MCU;
}

ISP_STATE ISP_COMMAND::SN_PACKAGE(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd1[Package_Size] = {0xa4,0,0,0,
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff),
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);		
	start_time = clock(); /* mircosecond */ 
	while(1)
	{
		pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;
		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
			return RES_TIME_OUT;
	}
	PacketNumber+=2;
	return RES_SN_OK;
}

void ISP_COMMAND::READ_CONFIG(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd[Package_Size] = {0xa2,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);
	Sleep(connect_sleep_time);
	start_time = clock(); /* mircosecond */ 
	while(1)
	{
		pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;
		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
		{
			printf("Time out\n\r");
			break;
		}
	}
	printf("config0: 0x%x\n\r",(buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000)));
	printf("config1: 0x%x\n\r",(buffer[12]|((buffer[13]<<8)&0xff00)|((buffer[14]<<16)&0xff0000)|((buffer[15]<<24)&0xff000000)));
	printf("\n\r");
	PacketNumber+=2;
}

void ISP_COMMAND::READ_CONFIG_UART(void)
{
	clock_t start_time, end_time;
	float total_time = 0;

	unsigned char cmd[Package_Size] = {0xa2,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)};
	WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);
	start_time = clock(); /* mircosecond */ 
	while(1)
	{
		ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
		package_printf("package: 0x%x\n\r",buffer[4]);
		if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
			break;
		end_time = clock(); 
		/* CLOCKS_PER_SEC is defined at time.h */ 
		if((end_time - start_time)>Time_Out_Value)
		{
			printf("Time out\n\r");
			break;
		}
	}
	printf("config0: 0x%x\n\r",(buffer[8]|((buffer[9]<<8)&0xff00)|((buffer[10]<<16)&0xff0000)|((buffer[11]<<24)&0xff000000)));
	printf("config1: 0x%x\n\r",(buffer[12]|((buffer[13]<<8)&0xff00)|((buffer[14]<<16)&0xff0000)|((buffer[15]<<24)&0xff000000)));
	printf("\n\r");
	PacketNumber+=2;
}


ISP_STATE ISP_COMMAND::File_Open_APROM(_TCHAR* temp)
{
	FILE *fp;
	file_size=0;
	if((fp=fopen(temp,"rb"))==NULL)
	{
		dbg_printf("APROM FILE OPEN FALSE\n\r");
		return RES_FILE_NO_FOUND;
	}	
	if(fp!=NULL)
	{
		while(!feof(fp)) {	
			fread(&W_APROM_BUFFER[file_size], sizeof(char), 1, fp);                
			file_size++;	
		}
	}

	file_size=file_size-1;	
	fclose(fp);

	if (file_size > (APROM_SIZE * 1024))
	{
		dbg_printf("APROM FILE over size\n\r");
		return RES_FILE_SIZE_OVER;
	}
	return RES_FILE_LOAD;
}

ISP_STATE ISP_COMMAND::UPDATE_APROM(void)
{
	clock_t start_time, end_time;
	float total_time = 0;
	unsigned int count=0; 
	unsigned char cmd[Package_Size] = {0xa0,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff),
		(Address&0xff),((Address>>8)&0xff),((Address>>16)&0xff),((Address>>24)&0xff),
		(file_size&0xff),((file_size>>8)&0xff),((file_size>>16)&0xff),((file_size>>24)&0xff),
	};

	unsigned char cmd1[Package_Size] = {0,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)
	};
	printf("                                                        ");
	printf("\r program progrss 0%%                                  ");
	//first package

	while(1)
	{
		cmd[count+16]=W_APROM_BUFFER[count];
		count++;
		if(count+16==Package_Size||count>file_size) 
		{		
			dbg_printf("send first\n\r");

			pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);					
			Sleep(2000);
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				/*
				end_time = clock(); 	    
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
				*/
			}
			break;
		}
	}
	dbg_printf("\n\r");
	///////////////////////////////////
	int j=0;


	while(1)
	{
		cmd1[j+8]=W_APROM_BUFFER[count];
		count++;
		j++;

		if(count>file_size) 
		{		
			PacketNumber=PacketNumber+2;
			cmd1[4]=(PacketNumber&0xff);
			cmd1[5]=(PacketNumber>>8)&0xff;
			cmd1[6]=(PacketNumber>>16)&0xff;
			cmd1[7]=(PacketNumber>>24)&0xff;
			dbg_printf("send late\n\r");
			pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);
			Sleep(connect_sleep_time);
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				/*
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
				*/
			}
			PacketNumber=PacketNumber+2;
			break;
		}

		if(j+8==Package_Size)
		{
			//printf("\r                                                           ");
			printf("\r program progrss: %f %%",((float)count/(float)file_size)*100);
			PacketNumber=PacketNumber+2;
			cmd1[4]=(PacketNumber&0xff);
			cmd1[5]=(PacketNumber>>8)&0xff;
			cmd1[6]=(PacketNumber>>16)&0xff;
			cmd1[7]=(PacketNumber>>24)&0xff;
			//dbg_printf("send\n\r");
			pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);
			Sleep(connect_sleep_time);
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				/*
				end_time = clock();
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
				*/
			}
			j=0;
		}
		//dbg_printf("\n\r");
	}
	printf("\r                                ");
	printf("\r program progrss: 100%% \n\r");
	return RES_PASS;
}



ISP_STATE ISP_COMMAND::UPDATE_APROM_UART(void)
{
	clock_t start_time, end_time;
	float total_time = 0;
	unsigned int count=0; 
	unsigned char cmd[Package_Size] = {0xa0,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff),
		(Address&0xff),((Address>>8)&0xff),((Address>>16)&0xff),((Address>>24)&0xff),
		(file_size&0xff),((file_size>>8)&0xff),((file_size>>16)&0xff),((file_size>>24)&0xff),
	};

	unsigned char cmd1[Package_Size] = {0,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)
	};
	printf("                                                        ");
	printf("\r program progrss 0%%                                  ");
	//first package

	while(1)
	{
		cmd[count+16]=W_APROM_BUFFER[count];
		count++;
		if(count+16==Package_Size||count>file_size) 
		{		
			dbg_printf("send first\n\r");

			//pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);					
			WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);	
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				//pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;

				end_time = clock(); 
				/*
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
				*/

			}
			break;
		}
	}
	dbg_printf("\n\r");
	///////////////////////////////////
	int j=0;


	while(1)
	{
		cmd1[j+8]=W_APROM_BUFFER[count];
		count++;
		j++;

		if(count>file_size) 
		{		
			PacketNumber=PacketNumber+2;
			cmd1[4]=(PacketNumber&0xff);
			cmd1[5]=(PacketNumber>>8)&0xff;
			cmd1[6]=(PacketNumber>>16)&0xff;
			cmd1[7]=(PacketNumber>>24)&0xff;
			dbg_printf("send late\n\r");
			//pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);
			WriteFile(m_hCom,cmd,Package_Size,&iBytesWritten,NULL);	
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				//pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				/*
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
				*/
			}
			PacketNumber=PacketNumber+2;
			break;
		}

		if(j+8==Package_Size)
		{
			printf("\r                                                           ");
			printf("\r program progrss: %f %%",((float)count/(float)file_size)*100);
			PacketNumber=PacketNumber+2;
			cmd1[4]=(PacketNumber&0xff);
			cmd1[5]=(PacketNumber>>8)&0xff;
			cmd1[6]=(PacketNumber>>16)&0xff;
			cmd1[7]=(PacketNumber>>24)&0xff;
			dbg_printf("send\n\r");
			//pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);
			WriteFile(m_hCom,cmd1,Package_Size,&iBytesWritten,NULL);	
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				//pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				ReadFile(m_hCom, &buffer,Package_Size, &iBytesRead, NULL);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				end_time = clock();
				/*
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
				*/
			}
			j=0;
		}
		dbg_printf("\n\r");
	}
	printf("\r                                ");
	printf("\r program progrss: 100%% \n\r");
	return RES_PASS;
}



ISP_STATE ISP_COMMAND::UPDATE_DATAFLASH(void)
{
	clock_t start_time, end_time;
	float total_time = 0;
	unsigned int count=0; 
	unsigned char cmd[Package_Size] = {0xc3,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff),
		(Address&0xff),((Address>>8)&0xff),((Address>>16)&0xff),((Address>>24)&0xff),
		(file_size&0xff),((file_size>>8)&0xff),((file_size>>16)&0xff),((file_size>>24)&0xff),
	};

	unsigned char cmd1[Package_Size] = {0,0, 0, 0, 
		(PacketNumber&0xff),((PacketNumber>>8)&0xff),((PacketNumber>>16)&0xff),((PacketNumber>>24)&0xff)
	};
	printf("                                                        ");
	printf("\r program progrss 0%%                                  ");
	//first package

	while(1)
	{
		cmd[count+16]=W_APROM_BUFFER[count];
		count++;
		if(count+16==Package_Size||count>file_size) 
		{		
			dbg_printf("send first\n\r");

			pUSB.WriteFile((unsigned char *)&cmd, sizeof(cmd), &Length, 2000);					
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;

				end_time = clock(); 	    
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}

			}
			break;
		}
	}
	dbg_printf("\n\r");
	///////////////////////////////////
	int j=0;


	while(1)
	{
		cmd1[j+8]=W_APROM_BUFFER[count];
		count++;
		j++;

		if(count>file_size) 
		{		
			PacketNumber=PacketNumber+2;
			cmd1[4]=(PacketNumber&0xff);
			cmd1[5]=(PacketNumber>>8)&0xff;
			cmd1[6]=(PacketNumber>>16)&0xff;
			cmd1[7]=(PacketNumber>>24)&0xff;
			dbg_printf("send late\n\r");
			pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
			}
			PacketNumber=PacketNumber+2;
			break;
		}

		if(j+8==Package_Size)
		{
			printf("\r                                                           ");
			printf("\r program progrss: %f %%",((float)count/(float)file_size)*100);
			PacketNumber=PacketNumber+2;
			cmd1[4]=(PacketNumber&0xff);
			cmd1[5]=(PacketNumber>>8)&0xff;
			cmd1[6]=(PacketNumber>>16)&0xff;
			cmd1[7]=(PacketNumber>>24)&0xff;
			dbg_printf("send\n\r");
			pUSB.WriteFile((unsigned char *)&cmd1, sizeof(cmd1), &Length, 2000);
			start_time = clock(); /* mircosecond */  
			while(1)
			{
				pUSB.ReadFile(buffer, Package_Size, &Length, 2000);
				package_printf("package: 0x%x\n\r",buffer[4]);
				if((buffer[4]|((buffer[5]<<8)&0xff00)|((buffer[6]<<16)&0xff0000)|((buffer[7]<<24)&0xff000000))==(PacketNumber+1))
					break;
				end_time = clock();
				if((end_time - start_time)>Time_Out_Value)
				{
					printf("Time out\n\r");
					return RES_TIME_OUT;
				}
			}
			j=0;
		}
		dbg_printf("\n\r");
	}
	printf("\r                                ");
	printf("\r program progrss: 100%% \n\r");
	return RES_PASS;
}


ISP_COMMAND::ISP_COMMAND(void)
{
	COM_OPEN_FLAG=0;;
	USB_OPEN_FLAG=0;
	Address=0;
	PacketNumber=1;
	APROM_SIZE=0;
	LDROM_SIZE=0;
	DATAFLASH_SIZE=0;
}


ISP_COMMAND::~ISP_COMMAND(void)
{

}
