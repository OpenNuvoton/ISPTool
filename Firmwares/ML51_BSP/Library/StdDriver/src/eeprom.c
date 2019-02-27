/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//***********************************************************************************************************

#include "ML51.h"


volatile unsigned char xdata page_buffer[128];

/**
 * @brief       Read Dataflash 
 * @param       Dataflash start address
 * @return      Dataflash Value
 * @details     None
 */
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr)
{
		UINT8 rdata;
		rdata = *u16_addr>>8;
		return rdata;
}


/**
 * @brief       Write Dataflash as EEPROM, 
 * @param       u16EPAddr the 16bit EEPROM start address. Any of APROM address can be defined as start address (0x3800)
 * @param       u8EPData the 8bit value need storage in (0x3800)
 * @return      none
 * @details     Storage dataflash page data into XRAM 380H-400H, modify data in XRAM, Erase dataflash page, writer updated XRAM data into dataflash
 */
void Write_DATAFLASH_BYTE(unsigned int u16EPAddr,unsigned char u8EPData)
{
	unsigned char looptmp=0;
	unsigned int u16_addrl_r;
	unsigned int RAMtmp;
	
//Check page start address
	u16_addrl_r=(u16EPAddr/128)*128;
//Save APROM data to XRAM0
	for(looptmp=0;looptmp<0x80;looptmp++)
	{
		RAMtmp = Read_APROM_BYTE((unsigned int code *)(u16_addrl_r+looptmp));
		page_buffer[looptmp]=RAMtmp;
	}
// Modify customer data in XRAM
	page_buffer[u16EPAddr&0x7f] = u8EPData;
	
//Erase APROM DATAFLASH page
		IAPAL = u16_addrl_r&0xff;
		IAPAH = (u16_addrl_r>>8)&0xff;
		IAPFD = 0xFF;
	  set_CHPCON_IAPEN; 
		set_IAPUEN_APUEN;
    IAPCN = 0x22; 		
 		set_IAPTRG_IAPGO; 
		
//Save changed RAM data to APROM DATAFLASH
		set_CHPCON_IAPEN; 
		set_IAPUEN_APUEN;
	  IAPCN = 0x21;
		for(looptmp=0;looptmp<0x80;looptmp++)
		{
			IAPAL = (u16_addrl_r&0xff)+looptmp;
      IAPAH = (u16_addrl_r>>8)&0xff;
			IAPFD = page_buffer[looptmp];
			set_IAPTRG_IAPGO;			
		}
		clr_IAPUEN_APUEN;
		clr_CHPCON_IAPEN;
}	
	
