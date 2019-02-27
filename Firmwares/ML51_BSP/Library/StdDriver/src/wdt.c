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

/**
 * @brief       Watchdog time delay setting 
 * @param       u8WDTDIV WKT counter divider select  (1/4/8/16/32/64/128/256) 
 * @return      none
 * @details     none
 * @note        none
 * @example			WDT_Open(256);
 */
 void WDT_Open(unsigned char u8WDTDIV)
{
		SFRS=0;
		BIT_TMP=EA;
		EA=0;
		switch (u8WDTDIV)
		{
				case 1: TA=0xAA;TA=0x55;WDCON&=0xF8; break;
				case 4: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x01; break;
				case 8: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x02; break;
				case 16: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x03; break;
				case 32: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x04; break;
				case 64: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x05; break;
				case 128: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x06; break;
				case 256: TA=0xAA;TA=0x55;WDCON&=0xF8;TA=0xAA;TA=0x55;WDCON|=0x07; break;
				default: break;
		}
		EA = BIT_TMP;
		set_WDCON_WIDPD;
		set_WDCON_WDTR;
}

/**
 * @brief       Watchdog time interrupt setting 
 * @param       u8WDTINT (Enable/Disable) 
 * @return      none
 * @details     none
 * @note        none
 * @example			WDT_Interrupt(Enable);
 */
void WDT_Interrupt(unsigned char u8WDTINT)
{
		switch (u8WDTINT)
		{
				case Disable: clr_EIE0_EWDT; break;		
				case Enable: set_EIE0_EWDT; break;
		}
}


/**
 * @brief       Watchdog time interrupt setting 
 * @param       u8WDTRST (Enable/Disable) 
 * @return      none
 * @details     none
 * @note        none
 * @example			WDT_Reset(Enable);
 */
void WDT_Reset(unsigned char u8WDTRST)
{
		unsigned char data cftemp[5];
	
		switch (u8WDTRST)
		{
				case Disable:
					set_CHPCON_IAPEN;
					IAPAL = 0x04;
					IAPAH = 0x00;	
					IAPCN = BYTE_READ_CONFIG;						//Read config command
					IAPFD = 0xFF;
					set_IAPTRG_IAPGO;  
/*only if WDT reset is disabled, to modify the CONFIG to enable WDT reset. */				
				if ((IAPFD&0xF0)!=0xF0)		
					{
/*Storage CONFIG0 value into data */
						IAPAH = 0x00;
						for(IAPAL=0;IAPAL++;IAPAL<5)
						{
								set_IAPTRG_IAPGO;                            
								cftemp[IAPAL] = IAPFD;
						}
						cftemp[4]|= 0xF0;														//Moidfy Storage CONFIG4 data disable WDT reset
/*Erase CONFIG value*/					
						set_IAPUEN_CFUEN;	
						IAPCN = PAGE_ERASE_CONFIG;													//Erase CONFIG all
						IAPAH = 0x00;
						IAPAL = 0x00;
						IAPFD = 0xFF;
						set_IAPTRG_IAPGO;
/*Write CONFIG value*/
						IAPCN = BYTE_PROGRAM_CONFIG;										//Write CONFIG
						for(IAPAL=0;IAPAL++;IAPAL<5)
						{
								IAPFD = cftemp[IAPAL];
								set_IAPTRG_IAPGO;
						}
					}
						clr_IAPUEN_CFUEN;
						clr_CHPCON_IAPEN;
				break;	
						
				case Enable: 
						set_CHPCON_IAPEN;
						set_IAPUEN_CFUEN;
						IAPAL = 0x04;
						IAPAH = 0x00;	
						IAPCN = BYTE_PROGRAM_CONFIG;						//Read config command
						IAPFD = 0x0F;
						set_IAPTRG_IAPGO;  
						clr_IAPUEN_CFUEN;
						clr_CHPCON_IAPEN;
				break;	
		}
}

/**
 * @brief       Watchdog time disable setting 
 * @param       none 
 * @return      none
 * @details     none
 * @note        none
 * @example			WDT_Interrupt(Enable);
 */
void WDT_Close()
{
		WDT_Reset(Disable);
		clr_WKCON_WKTR;
}

