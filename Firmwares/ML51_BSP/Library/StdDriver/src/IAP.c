/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/************************************************************************************************************
  Website: http://www.nuvoton.com
  E-Mail : MicroC-8bit@nuvoton.com
************************************************************************************************************/

#include "ML51.h"
unsigned char xdata DIDBuffer[4];
unsigned char xdata UIDBuffer[9];
unsigned char xdata UCIDBuffer[12];
unsigned char xdata IAPDataBuf[128];
unsigned char xdata IAPCFBuf[5];

void IAP_ERROR(void)
{
	while (1)
	{
		P03 = 0;
		P03 = 1;
	}
}


/**
 * @brief       Trig IAP and check status flag  
 * @param       None
 * @return      none
 * @details     Trig IAPGO, check IAP Status flag if error set IAP disable all.
 */
void Trigger_IAP(void)
{   
    set_IAPTRG_IAPGO;														//trigger IAP
    if((CHPCON|CLR_BIT6)==1)             				// if fail flag is set, toggle error LED and IAP stop
		{
			clr_CHPCON_IAPFF;
			IAP_ERROR();
		}
}

/**
 * @brief       Erase LDROM  
 * @param       u16IAPStartAddress define LDROM area start address
 * @param 			u16IAPDataSize define LDROM need be erase bytes size
 * @return      none
 * @details     Page erase LDROM area base on data start address 
 * @example			Erase_LDROM(0x0000,2048);
 */
void Erase_LDROM(unsigned int u16IAPStartAddress,unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;

    set_CHPCON_IAPEN;										// Enable IAP function
	  set_IAPUEN_LDUEN;										//  LDROM modify Enable
		IAPFD = 0xFF;												// IMPORTANT !! To erase function must setting IAPFD = 0xFF 
    IAPCN = PAGE_ERASE_LDROM;
		for(u16Count=0x0000;u16Count<(u16IAPDataSize/PAGE_SIZE);u16Count++)						// Loop page erase LDROM special define address area.
    {        
        IAPAL = LOBYTE(u16Count*PAGE_SIZE + u16IAPStartAddress);
        IAPAH = HIBYTE(u16Count*PAGE_SIZE + u16IAPStartAddress);
        Trigger_IAP(); 
    } 
    clr_IAPUEN_LDUEN;										// Disable LDROM modify 
    clr_CHPCON_IAPEN;										// Disable IAP
}

/**
 * @brief       LDROM blank check
 * @param       u16IAPStartAddress define LDROM area start address
 * @param 			u16IAPDataSize define LDROM need be erase bytes size
 * @return      none
 * @details     Check each byte of LDROM is FFH or not.
 * @example			LDROM_BlanckCheck(0x0000,2048);
 */
void LDROM_Blank_Check(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;
    set_CHPCON_IAPEN;
    IAPAL = LOBYTE(u16IAPStartAddress);
    IAPAH = HIBYTE(u16IAPStartAddress);
    IAPCN = BYTE_READ_LDROM;

    for(u16Count=0;u16Count<u16IAPDataSize;u16Count++)
    {   
        IAPFD = 0x00;    
        Trigger_IAP();
        if(IAPFD != 0xFF)
					IAP_ERROR();
        IAPAL++;
        if(IAPAL == 0x00)
          IAPAH++;
    } 
    clr_CHPCON_IAPEN;
}

/**
 * @brief       LDROM program loop
 * @param       u16IAPStartAddress define LDROM area start address
 * @param 			u16IAPDataSize define LDROM need be erase bytes size
 * @return      none
 * @details     Copy IAPDataBuf to LDROM
 * @example			LDROM_Program(0x0000,1024);
 */
void LDROM_Program(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;

    set_CHPCON_IAPEN;
    set_IAPUEN_LDUEN;    
    IAPAL = LOBYTE(u16IAPStartAddress);
    IAPAH = HIBYTE(u16IAPStartAddress);
    IAPCN = BYTE_PROGRAM_LDROM;
    
    for(u16Count=0;u16Count<u16IAPDataSize;u16Count++)
    {   
        IAPFD = IAPDataBuf[u16Count];     
        Trigger_IAP();
        IAPAL++;
        if(IAPAL == 0)
        {
            IAPAH++;
        }
    } 
    clr_IAPUEN_LDUEN;
    clr_CHPCON_IAPEN;
}


/**
 * @brief       LDROM check loop
 * @param       u16IAPStartAddress define LDROM area start address
 * @param 			u16IAPDataSize define LDROM need be erase bytes size
 * @return      none
 * @details     Check with XRAM IAPDataBuf with LDROM
 * @example			LDROM_Program_Verify(0x0000,1024);
 */
void LDROM_Read_Verify(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;

    set_CHPCON_IAPEN;
    IAPAL = LOBYTE(u16IAPStartAddress);
    IAPAH = HIBYTE(u16IAPStartAddress);
    IAPCN = BYTE_READ_LDROM;
    for(u16Count=0;u16Count<u16IAPDataSize;u16Count++)
    {   
				IAPFD = 0x00;
				Trigger_IAP();
        if (IAPFD != IAPDataBuf[u16Count])    
						IAP_ERROR();
        IAPAL++;
        if(IAPAL == 0)
        {
            IAPAH++;
        }
    } 
    clr_CHPCON_IAPEN;
}

/**
 * @brief       Erase APROM  
 * @param       u16IAPStartAddress define APROM area start address
 * @param 			u16IAPDataSize define LDROM need be erase bytes size
 * @return      none
 * @details     Page erase APROM area base on data start address 
 * @example			Erase_APROM(0x0000,2048);
 */
void Erase_APROM(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;

    set_CHPCON_IAPEN;										// Enable IAP function
	  set_IAPUEN_APUEN;										// APROM modify Enable
		IAPFD = 0xFF;												// IMPORTANT !! To erase function must setting IAPFD = 0xFF 
    IAPCN = PAGE_ERASE_APROM;
		for(u16Count=0x0000;u16Count<u16IAPDataSize/PAGE_SIZE;u16Count++)						// Loop page erase APROM special define address area.
    {        
        IAPAL = LOBYTE(u16Count*PAGE_SIZE + u16IAPStartAddress);
        IAPAH = HIBYTE(u16Count*PAGE_SIZE + u16IAPStartAddress);
        Trigger_IAP(); 
    } 
    clr_IAPUEN_APUEN;										// Disable APROM modify 
    clr_CHPCON_IAPEN;										// Disable IAP
}

/**
 * @brief       APROM blank check
 * @param       u16IAPStartAddress define APROM area start address
 * @param 			u16IAPDataSize define APROM need be erase bytes size
 * @return      none
 * @details     Check each byte of APPROM is FFH or not.
 * @example			APROM_Blank_Check(0x0000,2048);
 */
void APROM_Blank_Check(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;
	
    set_CHPCON_IAPEN;
    IAPAL = LOBYTE(u16IAPStartAddress);
    IAPAH = HIBYTE(u16IAPStartAddress);
    IAPCN = BYTE_READ_APROM;
    for(u16Count=0;u16Count<u16IAPDataSize;u16Count++)
    {   
        IAPFD = 0x00;    
        Trigger_IAP();
        if(IAPFD != 0xFF)
					IAP_ERROR();
        IAPAL++;
        if(IAPAL == 0x00)
          IAPAH++;
    } 
    clr_CHPCON_IAPEN;
}

/**
 * @brief       APROM program loop
 * @param       u16IAPStartAddress define APROM area start address
 * @param 			u16IAPDataSize define APROM need be erase bytes size
 * @return      none
 * @details     Copy APDataBuf to APROM
 * @example			APROM_Program(0x0000,1024);
 */
void APROM_Program(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;

    set_CHPCON_IAPEN;
    set_IAPUEN_APUEN;    
    IAPAL = LOBYTE(u16IAPStartAddress);
    IAPAH = HIBYTE(u16IAPStartAddress);
    IAPCN = BYTE_PROGRAM_APROM;
    for(u16Count=0;u16Count<u16IAPDataSize;u16Count++)
    {   
        IAPFD=IAPDataBuf[u16Count];     
        Trigger_IAP();
        IAPAL++;
        if(IAPAL == 0)
        {
            IAPAH++;
        }
    } 
    clr_IAPUEN_APUEN;
    clr_CHPCON_IAPEN;
}


/**
 * @brief       APROM check loop
 * @param       u16IAPStartAddress define APROM area start address
 * @param 			u16IAPDataSize define APROM need be erase bytes size
 * @return      none
 * @details     Check with XRAM IAPDataBuf with APROM
 * @example			APROM_Program_Verify(0x0000,1024);
 */
void APROM_Read_Verify(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize)
{   
    unsigned int u16Count;

    set_CHPCON_IAPEN;
    IAPAL = LOBYTE(u16IAPStartAddress);
    IAPAH = HIBYTE(u16IAPStartAddress);
    IAPCN = BYTE_READ_APROM;
    for(u16Count=0;u16Count<u16IAPDataSize;u16Count++)
    {   
				IAPFD = 0x00;
				Trigger_IAP();
        if (IAPFD != IAPDataBuf[u16Count])     
						IAP_ERROR();
        IAPAL++;
        if(IAPAL == 0)
        {
            IAPAH++;
        }
    } 
    clr_CHPCON_IAPEN;
}


/**
 * @brief       Modify CONFIG  
 * @param       u8CF0,u8CF1,u8CF2,u8CF3,u8CF4,
 * @return      none
 * @details     Since CONFIG whole area include in 1 page, save old config define in xram, erase config and program as param define, if fail load saved config and program to recover.    
 * @example			Erase_CONFIG();
 */
void Modify_CONFIG(unsigned char u8CF0,unsigned char u8CF1,unsigned char u8CF2,unsigned char u8CF3,unsigned char u8CF4)
{   
    unsigned char u8Count;
	
/* Loop save original CONFIG data in XRAM	*/
		set_CHPCON_IAPEN;										// Enable IAP function
    IAPCN = BYTE_READ_CONFIG;						
		IAPAH = 0x00;
		for(u8Count=0;u8Count<5;u8Count++)	
    {        
        IAPAL = u8Count;
			  Trigger_IAP(); 
				IAPCFBuf[u8Count] = IAPFD;
    } 
/* Erase CONFIG setting		*/
	  set_IAPUEN_CFUEN;										// APROM modify Enable
		IAPFD = 0xFF;												// IMPORTANT !! To erase function must setting IAPFD = 0xFF 
    IAPCN = PAGE_ERASE_CONFIG;
		IAPAL = 0x00;
		Trigger_IAP();
/* Modify CONFIG setting as customer define */
		IAPCN = BYTE_PROGRAM_CONFIG;
		IAPFD = u8CF0;
		Trigger_IAP();
		IAPAL++;
		IAPFD = u8CF1;
		Trigger_IAP();
		IAPAL++;
		IAPFD = u8CF2;
		Trigger_IAP();
		IAPAL++;
		IAPFD = u8CF3;
		Trigger_IAP();
		IAPAL++;
		IAPFD = u8CF4;
		Trigger_IAP();
		clr_IAPUEN_CFUEN;
/* Check programed data, if not match, program the storaged before data.	*/
		IAPCN = BYTE_READ_CONFIG;
		IAPAL = 0x00;
		Trigger_IAP(); 
		if (IAPFD != u8CF0)
			goto MDCFEND;
		IAPAL++;
		Trigger_IAP(); 
		if (IAPFD != u8CF1)
			goto MDCFEND;
		IAPAL++;
		Trigger_IAP(); 
		if (IAPFD != u8CF2)
			goto MDCFEND;
		IAPAL++;
		Trigger_IAP(); 
		if (IAPFD != u8CF3)
			goto MDCFEND;
		IAPAL++;
		Trigger_IAP(); 
		if (IAPFD != u8CF4)
			goto MDCFEND;
		goto CFCLOSE;
MDCFEND:
		set_IAPUEN_CFUEN;											// APROM modify Enable
		for(u8Count=0;u8Count<5;u8Count++)		// Loop page erase APROM special define address area.
    {        
        IAPAL = u8Count;
				IAPFD = IAPCFBuf[u8Count];
			  Trigger_IAP(); 
    } 
CFCLOSE:
    clr_IAPUEN_CFUEN;										// Disable APROM modify 
    clr_CHPCON_IAPEN;										// Disable IAP
		

}

/**
 * @brief       Read UID loop
 * @param       none
 * @return      none
 * @details     IAP command read UID area storage data in XRAM LIB_UIDBuffer[0:8]
 * @example			UID_Read();
*/
void UID_Read(void)
{   
    unsigned char u8Count;

    set_CHPCON_IAPEN;
    IAPAL = 0x00;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    for(u8Count=0;u8Count<9;u8Count++)
    {   
				IAPFD = 0x00;
				Trigger_IAP();
        UIDBuffer[u8Count] = IAPFD ;
	      IAPAL++;
    } 
    clr_CHPCON_IAPEN;
}


/**
 * @brief       Read UCID loop
 * @param       none
 * @return      none
 * @details     IAP command read UCID area storage data in XRAM UCIDBuffer[0:8]
 * @example			UCID_Read();
 */
void UCID_Read(void)
{   
    unsigned char u8Count;

    set_CHPCON_IAPEN;
    IAPAL = 0x20;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    for(u8Count=0;u8Count<12;u8Count++)
    {   
				IAPFD = 0x00;
				Trigger_IAP();
        UCIDBuffer[u8Count] = IAPFD ;
	      IAPAL++;
    } 
    clr_CHPCON_IAPEN;
}

/**
 * @brief       Read UID loop
 * @param       none
 * @return      none
 * @details     IAP command read UID area storage data in XRAM LIB_UIDBuffer[0:8]
 * @example			UID_Read();
*/
void DID_Read(void)
{   
    unsigned char u8Count;

    set_CHPCON_IAPEN;
    IAPAL = 0x00;
    IAPAH = 0x00;
    IAPCN = READ_DID;
    for(u8Count=0;u8Count<4;u8Count++)
    {   
				IAPFD = 0x00;
				Trigger_IAP();
        DIDBuffer[u8Count] = IAPFD ;
	      IAPAL++;
    } 
    clr_CHPCON_IAPEN;
}