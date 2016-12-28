/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
/*---------------------------------------------------------------------------------------------------------*/
/* Includes of local headers                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
//#include "NUC200Series.h"
#include "NUC230_240.h"
#include "ISP_SDCARD.h"
/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define DBG_PRINTF(...)		
//#define DBG_PRINTF printf

//SPI interface
#define SD_SPI SPI1

// Erase group size = 16 MMC FLASH sectors
#define PHYSICAL_BLOCK_SIZE     512 

// Command table value definitions
// Used in the MMC_Command_Exec function to 
// decode and execute MMC command requests
#define     EMPTY  0
#define     YES   1
#define     NO    0
#define     CMD   0
#define     RD    1
#define     WR    2
#define     RDB   3
#define     WDB   4
#define     R1    0
#define     R1b   1
#define     R2    2
#define     R3    3
#define     R7    4

// Start and stop data tokens for single and multiple
// block MMC data operations
#define     START_SBR      0xFE
#define     START_MBR      0xFE
#define     START_SBW      0xFE
#define     START_MBW      0xFC
#define     STOP_MBW       0xFD

// Mask for data response Token after an MMC write
#define     DATA_RESP_MASK 0x11

// Mask for busy Token in R1b response
#define     BUSY_BIT       0x80



//#define BACK_FROM_ERROR { SingleWrite(0xFF);SD_SPI->SSR = SPI_SSR_SW_SS_PIN_HIGH; return FALSE;}
#define BACK_FROM_ERROR { SingleWrite(0xFF);SPI_SET_SS0_HIGH(SPI1); return FALSE;}
#define _SPI_WRITE_TX_BUFFER0(port, x) ((port)->TX[0] = (x))
#define _SPI_WRITE_TX_BUFFER1(port, x) ((port)->TX[1] = (x))

typedef union {                        // byte-addressable unsigned long
    uint32_t l;
    uint8_t b[4];
    } UINT32;
typedef union {                        // byte-addressable unsigned int
    uint16_t i;
    uint8_t b[2];
    } UINT16;
// This structure defines entries into the command table;
typedef struct {
    uint8_t command_byte;      // OpCode;
    uint8_t arg_required;      // Indicates argument requirement;
    uint8_t u8CRC;               // Holds CRC for command if necessary;
    uint8_t trans_type;        // Indicates command transfer type;
    uint8_t response;          // Indicates expected response;
    uint8_t var_length;        // Indicates varialble length transfer;
} COMMAND;
// Command table for MMC.  This table contains all commands available in SPI
// mode;  Format of command entries is described above in command structure
// definition;
COMMAND __I command_list[] = {
    { 0,NO ,0x95,CMD,R1 ,NO },    // CMD0;  GO_IDLE_STATE: reset card;
    { 1,NO ,0xFF,CMD,R1 ,NO },    // CMD1;  SEND_OP_COND: initialize card;
    { 8,YES,0xFF,CMD,R7 ,NO },    // CMD8;	SEND_IF_COND
    { 9,NO ,0xFF,RD ,R1 ,NO },    // CMD9;  SEND_CSD: get card specific data;
    {10,NO ,0xFF,RD ,R1 ,NO },    // CMD10; SEND_CID: get card identifier;
    {12,NO ,0xFF,CMD,R1b,NO },    // CMD12; STOP_TRANSMISSION: end read;
    {13,NO ,0xFF,CMD,R2 ,NO },    // CMD13; SEND_STATUS: read card status;
    {16,YES,0xFF,CMD,R1 ,NO },    // CMD16; SET_BLOCKLEN: set block size;
    {17,YES,0xFF,RDB ,R1 ,NO },    // CMD17; READ_SINGLE_BLOCK: read 1 block;
    {18,YES,0xFF,RD ,R1 ,YES},    // CMD18; READ_MULTIPLE_BLOCK: read > 1;
    {23,NO ,0xFF,CMD,R1 ,NO },    // CMD23; SET_BLOCK_COUNT
    {24,YES,0xFF,WR ,R1 ,NO },    // CMD24; WRITE_BLOCK: write 1 block;
    {25,YES,0xFF,WR ,R1 ,YES},    // CMD25; WRITE_MULTIPLE_BLOCK: write > 1;
    {27,NO ,0xFF,CMD,R1 ,NO },    // CMD27; PROGRAM_CSD: program CSD;
    {28,YES,0xFF,CMD,R1b,NO },    // CMD28; SET_WRITE_PROT: set wp for group;
    {29,YES,0xFF,CMD,R1b,NO },    // CMD29; CLR_WRITE_PROT: clear group wp;
    {30,YES,0xFF,CMD,R1 ,NO },    // CMD30; SEND_WRITE_PROT: check wp status;
    {32,YES,0xFF,CMD,R1 ,NO },    // CMD32; TAG_SECTOR_START: tag 1st erase;
    {33,YES,0xFF,CMD,R1 ,NO },    // CMD33; TAG_SECTOR_END: tag end(single);
    {34,YES,0xFF,CMD,R1 ,NO },    // CMD34; UNTAG_SECTOR: deselect for erase;
    {35,YES,0xFF,CMD,R1 ,NO },    // CMD35; TAG_ERASE_GROUP_START;
    {36,YES,0xFF,CMD,R1 ,NO },    // CMD36; TAG_ERASE_GROUP_END;
    {37,YES,0xFF,CMD,R1 ,NO },    // CMD37; UNTAG_ERASE_GROUP;
    {38,YES,0xFF,CMD,R1b,NO },    // CMD38; ERASE: erase all tagged sectors;
    {42,YES,0xFF,CMD,R1 ,NO },    // CMD42; LOCK_UNLOCK;
    {55,NO ,0xFF,CMD,R1 ,NO },    // CMD55; APP_CMD	
    {58,NO ,0xFF,CMD,R3 ,NO },    // CMD58; READ_OCR: read OCR register;
    {59,YES,0xFF,CMD,R1 ,NO },    // CMD59; CRC_ON_OFF: toggles CRC checking;
    {0x80+13,NO ,0xFF,CMD,R2 ,NO },// ACMD13; SD_SEND_STATUS: read card status;
    {0x80+23,YES,0xFF,CMD,R1 ,NO },// ACMD23;SD_SET_WR_BLK_ERASE_COUNT	
    {0x80+41,YES,0xFF,CMD,R1 ,NO } // ACMD41; SD_SEND_OP_COND: initialize card;
};
/*---------------------------------------------------------------------------------------------------------*/
/* Parameter checking definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int8_t Is_Initialized=0,SDtype=0;
uint32_t LogicSector=0;

/*---------------------------------------------------------------------------------------------------------*/
/* SD CARD Protocol                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/*
	Transfer Length:48bit
	BIT POSITION       WIDTH(BITS)          VAULE
    [47]                1                    0:Start bit
    [46]                1                    1:Transmit   0:Receive
    [45:40]             6	                   CMD8:001000
    [39:8]              32                   Reserved
    [7:1]               7                    Reserved
    [0]                 1                    1:End bit

*/
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 GenerateCRC                                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               uint32_t u32Data         Input Data                                                         */
/*               uint32_t u32GenPoly      CRC7:0x1200   CRC16:0x1021                                         */
/*               uint32_t u32Accum        CRC value                                                          */
/*                                                                                                         */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               uint32_t u32Accum      CRC value                                                            */
/*               			                                                                               */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to generate CRC value                                               */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t GenerateCRC(uint32_t u32Data, uint32_t u32GenPoly, uint32_t u32Accum)
{
    volatile uint8_t i;

    u32Data <<= 8;
    for (i=8; i>0; i--)
    {
        if ((u32Data ^ u32Accum) & 0x8000)
            u32Accum = (u32Accum << 1) ^ u32GenPoly;
        else
            u32Accum <<= 1;
        u32Data <<= 1;
    }
    return u32Accum;
}  
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 SingleWrite                                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               UINT32 u32Data        Data to send                                                        */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               None			                                                                           */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to send data though SPI to general clock for SDCARD operation       */
/*---------------------------------------------------------------------------------------------------------*/
#if 0
static uint32_t SingleWrite(uint32_t u32Data)
{
    _SPI_WRITE_TX_BUFFER0(SD_SPI, u32Data);
    _SPI_SET_GO(SD_SPI);
    while (_SPI_GET_BUSY_STATUS(SD_SPI));
    return _SPI_GET_RX0_DATA(SD_SPI);
}
#endif
static uint32_t SingleWrite(uint32_t u32Data)
{
    SPI_WRITE_TX0(SPI1, u32Data);
    SPI_TRIGGER(SPI1);
    while(SPI_IS_BUSY(SPI1));

    return SPI_READ_RX0(SPI1);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 MMC_Command_Exec		                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               uint8_t 	nCmd          Set command register                                             */
/*               uint32_t 	nArg          Set command argument                                             */
/*               uint8_t 	*pchar        Get register and data                                            */
/*               uint32_t 	*response     Get response                                                     */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               TRUE	get response		                                                               */
/*               FALSE	1.SD Card busy, 2.Card moved, 3.Timeout	                                           */
/*					             																           */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to Send SDCARD CMD and Receive Response                             */
/*---------------------------------------------------------------------------------------------------------*/


#define _SPI_DISABLE_FIFO_MODE(port) ((port)->CNTRL &= (~SPI_CNTRL_FIFO_Msk))
#define _SPI_DISABLE_REORDER_FUNCTION(port) ((port)->CNTRL &= (~SPI_CNTRL_REORDER_Msk))
#define _SPI_GET_RX_FIFO_THRESHOLD_INT_FLAG(port) (((port)->STATUS & SPI_STATUS_RX_INTSTS_Msk)>>SPI_STATUS_RX_INTSTS_Pos)
#define _SPI_ENABLE_REORDER_FUNCTION(port) ((port)->CNTRL |= SPI_CNTRL_REORDER_Msk)
#define _SPI_ENABLE_FIFO_MODE(port) ((port)->CNTRL |= SPI_CNTRL_FIFO_Msk)
#define _SPI_SET_RX_THRESHOLD(port, x) ((port)->FIFO_CTL = ((port)->FIFO_CTL & (~SPI_FIFO_CTL_RX_THRESHOLD_Msk)) | (x)<<SPI_FIFO_CTL_RX_THRESHOLD_Pos)


uint32_t MMC_Command_Exec (uint8_t nCmd, uint32_t nArg,uint8_t *pchar, uint32_t *response)
{
    uint8_t	loopguard;
    COMMAND current_command;      			// Local space for the command table 
    UINT32	long_arg;               		// Local space for argument
    static uint32_t current_blklen = 512;
    uint32_t old_blklen = 512;     			
    int32_t counter = 0;     				// Byte counter for multi-byte fields;
    UINT16 card_response;           			// Variable for storing card response;
    uint8_t data_resp;      				// Variable for storing data response;
    UINT16 dummy_CRC;      					// Dummy variable for storing CRC field;

    card_response.i = 0;						  
                                     
    current_command = command_list[nCmd];// Retrieve desired command table entry
											// from code space;
    if(current_command.command_byte & 0x80)	// Detect ACMD
    {
        if(MMC_Command_Exec(APP_CMD,EMPTY,EMPTY,response)==FALSE)//Send APP_CMD
            return FALSE;
    }


    //SD_SPI->SSR = SPI_SSR_SW_SS_PIN_LOW;
		SPI_SET_SS0_LOW(SPI1); // CS = 0
    SingleWrite(0xFF);
    SingleWrite((current_command.command_byte | 0x40)&0x7f);
    DBG_PRINTF("CMD:%d,",current_command.command_byte&0x7f);

    long_arg.l = nArg;              	// Make argument byte addressable;
                                      		// If current command changes block
                                      		// length, update block length variable
                                      		// to keep track;
		                                    // Command byte = 16 means that a set
		                                    // block length command is taking place
		                                    // and block length variable must be
		                                    // set;
    if(current_command.command_byte == 16) 
        {current_blklen = nArg;} 
                                       // Command byte = 9 or 10 means that a
                                       // 16-byte register value is being read
                                       // from the card, block length must be
                                       // set to 16 bytes, and restored at the
                                       // end of the transfer;
    if((current_command.command_byte == 9)||(current_command.command_byte == 10)) 
    {
        old_blklen = current_blklen;     // Command is a GET_CSD or GET_CID,
        current_blklen = 16;             // set block length to 16-bytes;
    }
                                       // If an argument is required, transmit
                                       // one, otherwise transmit 4 bytes of
                                       // 0x00;
    if(current_command.arg_required == YES)
    {
        dummy_CRC.i = GenerateCRC((current_command.command_byte | 0x40), 0x1200, 0);
        for(counter=3;counter>=0;counter--) 
        {
            SingleWrite(long_arg.b[counter]);
            dummy_CRC.i = GenerateCRC(long_arg.b[counter], 0x1200, dummy_CRC.i);
        }
        dummy_CRC.i = (dummy_CRC.i >> 8)| 0x01;
        SingleWrite(dummy_CRC.b[0]);
    } else 
    {
        counter = 0;
        while(counter <= 3) 
        {
            SingleWrite(0x00);
            counter++;
        }
        SingleWrite(current_command.u8CRC);
//        SingleWrite(current_command.CRC);
    }
	
											// The command table entry will indicate
                                       		// what type of response to expect for
                                       		// a given command;  The following 
                                       		// conditional handles the MMC response;
    if(current_command.response == R1)  	// Read the R1 response from the card;
    {
        loopguard=0;
        do{
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        }while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R1:0x%x, counter:%d\n",card_response.b[0],loopguard);
        if(!loopguard){BACK_FROM_ERROR;}
        *response=card_response.b[0];
    }                                     
    else if(current_command.response == R1b)// Read the R1b response; 
    {
        loopguard = 0;	
        do {
            card_response.b[0] =  SingleWrite(0xFF);
            if(!++loopguard) break;
        }while((card_response.b[0] & BUSY_BIT));
        while((SingleWrite(0xFF)&0xFF) == 0x00);
    }
    else if(current_command.response == R2) 
    {
        loopguard=0;
        do{
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        }while((card_response.b[0] & BUSY_BIT));
        card_response.b[1] = SingleWrite(0xFF);
        DBG_PRINTF("R2:0x%x, counter:%d\n",card_response.i,loopguard);
        if(!loopguard) { BACK_FROM_ERROR; }
        *response=card_response.i;		
    }else if(current_command.response == R3)  
    {                               // Read R3 response;
    	loopguard=0;
      	do {
        	card_response.b[0] = SingleWrite(0xFF);
		    if(!++loopguard) break;
      	} while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R3:0x%x, counter:%d\n",card_response.b[0],loopguard);
        if(!loopguard) { BACK_FROM_ERROR; }
        counter = 0;
        while(counter <= 3)              // Read next three bytes and store them
        {                                // in local memory;  These bytes make up
            counter++;                    // the Operating Conditions Register
            *pchar++ = SingleWrite(0xFF);
        }
        *response=card_response.b[0];
    }else  
    {                               // Read R7 response;
        loopguard=0;
        do {
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R7:0x%x, counter:%d\n",card_response.b[0],loopguard);
        if(!loopguard) { BACK_FROM_ERROR; }
        counter = 0;
        while(counter <= 3)              // Read next three bytes and store them
        {                                // in local memory;  These bytes make up
            counter++;                    // the Operating Conditions Register
            *pchar++ = SingleWrite(0xFF);
        }
        *response=card_response.b[0];
    }
		
    switch(current_command.trans_type)  // This conditional handles all data 
    {                                   // operations;  The command entry
                                       // determines what type, if any, data
                                       // operations need to occur;
    case RDB:                         // Read data from the MMC;
        loopguard = 0;

        while((SingleWrite(0xFF)&0xFF)!=START_SBR) 
        {
            if(!++loopguard) {BACK_FROM_ERROR;}
            CLK_SysTickDelay(1);
        }		
        counter = 0;                  	// Reset byte counter;
                                       		// Read <current_blklen> bytes;
        _SPI_WRITE_TX_BUFFER0(SD_SPI, 0xFFFFFFFF);
				//SPI_WRITE_TX0(SPI1, 0xFFFFFFFF);
        if(pchar)
        {
            /*Set pchar+counter is a multiple of 4*/
            while(((uint32_t)pchar+counter)&0x03)
            {
               // _SPI_SET_GO(SD_SPI);
							SPI_TRIGGER(SPI1);
               // while (_SPI_GET_BUSY_STATUS(SD_SPI));
							while(SPI_IS_BUSY(SPI1));
                //*(pchar+counter++)=_SPI_GET_RX0_DATA(SD_SPI);
							 *(pchar+counter++)=SPI_READ_RX0(SPI1);
            }
            /*Read data by word*/
            //_SPI_SET_TRANSFER_BIT_LENGTH(SD_SPI,32);
						SPI_SET_DATA_WIDTH(SPI1,32);
            _SPI_ENABLE_REORDER_FUNCTION(SD_SPI);		
						_SPI_ENABLE_FIFO_MODE(SD_SPI);
						_SPI_SET_RX_THRESHOLD(SD_SPI, 1);
						_SPI_WRITE_TX_BUFFER0(SD_SPI, 0xFFFFFFFF);
            for (; counter<current_blklen-7; )
            {
                _SPI_WRITE_TX_BUFFER0(SD_SPI, 0xFFFFFFFF);
				//			 SPI_WRITE_TX0(SPI1, 0xffffffff);
                while (_SPI_GET_RX_FIFO_THRESHOLD_INT_FLAG(SD_SPI)==0);
                //*((uint32_t*)(pchar+counter))=_SPI_GET_RX0_DATA(SD_SPI);
							*((uint32_t*)(pchar+counter))=SPI_READ_RX0(SPI1);
                counter+=4;	
            }
						//*((uint32_t*)(pchar+counter))=_SPI_GET_RX0_DATA(SD_SPI);
						*((uint32_t*)(pchar+counter))=SPI_READ_RX0(SPI1);
						counter+=4;
            _SPI_DISABLE_REORDER_FUNCTION(SD_SPI);
						_SPI_DISABLE_FIFO_MODE(SD_SPI);
            //_SPI_SET_TRANSFER_BIT_LENGTH(SD_SPI,8);
						SPI_SET_DATA_WIDTH(SPI1,8);
            /*Read data by byte*/
            for (; counter<current_blklen; counter++)
            {
              //  _SPI_SET_GO(SD_SPI);
							SPI_TRIGGER(SPI1);
                //while (_SPI_GET_BUSY_STATUS(SD_SPI));
							 while(SPI_IS_BUSY(SPI1));
              //  *(pchar+counter)=_SPI_GET_RX0_DATA(SD_SPI);				
							*(pchar+counter)=SPI_READ_RX0(SPI1);
							
            }
        }else
        {
            for (; counter<current_blklen; counter++)
            { 
                //_SPI_SET_GO(SD_SPI);
							SPI_TRIGGER(SPI1);
                //while (_SPI_GET_BUSY_STATUS(SD_SPI));							
while(SPI_IS_BUSY(SPI1));							
            }
        }
           	dummy_CRC.b[1] = SingleWrite(0xFF);	// After all data is read, read the two
           	dummy_CRC.b[0] = SingleWrite(0xFF);	// CRC bytes;  These bytes are not used
                               					// in this mode, but the placeholders 
                   								// must be read anyway;			      
        break;
    case RD:                         // Read data from the MMC;
        loopguard = 0;

        while((SingleWrite(0xFF)&0xFF)!=START_SBR) 
        {
            if(!++loopguard) {BACK_FROM_ERROR;}
        }		
        counter = 0;                  	// Reset byte counter;
                                       		// Read <current_blklen> bytes;
        if(pchar)
        {
            for (counter=0; counter<current_blklen; counter++)
            { 
                _SPI_WRITE_TX_BUFFER0(SD_SPI, 0xFF);
				//			SPI_WRITE_TX0(SPI1, 0xFF);
             //   _SPI_SET_GO(SD_SPI);
							  SPI_TRIGGER(SPI1);
                //while (_SPI_GET_BUSY_STATUS(SD_SPI));
							 while(SPI_IS_BUSY(SPI1));
                ///*(pchar+counter)=_SPI_GET_RX0_DATA(SD_SPI);										
							*(pchar+counter)=SPI_READ_RX0(SPI1);
            }
        }else
        {
            for (counter=0; counter<current_blklen; counter++)
            { 
                _SPI_WRITE_TX_BUFFER0(SD_SPI, 0xFF);
							 //SPI_WRITE_TX0(SPI1, 0xFF);
                //_SPI_SET_GO(SD_SPI);
							SPI_TRIGGER(SPI1);
                //while (_SPI_GET_BUSY_STATUS(SD_SPI));								
							while(SPI_IS_BUSY(SPI1));
            }
        }
        dummy_CRC.b[1] = SingleWrite(0xFF);	// After all data is read, read the two
        dummy_CRC.b[0] = SingleWrite(0xFF);	// CRC bytes;  These bytes are not used
                               					// in this mode, but the placeholders 
                   								// must be read anyway;			      
        break;

    case WR: 			
        SingleWrite(0xFF);
        SingleWrite(START_SBW);
          
        for (counter=0; counter<current_blklen; counter++)
        {
            _SPI_WRITE_TX_BUFFER0(SD_SPI, *(pchar+counter)); 
					//SPI_WRITE_TX0(SPI1, *(pchar+counter));
           // _SPI_SET_GO(SD_SPI);
					SPI_TRIGGER(SPI1);
            dummy_CRC.i = GenerateCRC(*(pchar+counter), 0x1021, dummy_CRC.i);				
            //while (_SPI_GET_BUSY_STATUS(SD_SPI));
					while(SPI_IS_BUSY(SPI1));
        }
        SingleWrite(dummy_CRC.b[1]);
        SingleWrite(dummy_CRC.b[0]);
	
        loopguard = 0;
        do                            // Read Data Response from card;
        {  
            data_resp = SingleWrite(0xFF);
            if(!++loopguard) break;
        }while((data_resp & DATA_RESP_MASK) != 0x01);	// When bit 0 of the MMC response
	                                       					// is clear, a valid data response
	                                       					// has been received;
	        
        if(!loopguard) { BACK_FROM_ERROR; }


        while((SingleWrite(0xFF)&0xFF)!=0xFF);//Wait for Busy
        SingleWrite(0xFF);	        
        break;
    default: break;
    }
    SingleWrite(0xFF);
   // SD_SPI->SSR = SPI_SSR_SW_SS_PIN_HIGH;
		SPI_SET_SS0_HIGH(SPI1);  // CS = 0
    if((current_command.command_byte == 9)||(current_command.command_byte == 10)) {
        current_blklen = old_blklen;    
    }
    DBG_PRINTF("True\n");
    return TRUE;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 MMC_FLASH_Init                                                                            */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               None                                                                                      */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               None                                                                                      */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to initialize the flash card                                        */
/*---------------------------------------------------------------------------------------------------------*/
void MMC_FLASH_Init(void)
{
    uint32_t response;
    uint8_t loopguard;
    uint32_t i;
    uint8_t counter = 0;    		
    uint8_t pchar[16];         		// Data pointer for storing MMC 
    uint32_t c_size,bl_len;
    uint8_t c_mult;


    Is_Initialized = 0;

    //SD_SPI->SSR = SPI_SSR_SW_SS_PIN_HIGH;
	  SPI_SET_SS0_HIGH(SPI1);  // CS = 0
    CLK_SysTickDelay(1000);
    //--------------------------------------------------------
    //	Send 74 SD clcok in SD mode for Toshiba SD Card
    //--------------------------------------------------------	
    for(counter = 0; counter < 10; counter++) {
        SingleWrite(0xFF);
    }
    CLK_SysTickDelay(1000);

    //SD_SPI->SSR = SPI_SSR_SW_SS_PIN_LOW;
		 SPI_SET_SS0_LOW(SPI1);  // CS = 0

    while(MMC_Command_Exec(GO_IDLE_STATE,EMPTY,EMPTY,&response)==FALSE);
    if(response!=0x01)
        return;

    if(MMC_Command_Exec(SEND_IF_COND,0x15A,pchar,&response) && response==1)
    {/* SDC ver 2.00 */
        if (pchar[2] == 0x01 && pchar[3] == 0x5A) 
        {	/* The card can work at vdd range of 2.7-3.6V */
            loopguard=0;
            do
            {
                MMC_Command_Exec(SD_SEND_OP_COND,0x40000000,EMPTY,&response);//Enable HCS(OCR[30])
                if(!++loopguard) break;
                CLK_SysTickDelay(50);
            }while(response!=0);
            if(!loopguard) return;

            MMC_Command_Exec(READ_OCR,EMPTY,pchar,&response);
            SDtype=(pchar[0]&0x40)?SDv2|SDBlock:SDv2;
            DBG_PRINTF("SDv2\n");
        }
    }else
    {/* SDv1 or MMCv3 */
        MMC_Command_Exec(SD_SEND_OP_COND,0x00,EMPTY,&response);
        if (response <= 1) 
        {
            loopguard=0;
            do
            {
                MMC_Command_Exec(SD_SEND_OP_COND,0x00,EMPTY,&response);
                if(!++loopguard) break;
                    CLK_SysTickDelay(50);
            }while(response!=0);
            if(!loopguard) return;
            SDtype=SDv1;	/* SDv1 */
            DBG_PRINTF("SDv1\n");
        } else 
        {
            loopguard=0;
            do
            {
                MMC_Command_Exec(SEND_OP_COND,0x00,EMPTY,&response);
                if(!++loopguard) break;
                CLK_SysTickDelay(50);
            }while(response!=0);
            if(!loopguard) return;
            SDtype=MMCv3;	/* MMCv3 */
            DBG_PRINTF("MMCv3\n");
        }
        MMC_Command_Exec(SET_BLOCKLEN,(uint32_t)PHYSICAL_BLOCK_SIZE,EMPTY,&response);
    }
    if(MMC_Command_Exec(SEND_CSD,EMPTY,pchar,&response)==FALSE)
        return;

    if(response==0) 
    {
        DBG_PRINTF("Change speed:");
        for(i=0;i<16;i++) {	DBG_PRINTF("0x%X ",pchar[i]);}

    } else {
        DBG_PRINTF("CARD STATUS 0x%X:\n",response);
        for(i=0;i<16;i++) {	DBG_PRINTF("0x%X ",pchar[i]);}
        LogicSector=0;
        return;
    }

    if(SDtype&SDBlock)// Determine the number of MMC sectors;
    {
        bl_len = 1 << (pchar[5] & 0x0f) ;
        c_size = ((pchar[7] & 0x3F) << 16) |(pchar[8] << 8) | (pchar[9]);
        LogicSector=c_size*((512*1024)/bl_len);
    }else
    {
        bl_len = 1 << (pchar[5] & 0x0f) ;
        c_size = ((pchar[6] & 0x03) << 10) |(pchar[7] << 2) | ((pchar[8] &0xc0) >> 6);
        c_mult = (((pchar[9] & 0x03) << 1) | ((pchar[10] & 0x80) >> 7));
        LogicSector=(c_size+1)*(1 << (c_mult+2))*(bl_len/512);
    }                                     
    DBG_PRINTF("\nLogicSector:%d, PHYSICAL_SIZE:%dMB\n",LogicSector,(LogicSector/2/1024));

    loopguard = 0;
    while((MMC_Command_Exec(READ_SINGLE_BLOCK,0,0,&response)==FALSE)) {
        if(!++loopguard) break;
    } 
    //MMC_Command_Exec(GO_IDLE_STATE,EMPTY,EMPTY,&response);
    Is_Initialized = 1;
}					  
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 DrvSDCARD_Open                                                                            */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               None                                                                                      */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               E_DRVSDCARD_INITIAL_FAIL    Intial SDCARD Failed                                          */
/*               E_SUCCESS			         Success                                                       */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to Open GPIO function and intial SDCARD                             */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t DrvSDCARD_Open(void)
{
	   GPIO_SetMode(PC, BIT12, GPIO_PMD_OUTPUT);
	 PC12 = 0;
		
	/* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);
		
    /* Select HCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL1_SPI1_S_HCLK, MODULE_NoMsk);


    /* Setup SPI0 multi-function pins */
    SYS->GPC_MFP = SYS_GPC_MFP_PC8_SPI1_SS0 | SYS_GPC_MFP_PC9_SPI1_CLK | SYS_GPC_MFP_PC10_SPI1_MISO0 | SYS_GPC_MFP_PC11_SPI1_MOSI0;
    SYS->ALT_MFP = SYS_ALT_MFP_PC8_SPI1_SS0 | SYS_ALT_MFP_PC9_SPI1_CLK | SYS_ALT_MFP_PC10_SPI1_MISO0 | SYS_ALT_MFP_PC11_SPI1_MOSI0;
 	 /* Configure SPI1 as a master, 8-bit transaction*/
    SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 8, 300000);

    SPI_DisableAutoSS(SPI1);
    SPI_SET_MSB_FIRST(SPI1);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    //SD_SPI->SSR = SPI_SSR_SW_SS_PIN_HIGH;
	    SPI_SET_SS0_HIGH(SPI1); // CS = 1
	
   _SPI_WRITE_TX_BUFFER0(SD_SPI, 0xFFFFFFFF);
	 //SPI_WRITE_TX0(SPI1, 0xFFFFFFFF);
    _SPI_WRITE_TX_BUFFER1(SD_SPI, 0xFFFFFFFF);
		 //SPI_WRITE_TX0(SPI1, 0xFFFFFFFF);
	
    MMC_FLASH_Init();
    CLK_SysTickDelay(1000);
    SPI_SetBusClock(SPI1, 24000000);
    if (Is_Initialized)
    {
        DBG_PRINTF("SDCARD INIT OK\n\n");
        return E_SUCCESS;
    }
    else
        return E_DRVSDCARD_INITIAL_FAIL;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 DrvSDCARD_Close                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               None                                                                                      */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               None		                                                                               */
/*               			                                                                               */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to close SDCARD                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#if 0
void DrvSDCARD_Close(void)
{
		/*Reset SPI1*/
    SYS->IPRSTC2 |= SYS_IPRSTC2_SPI1_RST_Msk;	
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI1_RST_Msk;	
    SYSCLK->APBCLK &= ~SYSCLK_APBCLK_SPI1_EN_Msk;
    SYS->GPC_MFP = ~(SYS_GPC_MFP_PC11_Msk | SYS_GPC_MFP_PC10_Msk | SYS_GPC_MFP_PC9_Msk | SYS_GPC_MFP_PC8_Msk);
}
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 DrvSDCARD_GetCardSize                                                                     */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               PUINT32 pu32TotSecCnt                                                                     */
/*               			                                                                               */
/* Returns:                                                                                                */
/*               TRUE	The size is already saved in arg1                                                  */
/*               FALSE	The size is zero                                                                   */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to get card total sector after SDCARD is opened                     */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t DrvSDCARD_GetCardSize(uint32_t* pu32TotSecCnt)
{
    if (LogicSector == 0)
        return FALSE;
    else 
        *pu32TotSecCnt = LogicSector;

    return TRUE;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 GetLogicSector                                                                            */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               None                                                                                      */
/*               			                                                                               */
/* Returns:                                                                                                */
/*               The Logic Sector size                                                                     */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to get card total sector after SDCARD is opened                     */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetLogicSector(void)
{
    return LogicSector;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 DrvSDCARD_GetVersion                                                                      */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               None                                                                                      */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               SDCARD Library Version                                                                    */
/*               			                                                                               */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to Get SD driver version                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t DrvSDCARD_GetVersion(void)
{
    return (DRVSDCARD_MAJOR_NUM << 16) | (DRVSDCARD_MINOR_NUM << 8) | DRVSDCARD_BUILD_NUM;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 SpiRead                                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               uint32_t addr		Set start address for LBA                                              */
/*               uint32_t size      Set data size (byte)                                                   */
/*               uint32_t buffer    Set buffer pointer                                                     */
/*																		                                   */
/* Returns:                                                                                                */
/*               SDCARD Library Version                                                                    */
/*               			                                                                               */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to Get data from SD card                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SpiRead(uint32_t addr, uint32_t size, uint8_t* buffer)
{
    /* This is low level read function of USB Mass Storage */
    uint32_t response;
		if(addr>=LogicSector)
		{
			DBG_PRINTF("Read illegal Sector:0x%x\n",addr);
			return;
		}
    if(SDtype&SDBlock)
    {

        while(size >= PHYSICAL_BLOCK_SIZE)
        {
            while(MMC_Command_Exec(READ_SINGLE_BLOCK,addr,buffer,&response)==FALSE);
            addr   ++;
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }
    }else
    {
        addr*=PHYSICAL_BLOCK_SIZE;
        while(size >= PHYSICAL_BLOCK_SIZE)
        {
            while(MMC_Command_Exec(READ_SINGLE_BLOCK,addr,buffer,&response)==FALSE);
            addr   += PHYSICAL_BLOCK_SIZE;
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
		}
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: 	 SpiWrite                                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               uint32_t addr		Set start address LBA                                                  */
/*               uint32_t size      Set data size (byte)                                                   */
/*               uint32_t buffer    Set buffer pointer                                                     */
/*																		                                   */
/* Returns:                                                                                                */
/*               SDCARD Library Version                                                                    */
/*               			                                                                               */
/*                                                                                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*               This function is used to store data into SD card                                          */
/*---------------------------------------------------------------------------------------------------------*/
void SpiWrite(uint32_t addr, uint32_t size, uint8_t* buffer)
{
    uint32_t response;
		if(addr>=LogicSector)
		{
			DBG_PRINTF("Write illegal Sector:0x%x\n",addr);
			return;
		}			
    if(SDtype&SDBlock)
    {
        while(size >= PHYSICAL_BLOCK_SIZE)
        {
            while(MMC_Command_Exec(WRITE_BLOCK,addr,buffer,&response)==FALSE);
            addr   ++;
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }
    }else
    {
        addr*=PHYSICAL_BLOCK_SIZE;
        while(size >= PHYSICAL_BLOCK_SIZE)
        {
            while(MMC_Command_Exec(WRITE_BLOCK,addr,buffer,&response)==FALSE);
            addr   += (PHYSICAL_BLOCK_SIZE);
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }
    }
}




