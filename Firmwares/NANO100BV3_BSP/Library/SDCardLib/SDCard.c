/****************************************************************************//**
 * @file     sdcard.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/06/03 3:44p $
 * @brief    Nano100 series SD Card source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "SDCARD.h"
#include "Nano100Series.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NANO100_SDCARD SDCARD Library
  @{
*/
/// @cond HIDDEN_SYMBOLS
int8_t Is_Initialized=0,SDtype=0;
uint32_t LogicSector=0;

// Command table for MMC.  This table contains all commands available in SPI
// mode;  Format of command entries is described above in command structure
// definition;
COMMAND __I command_list[] = {
    { 0,NO ,0x95,CMD,R1 ,NO },    // CMD0;  GO_IDLE_STATE: reset card;
    { 1,NO ,0xFF,CMD,R1 ,NO },    // CMD1;  SEND_OP_COND: initialize card;
    { 8,YES,0xFF,CMD,R7 ,NO },    // CMD8;  SEND_IF_COND
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
/// @endcond HIDDEN_SYMBOLS
/** @addtogroup NANO100_SDCARD_EXPORTED_FUNCTIONS SDCARD Library Exported Functions
  @{
*/

/**
  * @brief Delay function.
  * @param[in] count loop count.
  * @return none
  */
void SD_Delay(uint32_t count)
{
    uint32_t volatile loop;
    for(loop=0; loop<count; loop++);
}
/*---------------------------------------------------------------------------------------------------------*/
/* SD CARD Protocol                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/*
    Transfer Length:48bit
    BIT POSITION       WIDTH(BITS)          VAULE
            [47]                1               0:Start bit
            [46]                1               1:Transmit   0:Receive
         [45:40]                6               CMD8:001000
          [39:8]               32               Reserved
           [7:1]                7               Reserved
             [0]                1               1:End bit

*/

/**
  * @brief This function is used to generate CRC value
  * @param[in] u32Data Input Data
  * @param[in] u32GenPoly CRC7:0x1200, CRC16:0x1021
  * @param[in] u32Accum CRC value
  * @return CRC value
  */
static uint32_t GenerateCRC(uint32_t u32Data, uint32_t u32GenPoly, uint32_t u32Accum)
{
    volatile uint8_t i;

    u32Data <<= 8;
    for (i=8; i>0; i--) {
        if ((u32Data ^ u32Accum) & 0x8000)
            u32Accum = (u32Accum << 1) ^ u32GenPoly;
        else
            u32Accum <<= 1;
        u32Data <<= 1;
    }
    return u32Accum;
}

/**
  * @brief This function is used to send data though SPI to general clock for SDCARD operation
  * @param[in] u32Data Data to send
  * @return none
  */
static uint32_t SingleWrite(uint32_t u32Data)
{
    SPI_WRITE_TX0(SPI1, u32Data);
    SPI_TRIGGER(SPI1);
    while(SPI_IS_BUSY(SPI1));

    return SPI_READ_RX0(SPI1);
}

/**
  * @brief This function is used to Send SDCARD CMD and Receive Response
  * @param[in] nCmd Set command register
  * @param[in] nArg Set command argument
  * @param[out] *pchar Get register and data
  * @param[out] *response Get response
  * @retval TRUE get response
  * @retval FALSE 1.SD Card busy, 2.Card moved, 3.Timeout
  */
uint32_t MMC_Command_Exec (uint8_t nCmd, uint32_t nArg,uint8_t *pchar, uint32_t *response)
{
    uint8_t loopguard;
    COMMAND current_command;                // Local space for the command table
    UINT32  long_arg;                       // Local space for argument
    static uint32_t current_blklen = 512;
    uint32_t old_blklen = 512;
    int32_t counter = 0;                    // Byte counter for multi-byte fields;
    UINT16 card_response;                       // Variable for storing card response;
    uint8_t data_resp;                      // Variable for storing data response;
    UINT16 dummy_CRC;                       // Dummy variable for storing CRC field;

    card_response.i = 0;

    current_command = command_list[nCmd];// Retrieve desired command table entry
    // from code space;
    if(current_command.command_byte & 0x80) { // Detect ACMD
        if(MMC_Command_Exec(APP_CMD,EMPTY,EMPTY,response)==FALSE)//Send APP_CMD
            return FALSE;
    }

    SPI_SET_SS0_LOW(SPI1);  // CS = 0

    SingleWrite(0xFF);
    SingleWrite((current_command.command_byte | 0x40)&0x7f);
    DBG_PRINTF("CMD:%d,",current_command.command_byte&0x7f);

    long_arg.l = nArg;                  // Make argument byte addressable;
    // If current command changes block
    // length, update block length variable
    // to keep track;
    // Command byte = 16 means that a set
    // block length command is taking place
    // and block length variable must be
    // set;
    if(current_command.command_byte == 16) {
        current_blklen = nArg;
    }
    // Command byte = 9 or 10 means that a
    // 16-byte register value is being read
    // from the card, block length must be
    // set to 16 bytes, and restored at the
    // end of the transfer;
    if((current_command.command_byte == 9)||(current_command.command_byte == 10)) {
        old_blklen = current_blklen;     // Command is a GET_CSD or GET_CID,
        current_blklen = 16;             // set block length to 16-bytes;
    }
    // If an argument is required, transmit
    // one, otherwise transmit 4 bytes of
    // 0x00;
    if(current_command.arg_required == YES) {
        dummy_CRC.i = GenerateCRC((current_command.command_byte | 0x40), 0x1200, 0);
        for(counter=3; counter>=0; counter--) {
            SingleWrite(long_arg.b[counter]);
            dummy_CRC.i = GenerateCRC(long_arg.b[counter], 0x1200, dummy_CRC.i);

        }
        dummy_CRC.i = (dummy_CRC.i >> 8)| 0x01;
        SingleWrite(dummy_CRC.b[0]);
    } else {
        counter = 0;
        while(counter <= 3) {
            SingleWrite(0x00);
            counter++;
        }
        SingleWrite(current_command.CRC);
    }

    // The command table entry will indicate
    // what type of response to expect for
    // a given command;  The following
    // conditional handles the MMC response;
    if(current_command.response == R1) {    // Read the R1 response from the card;
        loopguard=0;
        do {
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R1:0x%x, counter:%d\n",card_response.b[0],loopguard);
        if(!loopguard) {
            BACK_FROM_ERROR;
        }
        *response=card_response.b[0];
    } else if(current_command.response == R1b) { // Read the R1b response;
        loopguard = 0;
        do {
            card_response.b[0] =  SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((card_response.b[0] & BUSY_BIT));
        while((SingleWrite(0xFF)&0xFF) == 0x00);
    } else if(current_command.response == R2) {
        loopguard=0;
        do {
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((card_response.b[0] & BUSY_BIT));
        card_response.b[1] = SingleWrite(0xFF);
        DBG_PRINTF("R2:0x%x, counter:%d\n",card_response.i,loopguard);
        if(!loopguard) {
            BACK_FROM_ERROR;
        }
        *response=card_response.i;
    } else if(current_command.response == R3) {
        // Read R3 response;
        loopguard=0;
        do {
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R3:0x%x, counter:%d\n",card_response.b[0],loopguard);
        if(!loopguard) {
            BACK_FROM_ERROR;
        }
        counter = 0;
        while(counter <= 3) {            // Read next three bytes and store them
            // in local memory;  These bytes make up
            counter++;                    // the Operating Conditions Register
            *pchar++ = SingleWrite(0xFF);
        }
        *response=card_response.b[0];
    } else {
        // Read R7 response;
        loopguard=0;
        do {
            card_response.b[0] = SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R7:0x%x, counter:%d\n",card_response.b[0],loopguard);
        if(!loopguard) {
            BACK_FROM_ERROR;
        }
        counter = 0;
        while(counter <= 3) {            // Read next three bytes and store them
            // in local memory;  These bytes make up
            counter++;                    // the Operating Conditions Register
            *pchar++ = SingleWrite(0xFF);
        }
        *response=card_response.b[0];
    }


    switch(current_command.trans_type) { // This conditional handles all data
    // operations;  The command entry
    // determines what type, if any, data
    // operations need to occur;
    case RDB:                         // Read data from the MMC;
        loopguard = 0;

        while((SingleWrite(0xFF)&0xFF)!=START_SBR) {
            if(!++loopguard) {
                BACK_FROM_ERROR;
            }
            SD_Delay(1);
        }
        counter = 0;                    // Reset byte counter;
        // Read <current_blklen> bytes;

        SPI_WRITE_TX0(SPI1, 0xFFFFFFFF);
        if(pchar) {
            //SPI_SetBitLength(SPI1, 8);
            //SPI_SetFIFOMode(SPI1, TRUE, 2);
            //SPI_FIFOReadWrite8(SPI1, NULL, pchar, current_blklen);
            //SPI_SetFIFOMode(SPI1, FALSE, 2);

            for (counter=0; counter<current_blklen; counter++) {
                SPI_WRITE_TX0(SPI1, 0xFF);
                SPI_TRIGGER(SPI1);
                while(SPI_IS_BUSY(SPI1));
                *(pchar+counter)=SPI_READ_RX0(SPI1);
            }
        } else {
            for (; counter<current_blklen; counter++) {
                SPI_TRIGGER(SPI1);
                while(SPI_IS_BUSY(SPI1));
            }
        }
        dummy_CRC.b[1] = SingleWrite(0xFF); // After all data is read, read the two
        dummy_CRC.b[0] = SingleWrite(0xFF); // CRC bytes;  These bytes are not used
        // in this mode, but the place holders
        // must be read anyway;
        break;
    case RD:                         // Read data from the MMC;
        loopguard = 0;

        while((SingleWrite(0xFF)&0xFF)!=START_SBR) {
            if(!++loopguard) {
                BACK_FROM_ERROR;
            }
        }
        counter = 0;                    // Reset byte counter;
        // Read <current_blklen> bytes;
        if(pchar) {
            for (counter=0; counter<current_blklen; counter++) {
                SPI_WRITE_TX0(SPI1, 0xFF);
                SPI_TRIGGER(SPI1);
                while(SPI_IS_BUSY(SPI1));
                *(pchar+counter)=SPI_READ_RX0(SPI1);
            }
        } else {
            for (counter=0; counter<current_blklen; counter++) {
                SPI_WRITE_TX0(SPI1, 0xFF);
                SPI_TRIGGER(SPI1);
                while(SPI_IS_BUSY(SPI1));
            }
        }
        dummy_CRC.b[1] = SingleWrite(0xFF); // After all data is read, read the two
        dummy_CRC.b[0] = SingleWrite(0xFF); // CRC bytes;  These bytes are not used
        // in this mode, but the place holders
        // must be read anyway;
        break;

    case WR:
        SingleWrite(0xFF);
        SingleWrite(START_SBW);

        for (counter=0; counter<current_blklen; counter++) {
            SPI_WRITE_TX0(SPI1, *(pchar+counter));
            SPI_TRIGGER(SPI1);
            dummy_CRC.i = GenerateCRC(*(pchar+counter), 0x1021, dummy_CRC.i);
            while(SPI_IS_BUSY(SPI1));
        }
        SingleWrite(dummy_CRC.b[1]);
        SingleWrite(dummy_CRC.b[0]);

        loopguard = 0;
        do {                          // Read Data Response from card;
            data_resp = SingleWrite(0xFF);
            if(!++loopguard) break;
        } while((data_resp & DATA_RESP_MASK) != 0x01);   // When bit 0 of the MMC response
        // is clear, a valid data response
        // has been received;

        if(!loopguard) {
            BACK_FROM_ERROR;
        }


        while((SingleWrite(0xFF)&0xFF)!=0xFF);//Wait for Busy
        SingleWrite(0xFF);
        break;
    default:
        break;
    }
    SPI_SET_SS0_HIGH(SPI1); // CS = 1
    if((current_command.command_byte == 9)||(current_command.command_byte == 10)) {
        current_blklen = old_blklen;
    }
    return TRUE;
}

/**
  * @brief This function is used to initialize the flash card
  * @return none
  */
void MMC_FLASH_Init(void)
{
    uint32_t response;
    uint16_t loopguard;
    uint32_t i;
    uint8_t counter = 0;
    uint8_t pchar[16];              // Data pointer for storing MMC
    uint32_t c_size,bl_len;
    uint8_t c_mult;


    Is_Initialized = 0;

re_init:
    SPI_SET_SS0_HIGH(SPI1); // CS = 1
    SD_Delay(1000);
    //--------------------------------------------------------
    //  Send 74 SD clcok in SD mode for Toshiba SD Card
    //--------------------------------------------------------
    for(counter = 0; counter < 10; counter++) {
        SingleWrite(0xFF);
    }
    SD_Delay(1000);

    SPI_SET_SS0_LOW(SPI1);  // CS = 0
    while(MMC_Command_Exec(GO_IDLE_STATE,EMPTY,EMPTY,&response)==FALSE)
        SD_Delay(1000);
    if(response!=0x01)
        goto re_init;

    if(MMC_Command_Exec(SEND_IF_COND,0x15A,pchar,&response) && response==1) {
        /* SDC ver 2.00 */
        if (pchar[2] == 0x01 && pchar[3] == 0x5A) {
            /* The card can work at VDD range of 2.7-3.6V */
            loopguard=0;
            do {
                MMC_Command_Exec(SD_SEND_OP_COND,0x40000000,EMPTY,&response);//Enable HCS(OCR[30])
                if(!++loopguard) break;
                SD_Delay(0x100);
            } while(response!=0);
            if(!loopguard)
                return;

            MMC_Command_Exec(READ_OCR,EMPTY,pchar,&response);
            SDtype=(pchar[0]&0x40)?SDv2|SDBlock:SDv2;
        }
    } else {
        /* SDv1 or MMCv3 */
        MMC_Command_Exec(SD_SEND_OP_COND,0x00,EMPTY,&response);
        if (response <= 1) {
            loopguard=0;
            do {
                MMC_Command_Exec(SD_SEND_OP_COND,0x00,EMPTY,&response);
                if(!++loopguard) break;
                SD_Delay(50);
            } while(response!=0);
            if(!loopguard)
                return;
            SDtype=SDv1;    /* SDv1 */
        } else {
            loopguard=0;
            do {
                MMC_Command_Exec(SEND_OP_COND,0x00,EMPTY,&response);
                if(!++loopguard) break;
                SD_Delay(50);
            } while(response!=0);
            if(!loopguard)
                return;
            SDtype=MMCv3;   /* MMCv3 */
        }
        MMC_Command_Exec(SET_BLOCKLEN,(uint32_t)PHYSICAL_BLOCK_SIZE,EMPTY,&response);
    }
    if(MMC_Command_Exec(SEND_CSD,EMPTY,pchar,&response)==FALSE)
        return;

    if(response==0) {
        DBG_PRINTF("Change speed:");
        for(i=0; i<16; i++) {
            DBG_PRINTF("0x%X ",pchar[i]);
        }

    } else {
        DBG_PRINTF("CARD STATUS 0x%X:\n",response);
        for(i=0; i<16; i++) {
            DBG_PRINTF("0x%X ",pchar[i]);
        }
        LogicSector=0;
        return;
    }

    if(SDtype&SDBlock) { // Determine the number of MMC sectors;
        bl_len = 1 << (pchar[5] & 0x0f) ;
        c_size = ((pchar[7] & 0x3F) << 16) |(pchar[8] << 8) | (pchar[9]);
        LogicSector=c_size*((512*1024)/bl_len);
    } else {
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
    Is_Initialized = 1;
}

/**
  * @brief This function is used to Open GPIO function and initial SDCARD
  * @retval SD_FAIL Initial SDCARD Failed
  * @retval SD_SUCCESS Success
  */
uint32_t SDCARD_Open(void)
{
    /* Configure SPI1 as a master, 8-bit transaction*/
    SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 8, 300000);

    SPI_DisableAutoSS(SPI1);
    SPI_SET_MSB_FIRST(SPI1);

    DBG_PRINTF("SPI is running at %d Hz\n", SPI_GetBusClock(SPI1));

    MMC_FLASH_Init();
    SD_Delay(3000);

    if (Is_Initialized)
        DBG_PRINTF("SDCARD INIT OK\n\n");
    else {
        DBG_PRINTF("SDCARD INIT FAIL\n\n");
        return SD_FAIL;
    }

    SPI_SetBusClock(SPI1, 5000000);
    DBG_PRINTF("Now, SPI is running at %d Hz\n", SPI_GetBusClock(SPI1));

    return SD_SUCCESS;
}

/**
  * @brief This function is used to close SDCARD
  * @return none
  */
void SDCARD_Close(void)
{
    SPI_Close(SPI1);
}

/**
  * @brief This function is used to get card total sector after SDCARD is opened
  * @param[out] *pu32TotSecCnt Get sector count
  * @retval TRUE The size is already saved
  * @retval FALSE The size is zero
  */
uint32_t SDCARD_GetCardSize(uint32_t *pu32TotSecCnt)
{
    if (LogicSector == 0)
        return FALSE;
    else
        *pu32TotSecCnt = LogicSector;

    return TRUE;
}

/**
  * @brief This function is used to get logic sector size
  * @return The Logic Sector size
  */
uint32_t GetLogicSector(void)
{
    return LogicSector;
}

/**
  * @brief This function is used to Get data from SD card
  * @param[in] addr Set start address for LBA
  * @param[in] size Set data size (byte)
  * @param[in] buffer Set buffer pointer
  * @return none
  */
void SpiRead(uint32_t addr, uint32_t size, uint8_t* buffer)
{
    /* This is low level read function of USB Mass Storage */
    uint32_t response;
    if(SDtype&SDBlock) {
        while(size >= PHYSICAL_BLOCK_SIZE) {
            MMC_Command_Exec(READ_SINGLE_BLOCK,addr,buffer,&response);
            addr   ++;
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }

    } else {
        addr*=PHYSICAL_BLOCK_SIZE;
        while(size >= PHYSICAL_BLOCK_SIZE) {
            MMC_Command_Exec(READ_SINGLE_BLOCK,addr,buffer,&response);
            addr   += PHYSICAL_BLOCK_SIZE;
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }
    }
}

/**
  * @brief This function is used to store data into SD card
  * @param[in] addr Set start address for LBA
  * @param[in] size Set data size (byte)
  * @param[in] buffer Set buffer pointer
  * @return none
  */
void SpiWrite(uint32_t addr, uint32_t size, uint8_t* buffer)
{
    uint32_t response;
    if(SDtype&SDBlock) {
        while(size >= PHYSICAL_BLOCK_SIZE) {
            MMC_Command_Exec(WRITE_BLOCK,addr,buffer,&response);
            addr   ++;
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }
    } else {
        addr*=PHYSICAL_BLOCK_SIZE;
        while(size >= PHYSICAL_BLOCK_SIZE) {
            MMC_Command_Exec(WRITE_BLOCK,addr,buffer,&response);
            addr   += (PHYSICAL_BLOCK_SIZE);
            buffer += PHYSICAL_BLOCK_SIZE;
            size  -= PHYSICAL_BLOCK_SIZE;
        }
    }
}
/*@}*/ /* end of group NANO100_SDCARD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO100_SDCARD_Driver */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
