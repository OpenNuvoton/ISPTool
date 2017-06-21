/**************************************************************************//**
 * @file     smartcard_t1protocol.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/07/31 7:25p $
 * @brief    This file handles T=1 protocol
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sclib_int.h"

/** @addtogroup NUC400_Library NUC400 Library
  @{
*/

/** @addtogroup NUC400_SCLIB Smartcard Library
  @{
*/

/** @addtogroup NUC400_SCLIB_PRIVATE_FUNCTIONS Smartcard Library Private Functions
  @{
*/

const uint16_t crc16a[] = {
    0000000,  0140301,  0140601,  0000500,
    0141401,  0001700,  0001200,  0141101,
    0143001,  0003300,  0003600,  0143501,
    0002400,  0142701,  0142201,  0002100,
};
const uint16_t crc16b[] = {
    0000000,  0146001,  0154001,  0012000,
    0170001,  0036000,  0024000,  0162001,
    0120001,  0066000,  0074000,  0132001,
    0050000,  0116001,  0104001,  0043000,
};

// Buffer use for holding T = 1 Tx/Rx block
extern uint8_t g_RxBUF[SCLIB_MAX_T1_BUFFER_SIZE];
extern uint8_t g_TxBUF[SCLIB_MAX_T1_BUFFER_SIZE];


/**
  * @brief  This routine calculates the epilogue field for a T1 block. It calculates the LRC
  *         for all the data in the IBlock.
  * @param[in]  block   T1 block, to be sent, or just read, from the card.
  * @param[in]  edc     ErrorDetectionCode as described in ISO
  * @param[in]  verify  If this is a block that was received form the card, TRUE will cause this routine
  *                 to check the epilogue field, included with this buffer, against the calculated one
  * @return Check sum calculate result
  * @retval TRUE if Verify = TRUE and epilogue fields match or Verify = FALSE
  * @retval FALSE if Verify = TRUE and an error was detected (mismatch)
  */
static uint32_t _SCLIB_CalcT1Chksum(uint8_t *block, uint8_t edc, uint32_t verify)
{
    uint32_t fRet = TRUE;
    uint16_t i;
    uint8_t lrc, tmp;
    uint16_t crc = 0;
    uint32_t offset = block[2] + 3; // modified by smfan

    if (edc & SCLIB_T1_CRC_CHECK) {
        // Calculate CRC using tables.
        for ( i = 0; i < offset;  i++) {

            tmp = block[i] ^ (uint8_t) crc;
            crc = (crc >> 8) ^ crc16a[tmp & 0x0f] ^ crc16b[tmp >> 4];
        }

        if (verify) {
            return((crc == (block[offset + 1] | (block[offset] << 8))) ? TRUE : FALSE);
        } else {
            block[offset] = (uint8_t) (crc >> 8 );       //MSB of crc
            block[offset + 1] = (uint8_t) (crc & 0x00ff);  //LSB of crc
        }
    } else {
        // Calculate LRC by X-Oring all the bytes.
        lrc = block[0];

        for(i = 1; i < offset; i++) {
            lrc ^= block[i];
        }

        if (verify) {
            return (lrc == block[offset] ? TRUE : FALSE);
        } else {
            block[offset] = lrc;
        }
    }
    return fRet;
}



/**
  * @brief Check the type of input block
  * @param[in]  block  The received block data from card
  * @return The type of block
  * @return \ref SCLIB_T1_BLOCK_I Received block is an I block
  * @return \ref SCLIB_T1_BLOCK_R Received block is an R block
  * @return \ref SCLIB_T1_BLOCK_S Received block is an S block
  */
int32_t _SCLIB_GetBlockType(uint8_t* block)
{
    if ((block[1] & 0x80) == SCLIB_T1_BLOCK_I)
        return SCLIB_T1_BLOCK_I;

    return (block[1] & 0xC0);
}


/**
  * @brief  Get the send sequence number of protocol control byte in prologue field of I block
  * @param[in]  block  The received block data from card
  * @return The send-sequence number of I-Block
  */
__STATIC_INLINE uint8_t _SCLIB_GetNS(uint8_t* block)
{
    return ((block[1] >> 6) & 0x01);
}

/**
  * @brief  Get the more bit of protocol control byte (PCB) in prologue field
  * @param[in]  block   The received block data from card
  * @return The more bit of I block
  * @return 1 Subsequent block follows
  * @return 0 Last block of the chain
  */
__STATIC_INLINE uint8_t _SCLIB_GetMBit(uint8_t* block)
{
    return ((block[1] >> 5) & 0x01);
}

/**
  * @brief  Get the send sequence number of protocol control byte in prologue field of R block
  * @param[in]  block   The received block data from card
  * @retval The send-sequence number of R-Block
  */
__STATIC_INLINE uint8_t _SCLIB_GetNR(uint8_t* block)
{
    return ((block[1] >> 4) & 0x01);
}


/**
  * @brief  Get the start pointer of information field
  * @param[in]  block  The received block data from card
  * @param[in]  len  The byte length of received block
  * @return The start pointer INF field
  * @retval NULL INF field does not exist on this block
  * @retval Others Pointer to first byte of INF
  */
__STATIC_INLINE uint8_t* _SCLIB_GetINF (uint8_t* block, int len)
{
    if (len < 5)
        return NULL;

    return block + 3;
}

/**
  * @brief  Get the content of length (LEN) byte in prologue field
  * @param[in] block The received block data from card
  * @return The content of LEN byte
  */
uint8_t _SCLIB_GetLEN(uint8_t* block)
{
    return block[2];
}

/**
  * @brief Get the node address (NAD) byte in prologue field
  * @param[in] block  The received block data from card
  * @return The content of NAD byte
  */
uint8_t _SCLIB_GetNAD(uint8_t* block)
{
    return block[0];
}

/* record response type; if prior one belongs to next one, retransmit counter will increase one */
static int32_t T1BlockRetransmitCheck(uint8_t *recBlock, int32_t retransmit, uint8_t *prior_rsptype, uint8_t *cur_rsptype)
{
    if(retransmit==0) {
        *prior_rsptype = _SCLIB_GetBlockType(recBlock);
        retransmit++;
    } else {
        *cur_rsptype = _SCLIB_GetBlockType(recBlock);
        if(*cur_rsptype == *prior_rsptype)
            retransmit++;
        else
            retransmit = 0;
        *prior_rsptype = _SCLIB_GetBlockType(recBlock);
    }

    return retransmit;
}

#if 0
/* Check CLA byte format */
static uint8_t T1BlockCLACheck(uint8_t *recBlock)
{
    if(recBlock[2]>=0x5)
        return (0x80 & recBlock[3]);
    else
        return 0x00;
}
#endif

/**
  * @brief  Check the received data if correctly
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
static int _SCLIB_PrecessT1Response(uint32_t num)
{
    int32_t i;
    int32_t readByte, err;
    uint32_t rBufLen, outlen;
    uint8_t rBuf[SCLIB_MAX_T1_BLOCK_SIZE];
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];


    /* [EMV 2000] */
    while((sc->TRSR & SC_TRSR_TX_ATV_Msk) && (dev->openflag != 0)) ;

    dev->errno = SCLIB_SUCCESS;
    dev->rcv_len = 3;   // Prologue field length
    dev->rcv_cnt = 0;
    dev->rcv_pos = 0;
    dev->op_state = SCLIB_OP_READ;

    while(dev->op_state == SCLIB_OP_READ && dev->openflag != 0) ;
    if((err = dev->errno) == SCLIB_ERR_TIME0OUT)
        return dev->errno;
    if(dev->openflag == 0)
        return SCLIB_ERR_CARD_REMOVED;


    rBufLen = 0;
    outlen = dev->snd_len;

    if (outlen < 3)
        return SCLIB_ERR_T1_PROTOCOL;

    readByte = dev->rcv_buf[0];
    rBuf[rBufLen] = readByte;
    rBufLen++;

    readByte = dev->rcv_buf[1];
    rBuf[rBufLen] = readByte;
    rBufLen++;

    readByte = dev->rcv_buf[2];
    rBuf[rBufLen] = readByte;
    rBufLen++;


    dev->errno = SCLIB_SUCCESS;
    dev->rcv_len = readByte + (dev->CardCapabilities.T1.EDC ? 2 : 1);   // Information field length + epilogue field
    dev->rcv_cnt = 0;
    dev->rcv_pos = rBufLen;
    dev->op_state = SCLIB_OP_READ;
    while(dev->op_state == SCLIB_OP_READ && dev->openflag != 0) ;
    if(dev->errno != SCLIB_SUCCESS)
        return dev->errno;
    if(err)
        return err;
    if(dev->openflag == 0)
        return SCLIB_ERR_CARD_REMOVED;

    SC_StopTimer(sc, 0);    // stop timer 0

    /* check that LEN doesn't exceed 254 */
    if (readByte > SCLIB_MAX_T1_BLOCK_INF_SIZE)
        return SCLIB_ERR_T1_PROTOCOL;

    /* read the information field */
    if (readByte >= 0) {
        for (i = 0 ; i < (readByte + (dev->CardCapabilities.T1.EDC ? 2 : 1)); i++)
            rBuf[3 + i] = dev->rcv_buf[3 + i];
        rBufLen += readByte;
    }

    /* read the EDC */
    if (_SCLIB_CalcT1Chksum(rBuf, dev->CardCapabilities.T1.EDC, TRUE) == FALSE ) {
        return SCLIB_ERR_T1_CHECKSUM;
    }

    SCLIB_T1INFO("T1CommandResponseProccessor: ");
    for(i = 0; i < (readByte + 3 + (dev->CardCapabilities.T1.EDC ? 2 : 1)); i++) {
        SCLIB_T1INFO("0x%02X ", rBuf[i]);
    }
    SCLIB_T1INFO("\n");

    return dev->errno;
}


/**
  * @brief  Send T=1 Block packets and block is ready to send
  * @param  dev     Data structure of smartcard information
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int _SCLIB_SendBlock (uint32_t num)
{
    int retVal;
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];
    uint8_t *sendbuf = dev->snd_buf;


    if (_SCLIB_GetBlockType(sendbuf) == SCLIB_T1_BLOCK_I)
        SCLIB_T1INFO( ">>>>>>>> SENDING I(%d, %d)\n", _SCLIB_GetNS(sendbuf), _SCLIB_GetMBit(sendbuf));
    else if (_SCLIB_GetBlockType(sendbuf) == SCLIB_T1_BLOCK_R)
        SCLIB_T1INFO( ">>>>>>>> SENDING R(%d)\n", _SCLIB_GetNR(sendbuf));
    else if (_SCLIB_GetBlockType(sendbuf) == SCLIB_T1_BLOCK_S) {
        switch (sendbuf[1]) {
        case SCLIB_T1_BLOCK_S_RESYNCH_REQ:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(RESYNCH request)\n");
            break;
        case SCLIB_T1_BLOCK_S_RESYNCH_RES:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(RESYNCH response)\n");
            break;
        case SCLIB_T1_BLOCK_S_IFS_REQ:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(IFS request)\n");
            break;
        case SCLIB_T1_BLOCK_S_IFS_RES:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(IFS = %d response)\n", sendbuf[3]);
            break;
        case SCLIB_T1_BLOCK_S_ABORT_REQ:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(ABORT request)\n");
            break;
        case SCLIB_T1_BLOCK_S_ABORT_RES:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(ABORT response)\n");
            break;
        case SCLIB_T1_BLOCK_S_WTX_REQ:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(WTX request)\n");
            break;
        case SCLIB_T1_BLOCK_S_WTX_RES:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(WTX = %d response)\n", sendbuf[3]);
            break;
        case SCLIB_T1_BLOCK_S_VPP_ERR:
            SCLIB_T1INFO( ">>>>>>>> SENDING S(VPP error)\n");
        }
    }

    _SCLIB_CalcT1Chksum(sendbuf, dev->CardCapabilities.T1.EDC, FALSE);
    if(dev->CardCapabilities.T1.EDC == SCLIB_T1_CRC_CHECK)
        dev->snd_len += 2;
    else
        dev->snd_len++;


    SCLIB_T1INFO("sendBlock: NAD = 0x%x, PCB = 0x%x, LEN = 0x%x, sendBlock.len = %d\n",
                 sendbuf[0], sendbuf[1], sendbuf[2], dev->snd_len);


    /* [EMV 2000] */
    SC_StopTimer(sc, 0);
    SCLIB_Delay();
    SC_StartTimer(sc, 0, SC_TMR_MODE_7, (dev->CardCapabilities.T1.BWT / dev->CardCapabilities.etu) + dev->T1.WTX * (BitRateAdjustment[dev->CardCapabilities.Dl].DNumerator * 2400) + 1);

    dev->errno = SCLIB_SUCCESS;
    dev->snd_pos = 0;
    dev->op_state = SCLIB_OP_WRITE;
    sc->IER |= SC_IER_TBE_IE_Msk;
    while(dev->op_state == SCLIB_OP_WRITE && !(dev->errno));
    sc->IER &= ~SC_IER_TBE_IE_Msk;  //ya

    if (dev->errno != SCLIB_SUCCESS) {
        SCLIB_ERR( "SendBlock - Error!\n");
        return dev->errno;
    }

    retVal = _SCLIB_PrecessT1Response(num);

    if (retVal != SCLIB_SUCCESS)
        SCLIB_ERR( "SendBlock - T1CommandResponse Error! Msg: %d \n", retVal);

    return retVal;
}


/**
  * @brief  Send I-Block to specified interface.
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param buf The pointer of buffer of transmitted data
  * @param len The length of transmitted data
  * @param more More bit
  * @param increaseNs  Send-sequence number control
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int _SCLIB_SendIBlock (uint32_t num, uint8_t *buf, int len, char more, int increaseNs )
{
    uint8_t PCB = 0, LEN;
    SCLIB_DEV_T *dev = &_scDev[num];
    int retVal, i;

    if (increaseNs)
        dev->T1.SSN = (dev->T1.SSN + 1) % 2;

    if (dev->T1.SSN)
        PCB |= 0x40; /* N(s) */

    LEN = len;
    if (more)
        PCB |= 0x20; /* M=1 */
    else
        PCB &= 0xdf; /* M=0 */

    /* prepare the sendBlock */
    dev->snd_buf[0] = 0x00; /* NAD */
    dev->snd_buf[1] = PCB;  /* PCB */
    dev->snd_buf[2] = LEN;  /* LEN */

    /* copy the INF data (if exists) */
    /* INF dat is ready for emulation */
    for (i = 0 ; i < LEN ; ++i)
        dev->snd_buf[3 + i] = buf[i];

    dev->snd_len = len + 3;

    retVal = _SCLIB_SendBlock(num);

    return retVal;
}


/**
  * @brief  Send S-Block to specified interface.
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param  control Protocol control byte of S-Block
  * @param  data The configured data of protocol control byte. Could be \ref SCLIB_T1_BLOCK_S_WTX_REQ or \ref SCLIB_T1_BLOCK_S_IFS_REQ
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int32_t _SCLIB_SendSBlock(uint32_t num, uint8_t control, uint8_t data)
{
    uint8_t PCB = 0, LEN = 0;
    SCLIB_DEV_T *dev = &_scDev[num];
    int retVal;


    PCB = control;
    if (PCB == SCLIB_T1_BLOCK_S_IFS_REQ || PCB == SCLIB_T1_BLOCK_S_IFS_RES || PCB == SCLIB_T1_BLOCK_S_WTX_RES)
        LEN = 1;

    /* prepare the sendBlock */
    dev->snd_buf[0] = 0x00; /* NAD */
    dev->snd_buf[1] = PCB;  /* PCB */
    dev->snd_buf[2] = LEN;  /* LEN */

    /* copy the INF data (if exists) */
    if (LEN)
        dev->snd_buf[3] = data;

    dev->snd_len = LEN + 3;

    retVal = _SCLIB_SendBlock(num);

    return retVal;
}


/**
  * @brief  Send R-Block to specified interface
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in]  nr Send-sequence number
  * @param[in]  error  Error code (0:error-free, 1:redundancy code or parity error, 2:indicate other errors)
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
static int32_t _SCLIB_SendRBlock(uint32_t num, char nr, char error)
{
    int retVal;
    SCLIB_DEV_T *dev = &_scDev[num];

    /* NAD */
    dev->snd_buf[0] = 0x00;
    /* PCB */
    dev->snd_buf[1] = 0x80;
    if (nr)
        dev->snd_buf[1] |= 0x10;
    /* LEN */
    dev->snd_buf[2] = 0x00;

    /* Error code*/
    dev->snd_buf[1] |= (0xF & error);   /* error=0, error-free
                                         * error=1, redundancy code or parity error
                                         * error=2, indicate other errors
                                         */

    dev->snd_len = 3;
    retVal = _SCLIB_SendBlock(num);

    return retVal;
}



/**
  * @brief  Handles S-Block requests from the card
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int32_t _SCLIB_ProcessSBlock(uint32_t num)
{
    int32_t retVal, origBwt;
    SCLIB_DEV_T *dev = &_scDev[num];
    uint8_t *recBlock = dev->rcv_buf, *setvalue;
    uint8_t wtx;
    int32_t pos, cnt;  // for remove IAR warning

    /* if this is WTX, update params including readers */
    if (recBlock[1] == SCLIB_T1_BLOCK_S_WTX_REQ) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(WTX request)\n");
        pos = dev->rcv_pos;
        cnt = dev->rcv_cnt;
        if (!_SCLIB_GetINF(recBlock, pos + cnt))
            return SCLIB_ERR_T1_PROTOCOL;

        /* S-block (WTX request) : length has to be 0x01 && check NAD*/
        if (_SCLIB_GetLEN(recBlock)!=0x01 || _SCLIB_GetNAD(recBlock)!=0x00)
            return SCLIB_ERR_T1_PROTOCOL;

        pos = dev->rcv_pos;
        cnt = dev->rcv_cnt;
        wtx = *(_SCLIB_GetINF(recBlock, pos + cnt));       // get WTX value from card (WTX is ETU-based??????)

        origBwt = dev->CardCapabilities.T1.BWT;
        dev->CardCapabilities.T1.BWT *= wtx;
        dev->T1.WTX = wtx;
        retVal = _SCLIB_SendSBlock(num, SCLIB_T1_BLOCK_S_WTX_RES, wtx);

        /* for CCID Short APDU T=1 exchange */
        SCLIB_RequestTimeExtension(SCLIB_PROTOCOL_T1);

        /* even if an error occurred, we have to set the bwt to its original value */
        dev->CardCapabilities.T1.BWT = origBwt;
        dev->T1.WTX = 1;
        /* the next block has already been received, so we can reset the BWT of the
         reader to its original values. */


        /* check if parity error has happened in communication */
        //if( (retVal&PROROCOL_T1_P_ERR_NOTICE)==PROROCOL_T1_P_ERR_NOTICE )
        //  retVal = SC_ERR_PARITY_ERROR;

        // NOTE: the error of the SBlock is checked only after re-setting the reader params to the default
        if (retVal != SCLIB_SUCCESS) {
            return retVal;
        }

    }
    /* if this if IFS, update params */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_IFS_REQ) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(IFS request)\n");

        pos = dev->rcv_pos;
        cnt = dev->rcv_cnt;
        if (!_SCLIB_GetINF(recBlock, pos + cnt))
            return SCLIB_ERR_T1_PROTOCOL;

        /* IFSC shall have a value in the range '10' to 'FE' && check NAD */
        if ( _SCLIB_GetLEN(recBlock) != 0x01 || _SCLIB_GetNAD(recBlock) != 0x00)
            return SCLIB_ERR_T1_PROTOCOL;
        pos = dev->rcv_pos;
        cnt = dev->rcv_cnt;
        setvalue = _SCLIB_GetINF(recBlock, pos + cnt);
        if(*setvalue < 0x10 || *setvalue > 0xFE)
            return SCLIB_ERR_T1_PROTOCOL;

        pos = dev->rcv_pos;
        cnt = dev->rcv_cnt;
        dev->CardCapabilities.T1.IFSC = *_SCLIB_GetINF(recBlock, pos + cnt) < 254 ? *_SCLIB_GetINF(recBlock, pos + cnt) : 254;

        pos = dev->rcv_pos;
        cnt = dev->rcv_cnt;
        retVal = _SCLIB_SendSBlock(num, SCLIB_T1_BLOCK_S_IFS_RES, *_SCLIB_GetINF(recBlock, pos + cnt));

        if (retVal != SCLIB_SUCCESS) {
            return retVal;
        }
    }
    /* if Abort request -> response + return error code */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_ABORT_REQ) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(ABORT request)\n");

        retVal = _SCLIB_SendSBlock(num, SCLIB_T1_BLOCK_S_ABORT_RES, 0);

        /* EMV: just abort current card session */
        return SCLIB_ERR_T1_ABORT_RECEIVED;
    }
    /* if RESYNCH response -> return error code */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_RESYNCH_RES) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(RESYNCH response)\n");
        return SCLIB_ERR_T1_RESYNCH_RECEIVED;
    }
    /* if WTX response -> return error code */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_WTX_RES) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(WTX response)\n");

        return SCLIB_ERR_T1_WTXRES_RECEIVED;
    }
    /* if IFS response -> return error code */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_IFS_RES) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(IFS response)\n");

        return SCLIB_ERR_T1_IFSRES_RECEIVED;
    }
    /* if Abort response -> return error code */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_ABORT_RES) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(Abort response)\n");

        return SCLIB_ERR_T1_ABORTRES_RECEIVED;
    }
    /* if VPP error -> return error code */
    else if (recBlock[1] == SCLIB_T1_BLOCK_S_VPP_ERR) {

        SCLIB_T1INFO( "<<<<<<<<< RECEIVING S(VPP error)\n");

        return SCLIB_ERR_T1_VPP_ERROR_RECEIVED;
    }
    /* if other unknown command -> return error code */
    else {

        return SCLIB_ERR_T1_PROTOCOL;
    }

    /* Resynch request can only be sent by the interface device */
    return SCLIB_SUCCESS;
}

/* When the situation that two consecutive errors happened, */
/* driver would use the prior error code */
static uint8_t T1SelectErrcode(uint8_t *errcode, uint8_t default_errcode)
{
    if(*errcode == 0) {
        *errcode = default_errcode;
        return default_errcode;
    } else if(*errcode == 1)
        return 1;
    else if(*errcode == 2)
        return 2;
    else
        return 2;
}


/**
  * @brief  Apply the request of T=1 transmission and handle the retry process
  *         and processes S-Block
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in]  cmdBuf Command APDU buffer
  * @param[in]  cmdLen Command APDU length
  * @param[out]  rspBuf Response APDU buffer
  * @param[out]  rspLen Response APDU length
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int32_t _SCLIB_StartT1Transmission(uint32_t num, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rspBuf, uint32_t *rspLen)
{
    uint8_t prior_rsptype, cur_rsptype, errcode=0;
    uint8_t rsp_type, expected_next_I_SN = 0;
    int bytes;
    char nr;
    int retVal, counter, retransmit = 0;
    char more, finished;
    SCLIB_DEV_T *dev = &_scDev[num];
    uint8_t *recBlock = dev->rcv_buf;

    SCLIB_T1INFO(" _SCLIB_StartT1Transmission - Enter\n");

    /* calculate the number of bytes to send  */
    counter = 0;
    bytes = (cmdLen > dev->CardCapabilities.T1.IFSC ? dev->CardCapabilities.T1.IFSC : cmdLen);

    /* see if chaining is needed  */
    more = (cmdLen > dev->CardCapabilities.T1.IFSC);
    finished = 0;

    /*================================
     Send command
    ================================ */

    /* send block of data */
    retVal = _SCLIB_SendIBlock(num, cmdBuf, bytes, more, 1);

    /* calculate expected sequence number for R-block */
    if(dev->T1.IBLOCK_REC == 1)
        expected_next_I_SN = (dev->T1.RSN + 1) % 2;

    while (!finished) {

        /* check if an error occurred */
        if (retVal != SCLIB_SUCCESS) {
            // Check retransmit count and record block type
            retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );

            if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                return SCLIB_ERR_T1_PROTOCOL;
            // Send R-block for communication error
            else if( retVal == SCLIB_ERR_T1_PROTOCOL || retVal == SCLIB_ERR_TIME0OUT) {
                retVal = _SCLIB_SendRBlock(num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                // Process parity and checksum error
            } else if (retVal == SCLIB_ERR_PARITY_ERROR || retVal == SCLIB_ERR_T1_CHECKSUM) {
                retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 1) );       // EDC/parity error
#if 0
                // [2011.11.29]
                /* EMV Certification - parity error has high priority. */
                if(retVal==SCLIB_ERR_PARITY_ERROR || retVal==SCLIB_ERR_T1_CHECKSUM) {

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 1) );       // EDC/parity error

                } else if(_SCLIB_GetBlockType(recBlock)==SCLIB_T1_BLOCK_I) {
                    /* check NAD which has to be 0x00 */
                    if(_SCLIB_GetNAD(recBlock)!=0x00)
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors

                    /* Check CLA if fit for the rule bit[8] must be zero  */
                    /* LEN does NOT be 0xFF */
                    else if(T1BlockCLACheck(recBlock)==0x80 || _SCLIB_GetLEN(recBlock)==0xFF)
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors

                    /* check sequence-number that current SN of I-block needs to be different from SN of previous I-block  */
                    /* If no I-block was previously received, SN of received I-block shall be 0x00 */
                    else if((dev->T1.IBLOCK_REC==1) && (expected_next_I_SN != _SCLIB_GetNS(recBlock)))
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                    // Transmit failed, check NS
                    else if((dev->T1.IBLOCK_REC==0) && (0x00!=_SCLIB_GetNS(recBlock)))
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors

                    else {
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 1) );       // EDC/parity error
                    }
                } else {
                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 1) );       // EDC/parity error
                }
#endif
            } else {
                retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
            }
        } else {
            /* the block received is valid */
            rsp_type = _SCLIB_GetBlockType(recBlock);

            /* if all the data has been sent and the response from the card has arrived -> we are done here */
            if (rsp_type == SCLIB_T1_BLOCK_I) {

                /* check NAD which has to be 0x00 */
                if(_SCLIB_GetNAD(recBlock)!=0x00) {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                }
                /* Check CLA if fit for the rule bit[8] must be zero  */
                /* LEN does NOT be 0xFF */
                // [2011.11.24]
                /* EMV Certification : test tool does not check CLA byte  */
                else if(/*T1BlockCLACheck(recBlock)==0x80 || */_SCLIB_GetLEN(recBlock)==0xFF) {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                }
                /* check sequence-number that current SN of I-block needs to be different from SN of previous I-block  */
                /* If no I-block was previously received, SN of received I-block shall be 0x00 */
                else if((dev->T1.IBLOCK_REC==1) && (expected_next_I_SN!=_SCLIB_GetNS(recBlock))) {

                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                } else if((dev->T1.IBLOCK_REC==0) && (0x00!=_SCLIB_GetNS(recBlock))) {

                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                } else if( more == 0 )
                    finished = 1;

                // [2011.11.29]
                /* EMV Certification : protect machine not to get into loop state */
                // [2011.12.05]
                /* EMV Certification: ICC returns I-block that content has no any errors and is unexpected data */
                /* However, firmware considers this is erroneous block and return erroneous R-block */
                else {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                }

            } else if (rsp_type == SCLIB_T1_BLOCK_R) {
                /* erroneous R-Block received */
                if (!(recBlock[1] & 0x0F) == 0) {   // Error free?
                    SCLIB_T1INFO("<<<<<<<<< RECEIVING R(%d + ERROR)\n", _SCLIB_GetNR(recBlock));

                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    /* check NAD & Length & R-type */
                    if(_SCLIB_GetNAD(recBlock)!=0x00 || _SCLIB_GetLEN(recBlock)!=0x00 || (recBlock[1]&0xE0)!=0x80)
                        retVal = _SCLIB_SendRBlock(num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                    // [2011.11.28] case 1CE.122.01 add "else"
                    /* EMV Certification */
                    else if(_SCLIB_GetNR(recBlock)!=dev->T1.SSN)
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );

                    else if(_SCLIB_GetNR(recBlock)==dev->T1.SSN)
                        retVal = _SCLIB_SendIBlock(num, cmdBuf + counter, bytes, more, 0);

                }
                /* Error-free in error fields */
                /* R-block should not bring length is larger than 0x00 */
                /* Bit[8:6] of R-block should be b'100' */
                /* Check NAD */
                else if (_SCLIB_GetLEN(recBlock)!=0x00 || (recBlock[1]&0xE0)!=0x80 || _SCLIB_GetNAD(recBlock)!=0x00) {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                }
                /* positive ACK R-Block received -> send next I-Block (if there still data left to send) */
                else if (_SCLIB_GetNR(recBlock) != dev->T1.SSN) {

                    SCLIB_T1INFO("<<<<<<<<< RECEIVING R(%d)\n", _SCLIB_GetNR(recBlock));

                    retransmit = 0;

                    if (more) {
                        /* calculate the number of bytes to send */
                        counter += bytes;
                        bytes = ((cmdLen - counter)> dev->CardCapabilities.T1.IFSC ?
                                 dev->CardCapabilities.T1.IFSC : (cmdLen - counter));

                        /* see if chaining is needed  */
                        more = ((cmdLen - counter) > dev->CardCapabilities.T1.IFSC);

                        /* send block of data */
                        retVal = _SCLIB_SendIBlock(num, cmdBuf + counter, bytes, more, 1);
                    } else { /* no more data left to send -> send an R-Block to get acknowledgment */

                        /*
                         * last block of a chain shall be acknowledged by either an I-block if correctly received,
                         * or a R-block if incorrectly received.
                         * This condition does NOT fit the rules.
                         */
                        retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                    }
                }
                /* retransmit the current block */
                else {
                    SCLIB_T1INFO("<<<<<<<<< RECEIVING R(%d)\n", _SCLIB_GetNR(recBlock));

                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendIBlock(num, cmdBuf + counter, bytes, more, 0);
                }
            }
            /* process the S-Block request received */
            else if (rsp_type == SCLIB_T1_BLOCK_S) {
                retVal = _SCLIB_ProcessSBlock(num);
                if(retVal==SCLIB_ERR_T1_PROTOCOL)
                    retransmit = 0;

                /* Do NOT respond to that block was S(Response) block or other errors */
                if (retVal == SCLIB_ERR_T1_RESYNCH_RECEIVED ||
                        retVal == SCLIB_ERR_T1_WTXRES_RECEIVED ||
                        retVal == SCLIB_ERR_T1_IFSRES_RECEIVED ||
                        retVal == SCLIB_ERR_T1_ABORTRES_RECEIVED ||
                        retVal == SCLIB_ERR_T1_PROTOCOL) {

                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, expected_next_I_SN, T1SelectErrcode(&errcode, 2) );       // other errors
                }

                /* (scenario 20) */
                if (retVal == SCLIB_ERR_T1_ABORT_RECEIVED || retVal == SCLIB_ERR_T1_VPP_ERROR_RECEIVED) {
                    return retVal;
                }

            }
        }
    }

    /*===================================
     Receive response
    =================================== */

    counter = 0;
    more = 1;
    retransmit = 0;

    nr = _SCLIB_GetNS(recBlock);

    while (more) {

        /* check if an error occurred */
        if (retVal != SCLIB_SUCCESS) {
            retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
            if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                return SCLIB_ERR_T1_PROTOCOL;

            retVal = _SCLIB_SendRBlock( num, nr, 0);       // error-free
        } else {
            /* the block received is valid */
            rsp_type = _SCLIB_GetBlockType(recBlock);

            if (rsp_type == SCLIB_T1_BLOCK_I) {
                SCLIB_T1INFO("<<<<<<<<< RECEIVING I(%d,%d)\n", _SCLIB_GetNS(recBlock), _SCLIB_GetMBit(recBlock));

                retransmit = 0;

                if (nr != _SCLIB_GetNS(recBlock)) {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, nr, 0 );      // error-free
                    continue;
                }

                // [2011.11.24]
                /* EMV Certification: add */
                /* check NAD which has to be 0x00 */
                if(_SCLIB_GetNAD(recBlock)!=0x00) {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, nr, 0 );      // other errors
                    continue;   // [2011.11.24 night]
                }
                /* Check CLA if fit for the rule bit[8] must be zero  */
                /* LEN does NOT be 0xFF */
                /* EMV Certification : Do not check CLA byte  */
                else if(_SCLIB_GetLEN(recBlock)==0xFF) {
                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, nr, 0 );      // error-free
                    continue;   // [2011.11.24 night]
                }

                dev->T1.RSN = _SCLIB_GetNS(recBlock);
                dev->T1.IBLOCK_REC = 1;

                /* calculate nr */
                nr = (_SCLIB_GetNS(recBlock) + 1) % 2;

                /* save inf field */
                bytes = _SCLIB_GetLEN(recBlock);
                if (bytes)    // all data store in outbuf at one transmission
                    memcpy(rspBuf + counter, &(recBlock[3]), bytes);
                counter += bytes;
                *rspLen = counter;

                /* see if chaining is requested */
                more = _SCLIB_GetMBit(recBlock);

                if (more) {
                    retVal = _SCLIB_SendRBlock( num, nr, 0 );
                }
            }
            /* retransmit the current R-Block */
            else if ( rsp_type == SCLIB_T1_BLOCK_R) {

                SCLIB_T1INFO("<<<<<<<<< RECEIVING R(%d)\n", _SCLIB_GetNR(recBlock));

                retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                if (retransmit == 3)
                    return SCLIB_ERR_T1_PROTOCOL;

                retVal = _SCLIB_SendRBlock( num, nr, 0);       // error-free
            }
            /* process the S-Block request received */
            else if (rsp_type == SCLIB_T1_BLOCK_S) {
                retVal = _SCLIB_ProcessSBlock(num);
                if(retVal==SCLIB_SUCCESS)
                    retransmit = 0;

                /* Do NOT respond to that block was S(Response) block or other errors */
                if (retVal == SCLIB_ERR_T1_RESYNCH_RECEIVED ||
                        retVal == SCLIB_ERR_T1_WTXRES_RECEIVED ||
                        retVal == SCLIB_ERR_T1_IFSRES_RECEIVED ||
                        retVal == SCLIB_ERR_T1_ABORTRES_RECEIVED ||
                        retVal == SCLIB_ERR_T1_PROTOCOL) {

                    retransmit = T1BlockRetransmitCheck(recBlock, retransmit, &prior_rsptype, &cur_rsptype );
                    if (retransmit == SCLIB_MAX_T1_RETRANSMIT_CNT)
                        return SCLIB_ERR_T1_PROTOCOL;

                    retVal = _SCLIB_SendRBlock( num, nr, 0 );      // error-free
                }

                /* (scenario 20) */
                if (retVal == SCLIB_ERR_T1_ABORT_RECEIVED || retVal == SCLIB_ERR_T1_VPP_ERROR_RECEIVED) {
                    return retVal;
                }

            }
        }
    }


    SCLIB_T1INFO(" _SCLIB_StartT1Transmission - Exit\n");

    return retVal;
}



/**
  * @brief Send a command and get response from smartcard using T=1 protocol
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in]  cmdBuf Command APDU buffer
  * @param[in]  cmdLen Command APDU length
  * @param[out]  rspBuf Response APDU buffer
  * @param[out]  rspLen Response APDU length
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int32_t _SCLIB_T1Transmit(uint32_t num, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rspBuf, uint32_t *rspLen )
{
    int retVal, retransmitTimes = 0, i;
    SCLIB_DEV_T *dev = &_scDev[num];
    uint8_t *recBlock = &g_RxBUF[0];

    memset(&g_RxBUF[0], 0x00, sizeof(g_RxBUF));

    dev->snd_buf = &g_TxBUF[0];
    dev->rcv_buf = &g_RxBUF[0];

    retVal = _SCLIB_StartT1Transmission(num, cmdBuf, cmdLen, rspBuf, rspLen );

    while (retVal != SCLIB_SUCCESS && retransmitTimes < SCLIB_MAX_T1_RETRANSMIT_CNT) {
        /* if the command failed, check the reason and try to recover */
        if (retVal != SCLIB_SUCCESS) {

            if(dev->openflag == 0)
                return SCLIB_ERR_CARD_REMOVED;

            if (retVal == SCLIB_ERR_T1_VPP_ERROR_RECEIVED) {
                SCLIB_ERR("T1Command - Error! T1_VPP_ERROR_RECEIVED\n");

                return retVal; /* fatal error */
            }

            else if (retVal == SCLIB_ERR_T1_ABORT_RECEIVED) {
                SCLIB_ERR("T1Command - Error! T1_ABORT_RECEIVED - trying again\n");

                return retVal; /* fatal error by EMV */
            } else {

                SCLIB_ERR("T1Command - Error! Msg : %d \n", retVal);

                /* general error -> try sending a RESYNCH request up to 3 times */
                for (i = 0 ; (i < SCLIB_MAX_T1_RETRANSMIT_CNT) && (retVal != SCLIB_SUCCESS) ; ++i) {
                    retVal = _SCLIB_SendSBlock(num, SCLIB_T1_BLOCK_S_RESYNCH_REQ, 0);
                    if ((retVal == SCLIB_SUCCESS) && _SCLIB_GetBlockType(recBlock) == SCLIB_T1_BLOCK_S) {
                        /* success - check if it's S(RESYNCH response) */
                        retVal = _SCLIB_ProcessSBlock(num);
                        if (retVal == SCLIB_ERR_T1_RESYNCH_RECEIVED)
                            retVal = SCLIB_SUCCESS;
                    } else
                        retVal = SCLIB_ERR_T1_PROTOCOL; /* so that we could send RESYNCH req. again */
                }

                if (retVal != SCLIB_SUCCESS) {
                    /* RESYNCH failed 3 times */
                    SCLIB_ERR("T1Command - Error! could not recover - resetting the card\n");
                    // TODO: warm reset or deactivation...
                    SCLIB_Deactivate(num);

                    return SCLIB_ERR_T1_PROTOCOL;
                } else {
                    /* RESYNCH succeeded -> init protocol + retry command */
                    /* init the protocol */

                    /* reset send-sequence & more-data bit */
                    dev->T1.SSN = 1;    // default value
                    dev->T1.RSN = 0;    // default value
                    dev->T1.IBLOCK_REC = 0; // default value

                    /* try to transmit the command again */
                    retVal = _SCLIB_StartT1Transmission(num, cmdBuf, cmdLen, rspBuf, rspLen );
                    retransmitTimes++;
                }
            }
        }
    }


    return SCLIB_SUCCESS;
}

/*@}*/ /* end of group NUC400_SCLIB_PRIVATE_FUNCTIONS */

/*@}*/ /* end of group NUC400_SC_Library */

/*@}*/ /* end of group NUC400_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/




