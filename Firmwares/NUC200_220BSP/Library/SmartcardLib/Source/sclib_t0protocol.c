/**************************************************************************//**
 * @file     sclib_t0protocol.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/05/18 10:38a $
 * @brief    This file handles T=0 protocol
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sclib_int.h"

/** @addtogroup NUC200_Library NUC200 Library
  @{
*/

/** @addtogroup NUC200_SCLIB Smartcard Library
  @{
*/

/** @addtogroup NUC200_SCLIB_PRIVATE_FUNCTIONS Smartcard Library Private Functions
  @{
*/

extern uint8_t g_RxBUF[SCLIB_MAX_T1_BUFFER_SIZE];
extern uint8_t g_TxBUF[SCLIB_MAX_T1_BUFFER_SIZE];

/**
  * @brief  Prepare Command TPDU according to APDU
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in] buf APDU command buffer
  * @param[in] len Length of APDU command buffer
  * @retrun Successfully build TPDU or not
  * @retval SCLIB_SUCCESS Successfully build TPDU
  * @retval SCLIB_ERR_T0_PROTOCOL APDU send in does not comply with ISO 7816-3
  * @note This API does not support ISO 7816-3 case 2E, 3E, 4E
  */
int32_t _SCLIB_BuildT0TPDU(uint32_t num, uint8_t *buf, uint32_t *len)
{

    SCLIB_DEV_T *dev = &_scDev[num];
    uint32_t reqLen = *len;
    uint8_t *reqBuf = buf;

    if (reqLen < 4) {
        // A T=0 request needs at least 4 bytes (case 1)
        SCLIB_ERR("!SmartcardT0Request: APDU is too short (%d)\n", ioRequestDataLength);
        return SCLIB_ERR_T0_PROTOCOL;

    } else {

        if (reqLen <= 5) {
            // We request to read data from the card
            dev->T0.Lc = 0;

            if (reqLen == 4) {
                // Case 1 ISO 7816-3 12.2.2
                // This case requires that we append a 0 to the APDU to make it a TPDU

                dev->T0.Le = 0;
            } else {
                // Case 2S ISO 7816-3 12.2.3
                // outgoing data transfer: P3='00' introduces a 256-byte data transfer from the card
                dev->T0.Le = reqBuf[4] ? reqBuf[4] : 256;
            }

        } else {
            // Case 3S ISO 7816-3 12.2.4
            dev->T0.Lc = reqBuf[4];
            dev->T0.Le = 0;

            // 5 = CLA + INS + P1 + P2 + Lc
            if(dev->T0.Lc != reqLen - 5) { // this situation has two results: case3 failed or apply case4
                if(dev->T0.Lc != 0) {   // maybe Lc is zero, which is plus one byte, for APDU test card
                    if((reqLen-5-dev->T0.Lc) != 1) {
                        SCLIB_ERR("!SmartcardT0Request: Lc(%d) in TPDU doesn't match number of bytes to send(%d).\n",
                                  dev->T0.Lc,
                                  ioRequestDataLength - 5
                                 );
                        return SCLIB_ERR_T0_PROTOCOL;
                    }
                }

                if((reqLen - 5 - dev->T0.Lc) == 1) {
                    if(reqBuf[reqLen - 1] == 0x00) {
                        /* if Le byte is equal to 0x00, driver will adjust the length of data bytes */
                        *len -= 1;
                    }
                    // Case 4S ISO 7816-3 12.2.5
                    dev->T0.Le = reqBuf[reqLen - 1];
                }
            }
        }
    }

    return SCLIB_SUCCESS;
}


/**
  * @brief Send a command and get response from smartcard using T=0 protocol
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in]  cmdBuf Command APDU buffer
  * @param[in]  cmdLen Command APDU length
  * @param[out]  rspBuf Response APDU buffer
  * @param[out]  rspLen Response APDU length
  * @retrun The result status of process.
  * @retval SCLIB_SUCCESS Success
  * @retval Others Failed.
  */
int32_t _SCLIB_T0Transmit(uint32_t num, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rspBuf, uint32_t *rspLen)
{
    int status = SCLIB_SUCCESS;
    int32_t local_count, idx;
    uint8_t INS = cmdBuf[1], Le;
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];


    dev->rcv_buf = &g_RxBUF[0];
    dev->rcv_pos = 0;
    //  let the lib setup the T=0 TPDU & check for errors
    status = _SCLIB_BuildT0TPDU(num, cmdBuf, &cmdLen);

    SCLIB_INFO("Command Bytes: ");

    for(idx = 0; idx < cmdLen; idx++)
        SCLIB_INFO("%02X ", cmdBuf[idx]);
    SCLIB_INFO("\n");

    // store the Le value
    Le = dev->T0.Le;

    //  timer select
    // T=0 just use 24-bit Timer for waiting time check

    if( status == SCLIB_SUCCESS ) {
        // WWT
        /* EMV2000 */
        SC_StartTimer(sc, 0, SC_TMR_MODE_7,
                      (dev->CardCapabilities.T0.WT / dev->CardCapabilities.etu) + (480 * BitRateAdjustment[dev->CardCapabilities.Dl].DNumerator) + 1);

        // 5 = CLA + INS + P1 + P2 + Lc
        local_count = 5;
        dev->errno = 0;
        dev->snd_buf = cmdBuf;
        dev->snd_len = local_count;
        dev->op_state = SCLIB_OP_WRITE;
        sc->IER |= SC_IER_TBE_IE_Msk;
        while(dev->op_state == SCLIB_OP_WRITE && !(dev->errno) && dev->openflag == 1) ;
        sc->IER &= ~SC_IER_TBE_IE_Msk;  // need to disable tx interrupt anyway. we're not sending data now... ya
        if(dev->errno != SCLIB_SUCCESS) {

            SCLIB_ERR("Failed to Write Command header & body..Error Msg:%d \n", dev->errno);
            return dev->errno;
        }

        do {
            /* get procedure byte */
            do {

                dev->errno = 0;
                dev->rcv_len = 1;
                dev->rcv_cnt = 0;
                dev->op_state = SCLIB_OP_READ;
                while(dev->op_state == SCLIB_OP_READ && !(dev->errno) && dev->openflag == 1) ;
                if(dev->errno != SCLIB_SUCCESS) {
                    if(dev->EMV == TRUE && dev->errno == SCLIB_ERR_READ)
                        SCLIB_Deactivate(num);
                    SCLIB_ERR("Failed to Read Procedure byte..Error Msg:%d \n", dev->errno);
                    return dev->errno;
                }

                /* for CCID TPDU T=0 exchange */
                if(dev->rcv_buf[dev->rcv_pos] == 0x60)
                    SCLIB_RequestTimeExtension(SCLIB_PROTOCOL_T0);

            } while(dev->rcv_buf[dev->rcv_pos] == 0x60);    // loop here, waiting for procedure byte SW1 SW2


            if(dev->rcv_buf[dev->rcv_pos] == (INS ^ 0x01) || dev->rcv_buf[dev->rcv_pos] == INS) {
                /* process all remaining data  */
                if(dev->T0.Lc != 0)
                    local_count = dev->T0.Lc;
                else
                    local_count = dev->T0.Le;

            } else if((INS ^ 0xfe) == dev->rcv_buf[dev->rcv_pos] || (INS ^ 0xff) == dev->rcv_buf[dev->rcv_pos]) {
                /* next one byte at a time to send if it exists */
                local_count = 1;

            } else if( (dev->rcv_buf[dev->rcv_pos]&0xf0)==0x60 || (dev->rcv_buf[dev->rcv_pos]&0xf0)==0x90) {

                dev->errno = 0;
                dev->rcv_len = 1;
                // offset plus 1 for storing SW2
                dev->rcv_pos++;
                dev->rcv_cnt = 0;
                dev->op_state = SCLIB_OP_READ;

                while(dev->op_state == SCLIB_OP_READ && !(dev->errno) && dev->openflag == 1) ;
                if(dev->errno != SCLIB_SUCCESS) {
                    SCLIB_ERR("Failed to Read SW2..Error Msg:%d \n", dev->errno);
                    return dev->errno;
                }

                SC_StopTimer(sc, 0);

                dev->rcv_pos++; /* save SW2 */

                /* Show the response data */
                SCLIB_INFO("Date Length is %d \n", dev->rcv_pos);
                SCLIB_INFO("Got Data are ");
                if(dev->rcv_pos > 2)
                    for(idx=0; idx<Le; idx++) {
                        SCLIB_INFO("%02x  ", dev->rcv_buf[idx]);
                        *rspBuf++ = dev->rcv_buf[idx];
                    }

                SCLIB_INFO("\n");
                SCLIB_INFO("SW12 ");
                if(dev->rcv_pos > 2)
                    for(idx=Le; idx<dev->rcv_pos; idx++) {
                        SCLIB_INFO("%02X  ", dev->rcv_buf[idx]);
                        *rspBuf++ = dev->rcv_buf[idx];
                    }
                else
                    for(idx=0; idx<dev->rcv_pos; idx++) {
                        SCLIB_INFO("%02X  ", dev->rcv_buf[idx]);
                        *rspBuf++ = dev->rcv_buf[idx];
                    }
                SCLIB_INFO("\n");

                *rspLen = dev->rcv_pos;

                return SCLIB_SUCCESS;
            } else {
                SCLIB_INFO("Unknown procedure byte %02x\n", dev->rcv_buf[0]);
                return SCLIB_ERR_T0_PROTOCOL;
            }

            /* read/write card according lc, le and local_count */
            if(dev->T0.Lc != 0) {

                // keep snd_buf in user application, save some memcpy calls
                dev->snd_buf = cmdBuf + cmdLen - dev->T0.Lc;
                dev->snd_len = local_count;
                dev->snd_pos = 0;
                dev->op_state = SCLIB_OP_WRITE;
                sc->IER |= SC_IER_TBE_IE_Msk;
                while(dev->op_state == SCLIB_OP_WRITE && !(dev->errno) && dev->openflag == 1);
                sc->IER &= ~SC_IER_TBE_IE_Msk;  //ya
                if(dev->errno != SCLIB_SUCCESS) {
                    if(dev->EMV == TRUE && dev->errno == SCLIB_ERR_WRITE)
                        SCLIB_Deactivate(num);
                    SCLIB_ERR("Failed to Write Lc Data bytes..Error Msg:%d \n", dev->errno);
                    return dev->errno;
                }

                dev->T0.Lc -= local_count;  // when process finished, we check Lc if equal to zero
            } else if (local_count != 0) { // check local_count!=0 for APDU test card
                int32_t i;

                dev->errno = 0;
                dev->rcv_len = local_count;
                dev->rcv_cnt = 0;
                dev->op_state = SCLIB_OP_READ;
                while(dev->op_state == SCLIB_OP_READ && !(dev->errno) && dev->openflag == 1) ;
                if(dev->errno != SCLIB_SUCCESS) {
                    SCLIB_ERR("Failed to Read Response Data..Error Msg:%d \n", dev->errno);
                    return dev->errno;
                }



                dev->T0.Le -= local_count;  // when process finished, we check Le if equal to zero
                i = dev->rcv_cnt; // to remove IAR warning...
                dev->rcv_pos += i;
            }

            if(dev->openflag != 1)
                return SCLIB_ERR_CARD_REMOVED;

        } while(1);

    }

    return(status);
}

/*@}*/ /* end of group NUC200_SCLIB_PRIVATE_FUNCTIONS */

/*@}*/ /* end of group NUC200_SC_Library */

/*@}*/ /* end of group NUC200_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
