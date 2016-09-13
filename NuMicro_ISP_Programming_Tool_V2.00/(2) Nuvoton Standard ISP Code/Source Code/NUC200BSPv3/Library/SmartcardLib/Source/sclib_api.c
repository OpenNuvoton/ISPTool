/**************************************************************************//**
 * @file     smartcard_api.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/05/18 10:16a $
 * @brief    This file provides smartcard library APIs for user application
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sclib_int.h"

// NOTE: Doxygen stuffs are defined in sclib.h since we're not releasing source code of SCLIB

SC_T *_scBase[SC_INTERFACE_NUM] = {SC0, SC1, SC2};
uint32_t vccDelay[SC_INTERFACE_NUM] = {0, 0, 0};
SCLIB_DEV_T _scDev[SC_INTERFACE_NUM];

extern uint32_t atr_len, atr_remain;
extern int32_t ifbyte_flag, tck, ifcount;
extern int32_t atr_time;
extern int32_t atr_check_time;
extern int32_t atr_total_time_start_flag;
extern int32_t atr_final_chk;

extern uint8_t g_RxBUF[SCLIB_MAX_T1_BUFFER_SIZE];
extern uint8_t g_TxBUF[SCLIB_MAX_T1_BUFFER_SIZE];

int32_t SCLIB_Activate(uint32_t num, uint32_t u32EMVCheck)
{
    SC_T *sc = _scBase[num];
    SCLIB_DEV_T *dev = &_scDev[num];
    int32_t retval, clk;
    uint32_t reg = sc->IER;


    clk = _SCLIB_GetInterfaceClock(num);
    if(clk < 1000 || clk > 5000)
        return SCLIB_ERR_CLOCK;

    sc->IER &= ~SC_IER_CD_IE_Msk;

    if(SC_IsCardInserted(sc) == FALSE) {
        sc->IER = reg;
        return SCLIB_ERR_CARD_REMOVED;
    } else  // to set this variable safely, we must disable interrupt here to prevent contention
        dev->bCardRemoved = SCLIB_CARD_PRESENT;

    dev->EMV = u32EMVCheck;
    sc->IER = reg;
    SC_ResetReader(sc);

    retval = SCLIB_ColdReset(num);

    if(retval == SCLIB_ERR_ATR_INVALID_PARAM)
        retval = SCLIB_WarmReset(num);

    dev->snd_buf = &g_TxBUF[0];
    dev->rcv_buf = &g_RxBUF[0];

    return(retval);
}


int32_t SCLIB_ActivateDelay(uint32_t num, uint32_t u32EMVCheck, uint32_t u32Delay)
{
    if(num < SC_INTERFACE_NUM)
        vccDelay[num] = u32Delay;
    return SCLIB_Activate(num, u32EMVCheck);
}

int32_t SCLIB_ColdReset(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];

    dev->errno = _SCLIB_ResetCard(num, SCLIB_COLDRESET);

    if(dev->errno != SCLIB_SUCCESS)
        return (dev->errno);

    dev->pps_complete = 0;
    dev->errno = _SCLIB_UpdateCardCapabilities(num);

    if(dev->errno != SCLIB_SUCCESS)
        return (dev->errno);

    dev->errno = _SCLIB_ParseATR(num, SCLIB_COLDRESET);

    if(dev->errno != SCLIB_SUCCESS)
        SCLIB_ERR("Cold Reset Parse ATR..Error :%d \n", dev->errno);

    if(dev->errno == SCLIB_SUCCESS)
        dev->openflag = 1;

    return (dev->errno);
}


int32_t SCLIB_WarmReset(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];

    dev->errno = _SCLIB_ResetCard(num, SCLIB_WARMRESET);

    if(dev->errno != SCLIB_SUCCESS)
        return (dev->errno);

    dev->pps_complete = 0;
    dev->errno = _SCLIB_UpdateCardCapabilities(num);    //_SCLIB_ParseATR might call this after PPS exchange executed.

    if(dev->errno != SCLIB_SUCCESS)
        return (dev->errno);

    dev->errno = _SCLIB_ParseATR(num, SCLIB_WARMRESET);

    if(dev->errno != SCLIB_SUCCESS)
        SCLIB_ERR("Warm Reset Parse ATR..Error :%d \n", dev->errno);

    return (dev->errno);
}


void SCLIB_Deactivate(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

#if 1    
    // Wait for Tx & Rx get into idle mode
    while(sc->TRSR & SC_TRSR_TX_ATV_Msk) ;
    while(sc->TRSR & SC_TRSR_RX_ATV_Msk) ;
    // Reset both Tx and RX FIFO
    SC_ClearFIFO(sc);
    // Reset low
    SC_SET_RST_PIN(sc, SC_PIN_STATE_LOW);
    //while(sc->PINCSR & SC_PINCSR_SC_RST_Msk) ;  // wait HW reset pin really becomes LOW
    // Clock off
    SC_SET_CLK_PIN(sc, SC_CLK_OFF);
    // I/O low
    SC_SET_IO_PIN(sc, SC_PIN_STATE_LOW);
    //while(sc->PINCSR & SC_PINCSR_SC_DATA_O_Msk) ;       // wait HW I/O pin really becomes LOW
    // VCC low
    SC_SET_VCC_PIN(sc, SC_PIN_STATE_LOW);
#else
    sc->ISR |= SC_ISR_INIT_IS_Msk;
    sc->ALTCTL |= SC_ALTCTL_DACT_EN_Msk;
    while((sc->ISR & SC_ISR_INIT_IS_Msk) != SC_ISR_INIT_IS_Msk);
    sc->ISR |= SC_ISR_INIT_IS_Msk;
#endif

    SC_StopAllTimer(sc);

    dev->op_state = SCLIB_OP_IDLE;
    dev->openflag = 0;

}


int32_t SCLIB_GetCardInfo(uint32_t num, SCLIB_CARD_INFO_T *s_info)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    if(SC_IsCardInserted(sc) == FALSE)
        return SCLIB_ERR_CARD_REMOVED;

    if(dev->openflag == 0)          // Not activate yet.
        return SCLIB_ERR_DEACTIVE;

    s_info->T = dev->CardCapabilities.Protocol.Selected;
    s_info->ATR_Len = dev->CardCapabilities.ATR.Length;
    memcpy(s_info->ATR_Buf, dev->CardCapabilities.ATR.Buffer, dev->CardCapabilities.ATR.Length);
    // Fill reset of the space with 0x00
    memset(s_info->ATR_Buf + dev->CardCapabilities.ATR.Length, 0, SCLIB_MAX_ATR_LEN - dev->CardCapabilities.ATR.Length);

    return SCLIB_SUCCESS;
}


int32_t SCLIB_GetCardAttrib(uint32_t num, SCLIB_CARD_ATTRIB_T *s_attrib)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    if(SC_IsCardInserted(sc) == FALSE)
        return SCLIB_ERR_CARD_REMOVED;

    if(dev->openflag == 0)          // Not activate yet.
        return SCLIB_ERR_DEACTIVE;

    s_attrib->Fi = dev->CardCapabilities.Fl;
    s_attrib->Di = dev->CardCapabilities.Dl;
    s_attrib->conv = sc->CTL & SC_CTL_CON_SEL_Msk ? 1 : 0;
    s_attrib->chksum = dev->CardCapabilities.T1.EDC;
    s_attrib->GT = dev->CardCapabilities.GT;
    s_attrib->WI = dev->CardCapabilities.T0.WI;
    s_attrib->BWI = dev->CardCapabilities.T1.BWI;
    s_attrib->CWI = dev->CardCapabilities.T1.CWI;
    s_attrib->clkStop = 0x00;   // Clock stop not support
    s_attrib->IFSC = dev->CardCapabilities.T1.IFSC;
    s_attrib->NAD = 0x00;   // Always 0x00

    return SCLIB_SUCCESS;
}


int32_t SCLIB_StartTransmission(uint32_t num, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rspBuf, uint32_t *rspLen)
{
    int32_t retval = SCLIB_SUCCESS;
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    if(SC_IsCardInserted(sc) == FALSE)
        return SCLIB_ERR_CARD_REMOVED;

    if(dev->openflag == 0)          // Try to access smartcard before activation
        return SCLIB_ERR_DEACTIVE;

    if(dev->op_state)               // Card busy, previous transmission not complete
        return SCLIB_ERR_CARDBUSY;

    // Give up data remain in dev->buf, if anything remains...
    dev->snd_pos = 0;
    dev->rcv_pos = 0;

    if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T0) {
        retval = _SCLIB_T0Transmit(num, cmdBuf, cmdLen, rspBuf, rspLen);
    } else if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T1) {
        retval = _SCLIB_T1Transmit(num, cmdBuf, cmdLen, rspBuf, rspLen);
    }

    SC_StopAllTimer(sc);    // Stop all timers here, the transmission is complete no matter success or not.

    return retval;
}


int32_t SCLIB_SetIFSD(uint32_t num, uint8_t size)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    uint32_t retransmit = 0;
    int32_t retval;
    uint8_t rsp_type;

    do {
        retval = _SCLIB_SendSBlock(num, SCLIB_T1_BLOCK_S_IFS_REQ, size);
        rsp_type = _SCLIB_GetBlockType(dev->rcv_buf);
        if(rsp_type == SCLIB_T1_BLOCK_S) {
            /* check IFS RESPONSE */
            if(dev->rcv_buf[1] != SCLIB_T1_BLOCK_S_IFS_RES)
                retval = SCLIB_ERR_T1_PROTOCOL;
            // EMV 9.2.4.3 step 1&2, ISO-7816 11.6.2.3 Rule 4
            if(_SCLIB_GetLEN(dev->rcv_buf) != 0x01 || _SCLIB_GetNAD(dev->rcv_buf) != 0x00 || dev->rcv_buf[3] != size)
                retval = SCLIB_ERR_T1_PROTOCOL;
        }
    } while(retransmit++ < SCLIB_MAX_T1_RETRANSMIT_CNT || retval != SCLIB_SUCCESS || rsp_type != SCLIB_T1_BLOCK_S);

    return retval;
}


__weak void SCLIB_RequestTimeExtension(uint32_t u32Protocol)
{
    SCLIB_INFO("ICC requests a time extension for T=%d\n", u32Protocol == SCLIB_PROTOCOL_T0 ? 0 : 1);
}


uint32_t SCLIB_CheckCDEvent(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];
    uint32_t reg, retval = 0;

    if(sc->ISR & SC_ISR_CD_IS_Msk) {      /* toggle detect by card */
        SCLIB_INFO("SC%d: Card Present Int : SC_ISR : [%02x]\n", num, sc->ISR);
        sc->ISR = SC_ISR_CD_IS_Msk; // clear CD_IS bit
        reg = sc->PINCSR;

        if((reg & (SC_PINCSR_CD_REM_F_Msk | SC_PINCSR_CD_INS_F_Msk)) ==
                (SC_PINCSR_CD_REM_F_Msk | SC_PINCSR_CD_INS_F_Msk)) {  // CD_INS_F & CD_REM_F both trigger
            SCLIB_INFO("SC%d: Card status not sure\n", num);
            // [2011.11.30]
            /* protect writing operation forever */
            sc->IER &= ~SC_IER_TBE_IE_Msk;
            sc->PINCSR |= (SC_PINCSR_CD_REM_F_Msk | SC_PINCSR_CD_INS_F_Msk);

            dev->bCardRemoved = SCLIB_CARD_UNKNOWN;
            dev->errno = SCLIB_ERR_CARD_REMOVED;
            dev->op_state = SCLIB_OP_IDLE;
            if(dev->openflag) {
                //SC_DeactivationCmd(sc);   //  Already deactivate by interface if card remove flag is set
                SC_ClearFIFO(sc);           // clear FIFO
                SC_StopAllTimer(sc);
                dev->errno = SCLIB_ERR_CARD_REMOVED;
                dev->op_state = SCLIB_OP_IDLE;
                dev->openflag = 0;
                dev->CardCapabilities.ATR.Length = 0;

            }
        } else if((reg & SC_PINCSR_CD_INS_F_Msk) &&
                (((reg & SC_PINCSR_CD_PIN_ST_Msk) >> SC_PINCSR_CD_PIN_ST_Pos) == ((reg & SC_PINCSR_CD_LEV_Msk) >> SC_PINCSR_CD_LEV_Pos))) {
            sc->PINCSR |= SC_PINCSR_CD_INS_F_Msk;   // clear CD_INS_F flag
            SCLIB_INFO("SC%d: Card Inserted\n", num);

            dev->bCardRemoved = SCLIB_CARD_PRESENT;

        } else if((reg & SC_PINCSR_CD_REM_F_Msk) &&
                (((reg & SC_PINCSR_CD_PIN_ST_Msk) >> SC_PINCSR_CD_PIN_ST_Pos) != ((reg & SC_PINCSR_CD_LEV_Msk) >> SC_PINCSR_CD_LEV_Pos))) {
            sc->PINCSR |= SC_PINCSR_CD_REM_F_Msk;   // clear CD_REM_F flag
            dev->bCardRemoved = SCLIB_CARD_ABSENT;
            SCLIB_INFO("SC%d: Card Removed\n", num);

            // [2011.11.30]
            /* protect writing operation forever */
            sc->IER &= ~SC_IER_TBE_IE_Msk;
            // Already deactivate by interface if card remove flag is set
            SC_ClearFIFO(sc);           // clear FIFO
            SC_StopAllTimer(sc);
            dev->errno = SCLIB_ERR_CARD_REMOVED;
            dev->op_state = SCLIB_OP_IDLE;
            dev->openflag = 0;
            dev->CardCapabilities.ATR.Length = 0;


        } else {
            dev->bCardRemoved = SCLIB_CARD_ABSENT;
            dev->errno = SCLIB_ERR_CARD_REMOVED;
            SCLIB_ERR("SC%d: Card status is not sure and HW has something wrong...\n", num);
        }
        retval = 1;
    }

    return retval;
}

uint32_t SCLIB_CheckTimeOutEvent(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];
    uint32_t retval = 0;

    /* Timer0-2 time out */

    if((sc->ISR & SC_ISR_TMR0_IS_Msk) && (sc->IER & SC_IER_TMR0_IE_Msk)) {
        sc->ISR = SC_ISR_TMR0_IS_Msk;   // Clear interrupt flag
        if(atr_final_chk == 1)
            atr_final_chk = 0;
        else
            dev->errno = SCLIB_ERR_TIME0OUT;
        dev->op_state = SCLIB_OP_IDLE;
        SCLIB_INFO("SC%d: Timer0 time-out\n", num);
        retval = 1;
    }

    if((sc->ISR & SC_ISR_TMR1_IS_Msk) && (sc->IER & SC_IER_TMR1_IE_Msk)) {
        sc->ISR = SC_ISR_TMR1_IS_Msk;
        dev->errno = SCLIB_ERR_TIME1OUT;
        dev->op_state = SCLIB_OP_IDLE;
        SCLIB_INFO("SC%d: Timer1 time-out\n", num);
        retval = 1;
    }

    if((sc->ISR & SC_ISR_TMR2_IS_Msk) && (sc->IER & SC_IER_TMR2_IE_Msk)) {
        sc->ISR = SC_ISR_TMR2_IS_Msk;

        if(atr_check_time == 0) {
            SC_StopTimer(sc, 2);
            dev->errno = SCLIB_ERR_TIME2OUT;
        } else {
            if((atr_time - 256) > 0)
                atr_time -= 256;
            else
                atr_check_time = 0;

        }
        SCLIB_INFO("SC%d: Timer2 time-out\n", num);
        retval = 1;
    }
    return retval;
}

uint32_t SCLIB_CheckTxRxEvent(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];
    uint32_t retval = 0, i;
    uint8_t *atrbuffer = dev->CardCapabilities.ATR.Buffer;

    /* transmit buffer empty interrupt */
    if(sc->ISR & SC_ISR_TBE_IS_Msk) {
        retval = 1;
        if(dev->op_state == SCLIB_OP_WRITE) {
            // We can push 4 bytes into FIFO at most due to FIFO depth limitation
            for(i = 0; i < 4; i++) {
                SC_WRITE(sc, dev->snd_buf[dev->snd_pos++]);
                if(dev->snd_pos == dev->snd_len) {

                    sc->IER &= ~SC_IER_TBE_IE_Msk;
                    dev->op_state = SCLIB_OP_IDLE;
                    break;
                }
            }
#if 0
            if((dev->snd_pos + 1) == dev->snd_len) {
                SC_WRITE(sc, dev->snd_buf[dev->snd_pos++]);
                sc->IER &= ~SC_IER_TBE_IE_Msk;
                dev->op_state = SCLIB_OP_IDLE;
            }

            /* actual write */
            if((dev->snd_pos + 1) < dev->snd_len)
                SC_WRITE(sc, dev->snd_buf[dev->snd_pos++]);
#endif
        }
    }

    /* RDR data ready or Rx time out*/
    if(sc->ISR & (SC_ISR_RDA_IS_Msk | SC_ISR_RTMR_IS_Msk)) {
        retval = 1;
        if(dev->op_state == SCLIB_OP_READ) {
            // [2011.11.25]
            /* EMV Certification */
            if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T1)
                SC_StartTimer(sc, 0, SC_TMR_MODE_7, (dev->CardCapabilities.T1.CWT / dev->CardCapabilities.etu) + 4 + 1);

            if(sc->ISR & SC_ISR_RDA_IS_Msk) {
                dev->rcv_buf[dev->rcv_pos + dev->rcv_cnt++] = SC_READ(sc);

                if(dev->rcv_cnt>= dev->rcv_len || dev->rcv_pos + dev->rcv_cnt >= SCLIB_MAX_T1_BUFFER_SIZE) {

                    dev->op_state = SCLIB_OP_IDLE;
                }

            }
        } else if(dev->op_state == SCLIB_OP_ATR_READ) {  /* Read ATR ISR */
            // stop checking timer & start to check waiting time 9600
            SC_StopTimer(sc, 0);
            /* [ISO 7816-3] */
            //SMARTCARD_TimerCountSet(sc, 0, SC_TMR_MODE_0, 9627);  // default waiting time 9600 ETUs
            /* [EMV2000] */
            SC_StartTimer(sc, 0, SC_TMR_MODE_0, 9600 + 480);

            if(atr_total_time_start_flag == 0) {
                atr_total_time_start_flag = 1;
                /* start counting total time for ATR session */
                SC_StopTimer(sc, 2);
                SC_StartTimer(sc, 2, SC_TMR_MODE_4, 256);
                atr_check_time = 1;
            }

            if((sc->ISR & SC_ISR_RDA_IS_Msk) && atr_remain) {
                /*
                 * atr_len==0 : TS
                 * atr_len==1 : T0
                 */
                atrbuffer[atr_len] = SC_READ(sc);
                atr_remain--;
                ifbyte_flag--;

                if(atr_len == 1) {
                    atr_remain += (atrbuffer[atr_len] & 0xf); // Historical byte
                    ifbyte_flag = 0; // T0 contains Y(x) as well
                }

                if( ifbyte_flag == 0 ) {
                    if(atrbuffer[atr_len] & 0x10) {
                        ++atr_remain;
                        ++ifbyte_flag;
                    }
                    if(atrbuffer[atr_len] & 0x20) {
                        ++atr_remain;
                        ++ifbyte_flag;
                    }
                    if(atrbuffer[atr_len] & 0x40) {
                        ++atr_remain;
                        ++ifbyte_flag;
                    }
                    if(atrbuffer[atr_len] & 0x80) {
                        ++atr_remain;
                        ++ifbyte_flag;
                        if((tck == 0) && (atr_len != 1) && ((atrbuffer[atr_len] & 0xf) != 0)) {
                            ++atr_remain; //tck exist
                            tck = 1;
                        }
                    } else {
                        /* Here, it's special case for APDU test card */
                        if((tck == 0) && (atr_len != 1) && ((atrbuffer[atr_len] & 0xf) != 0)) {
                            ++atr_remain; //tck exist
                            tck = 1;
                        }
                        ifbyte_flag = -1;
                    }
                }

                atr_len++;   /* increase the length of ATR */
            }

            if(atr_remain == 0) {   /* receive ATR done */
                dev->CardCapabilities.ATR.Length = atr_len;
                SC_StopTimer(sc, 0);
                SC_StartTimer(sc, 0, SC_TMR_MODE_0, 480);
                SC_StopTimer(sc, 2);
                atr_final_chk = 1;
                if((sc->ISR & SC_ISR_RDA_IS_Msk) && atr_final_chk) {
                    SC_StopTimer(sc, 0);
                    dev->errno = SCLIB_ERR_ATR_INVALID_PARAM;
                    dev->op_state = SCLIB_OP_IDLE;
                    atr_final_chk = 0;
                }
            }
        } else {
            // Discard these data
            char c = SC_READ(SC0);
            SCLIB_ERR("SC%d: Unknown data==>%02x %d\n", num, c, dev->op_state);
        }
    }
    return retval;
}


uint32_t SCLIB_CheckErrorEvent(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];
    uint32_t retval = 0;

    /* auto convention error interrupt  */
    if(sc->ISR & SC_ISR_ACON_ERR_IS_Msk) {
        sc->ISR = SC_ISR_ACON_ERR_IS_Msk;
        dev->errno = SCLIB_ERR_AUTOCONVENTION;
        dev->op_state = SCLIB_OP_IDLE;
        SCLIB_ERR("SC%d: Auto Convention Error\n", num);
        retval = 1;
    }


    /* Transmit Error: break error, frame error, Rx/Tx over flow, parity error, invalid stop */
    if((sc->ISR & SC_ISR_TERR_IS_Msk) && (sc->IER & SC_IER_TERR_IE_Msk)) {
        if(sc->TRSR & SC_TRSR_RX_OVER_F_Msk) {
            sc->TRSR = SC_TRSR_RX_OVER_F_Msk;
            dev->errno = SCLIB_ERR_READ;
            if(dev->CardCapabilities.Protocol.Selected != SCLIB_PROTOCOL_T1)
                dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Rx Over Flow\n", num);
        }

        if(sc->TRSR & SC_TRSR_TX_OVER_F_Msk) {
            sc->TRSR = SC_TRSR_TX_OVER_F_Msk;
            dev->errno = SCLIB_ERR_WRITE;
            dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Tx Over Flow\n", num);
        }

        if(sc->TRSR & SC_TRSR_RX_EPA_F_Msk) {
            sc->TRSR = SC_TRSR_RX_EPA_F_Msk;
            dev->errno = SCLIB_ERR_PARITY_ERROR;
            sc->ALTCTL |= SC_ALTCTL_RX_RST_Msk;
            if(dev->CardCapabilities.Protocol.Selected != SCLIB_PROTOCOL_T1)    // for ATR reception
                dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Rx Parity Error\n", num);
        }

        if(sc->TRSR & SC_TRSR_RX_EBR_F_Msk) {
            sc->TRSR = SC_TRSR_RX_EBR_F_Msk;
            sc->ALTCTL |= SC_ALTCTL_RX_RST_Msk;
            dev->errno = SCLIB_ERR_READ;
            dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Rx Break Error\n", num);
        }

        if(sc->TRSR & SC_TRSR_RX_EFR_F_Msk) {
            sc->TRSR = SC_TRSR_RX_EFR_F_Msk;
            sc->ALTCTL |= SC_ALTCTL_RX_RST_Msk;
            dev->errno = SCLIB_ERR_READ;
            dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Rx Frame Error\n", num);
        }

        if(sc->TRSR & SC_TRSR_TX_OVER_F_Msk) {
            if(dev->op_state == SCLIB_OP_WRITE)
                sc->IER &= ~SC_IER_TBE_IE_Msk;
            sc->TRSR = (SC_TRSR_TX_OVER_F_Msk | SC_TRSR_TX_REERR_Msk);
            sc->ALTCTL |= SC_ALTCTL_TX_RST_Msk;         // Tx software reset
            dev->errno = SCLIB_ERR_WRITE;
            dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Tx Over Retry Error Retry \n", num);
        }

        if(sc->TRSR & SC_TRSR_RX_OVER_F_Msk) {
            sc->TRSR = (SC_TRSR_RX_OVER_F_Msk | SC_TRSR_RX_REERR_Msk);
            dev->errno = SCLIB_ERR_READ;
            dev->op_state = SCLIB_OP_IDLE;
            SCLIB_ERR("SC%d: Rx Over Retry Error Retry \n", num);
        }

        retval = 1;
    }
    return retval;
}

