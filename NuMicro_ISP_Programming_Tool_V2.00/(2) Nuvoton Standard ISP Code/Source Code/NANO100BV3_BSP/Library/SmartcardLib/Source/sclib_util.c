/**************************************************************************//**
 * @file     smartcard_protocol.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/07/31 7:25p $
 * @brief    This file provides smartcard utility functions
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

uint8_t g_RxBUF[SCLIB_MAX_T1_BUFFER_SIZE];
uint8_t g_TxBUF[SCLIB_MAX_T1_BUFFER_SIZE];
extern uint32_t vccDelay[SC_INTERFACE_NUM];

// This is the time resolution.
// We calculate all times not in seconds, but in micro seconds
#define TR ((unsigned long)(1000l * 1000l))     // for time resolution = micro-second unit

// Variables used for receiving ATR
uint32_t atr_len, atr_remain;
int32_t ifbyte_flag, tck, ifcount;
int32_t atr_time;
int32_t atr_check_time;
int32_t atr_total_time_start_flag;
int32_t atr_final_chk;

/**
  * @brief  This function computes 2 to the power of input parameter. An integer version of pow2().
  * @param[in]  u8Exp Exponent
  * @return The power of exponent by 2
  */
static unsigned long  Pow2(uint8_t u8Exp)
{
    unsigned long result = 1;

    while(u8Exp--)
        result *= 2;

    return result;
}

/**
  * @brief  This function gets smartcard clock frequency.
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @return Smartcard frequency in kHZ
  */
uint32_t _SCLIB_GetInterfaceClock(uint32_t num)
{
    uint32_t freq;
    if((CLK->CLKSEL2 & CLK_CLKSEL2_SC_S_Msk) == 0)
        freq = __HXT;
    else if((CLK->CLKSEL2 & CLK_CLKSEL2_SC_S_Msk) == (1 << CLK_CLKSEL2_SC_S_Pos))
        freq = CLK_GetPLLClockFreq();
    else
        freq = __HIRC12M;


    if(num == 0) {
        freq /= (((CLK->CLKDIV0 & CLK_CLKDIV0_SC0_N_Msk) >> (CLK_CLKDIV0_SC0_N_Pos)) + 1);
    } else if(num == 1) {
        freq /= (((CLK->CLKDIV1 & CLK_CLKDIV1_SC1_N_Msk) >> (CLK_CLKDIV1_SC1_N_Pos)) + 1);
    } else {  // 2
        freq /= (((CLK->CLKDIV1 & CLK_CLKDIV1_SC2_N_Msk) >> (CLK_CLKDIV1_SC2_N_Pos)) + 1);
    }
    return (freq /1000);
}

/**
  * @brief  This routine updates the CardCapabilities structure, which holds information about
  *         the smartcard that has just been reset and is currently in use. It reads the
  *         ATR string and retrieves all the relevant information.
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @retval \ref SCLIB_SUCCESS Smartcard warm reset success
  * @retval Others Smartcard warm reset failed
  */
int32_t _SCLIB_UpdateCardCapabilities(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SCLIB_CARD_CAPABILITIES *cardCapabilities = &(dev->CardCapabilities);
    uint8_t * atrString = cardCapabilities->ATR.Buffer;
    int8_t atrLength = cardCapabilities->ATR.Length;
    uint8_t Y, Tck, TA[SCLIB_MAX_ATR_CODE], TB[SCLIB_MAX_ATR_CODE];
    uint8_t TC[SCLIB_MAX_ATR_CODE], TD[SCLIB_MAX_ATR_CODE];
    uint32_t i, fs, numProtocols = 0, protocolTypes = 0;
    int32_t status = SCLIB_SUCCESS;
    uint32_t TA2Present = FALSE;


    if (atrLength < 2) {
        SCLIB_ERR("ATR is too short (Min. length is 2) \n");
        return SCLIB_ERR_ATR_UNRECOGNIZED;
    }


    if (atrString[0] != 0x3b && atrString[0] != 0x3f) {
        SCLIB_ERR("   Initial character %02xh of ATR is invalid\n", atrString[0] );
        return SCLIB_ERR_ATR_UNRECOGNIZED;
    }



    //
    // The caller might be calling this function repeatedly in order to
    // test if the ATR is valid. If the ATR we currently have here is
    // not valid then we need to be able re-invert an inverted ATR.
    //

    atrString += 1;
    atrLength -= 1;

    //
    // Calculate check char, but do not test now since if only T=0
    // is present the ATR doesn't contain a check char
    //
    for (i = 0, Tck = 0; i < atrLength; i++) {

        Tck ^= atrString[i];
    }

    // Initialize various data
    cardCapabilities->Protocol.Supported = 0;

    memset(TA, 0, sizeof(TA));
    memset(TB, 0, sizeof(TB));
    memset(TC, 0, sizeof(TC));
    memset(TD, 0, sizeof(TD));


    // Set default values as described in ISO 7816-3/ EMV

    // TA1 codes Fl in high-byte and Dl in low-byte;
    TA[0] = 0x11;   // Default value in EMV 8.3.3.1
    // TB1 codes II in bits b7/b6 and Pl1 in b5-b1. b8 has to be 0
    TB[0] = 0x25;
    // TC2 codes T=0 WI
    TC[1] = 10;

    // Record historical byte length (ISO-7816 8.2.2)
    cardCapabilities->ATR.HBLen = *atrString & 0x0f;

    Y = *atrString++ & 0xf0;        /* Get Format byte T0 */
    atrLength -= 1;

    for (i = 0; i < SCLIB_MAX_ATR_CODE; i++) {

        if (Y & 0x10) {

            if (i == 1) {

                TA2Present = TRUE;
            }
            TA[i] = *atrString++;
            atrLength -= 1;
        }

        if (Y & 0x20) {
            TB[i] = *atrString++;
            atrLength -= 1;
        }

        if (Y & 0x40) {
            TC[i] = *atrString++;
            atrLength -= 1;
        }

        if (Y & 0x80) {
            Y = *atrString & 0xf0;          /* Set next interface byte TDi */
            TD[i] = *atrString++ & 0x0f;    /* TD[] only set transmission protocol */
            atrLength -= 1;

            // Check if the next parameters are for a new protocol.
            if (((1 << TD[i]) & protocolTypes) == 0) {

                // Count the number of protocols that the card supports
                numProtocols++;
            }
            protocolTypes |= 1 << TD[i];

        } else {

            break;
        }
    }

    // Check if the card supports a protocol other than T=0
    if (protocolTypes & ~1) {

        // The ATR contains a checksum byte.
        // Exclude that from the historical byte length check
        atrLength -=1;

        // This card supports more than one protocol or a protocol
        // other than T=0, so test if the checksum is correct
        if (Tck != 0) {
            SCLIB_ERR("   ATR Checksum is invalid\n");
            status = SCLIB_ERR_ATR_INVALID_TCK;
            goto _exit;
        }
    }

    if (atrLength < 0 || atrLength != cardCapabilities->ATR.HBLen) {

        SCLIB_ERR("   ATR length is inconsistent\n");
        status = SCLIB_ERR_ATR_UNRECOGNIZED;
        goto _exit;
    }

    // Now convert TA - TD values to global interface bytes

    // Clock rate conversion
    cardCapabilities->Fl = (TA[0] & 0xf0) >> 4;

    // bit rate adjustment
    cardCapabilities->Dl = (TA[0] & 0x0f);

    // Extra guard time
    cardCapabilities->N = TC[0];

    //
    // Check if the Dl and Fl values are valid
    //
    if (BitRateAdjustment[cardCapabilities->Dl].DNumerator == 0 ||
            ClockRateConversion[cardCapabilities->Fl].F == 0) {

        SCLIB_ERR("   Dl = %02x or Fl = %02x invalid\n",
                  cardCapabilities->Dl,
                  cardCapabilities->Fl
                 );

        status = SCLIB_ERR_ATR_UNRECOGNIZED;
        goto _exit;
    }

    SCLIB_INFO("   Card parameters from ATR:\n Fl = %02x (Max. %ld KHz), Dl = %02x, N = %02x\n",
               cardCapabilities->Fl,
               ClockRateConversion[cardCapabilities->Fl].fs / 1000,
               cardCapabilities->Dl,
               cardCapabilities->N
              );


    fs = _SCLIB_GetInterfaceClock(num) * 1000l;//dev->clock * 1000l;

    //
    // We calculate the ETU on basis of the timing supplied by the
    // clk-frequency of the reader
    //
    //
    // Work ETU in units of time resolution(TR) (NOT in seconds)
    //
    cardCapabilities->etu = 1 + (TR * ClockRateConversion[cardCapabilities->Fl].F) / (BitRateAdjustment[cardCapabilities->Dl].DNumerator * fs);

    //
    // guard time in micro seconds
    // the guard time is the gap between the end of the
    // current character and the beginning of the next character
    //
    //cardCapabilities->GT = 0;
    cardCapabilities->GT = 12;  // by default GT = 12, modified by SM

    if (cardCapabilities->N == 0) {
        cardCapabilities->GT = 12;
    } else if (cardCapabilities->N == 255) {
        if (protocolTypes & SCLIB_PROTOCOL_T1)
            cardCapabilities->GT = 11;              // T=1, character guard time = 11 ETU
    } else if(cardCapabilities->N != 0) { // modified by SM
        //cardCapabilities->GT = cardCapabilities->N;       // mask by SM
        cardCapabilities->GT = cardCapabilities->N + 12;    // modified by SM
    }

    SCLIB_INFO("   Calculated timing values:\n Work etu = %ld micro sec, Guard time = %ld ETU\n",
               cardCapabilities->etu,
               cardCapabilities->GT
              );

    if (TA2Present || (numProtocols <= 1 && cardCapabilities->Fl == 1 && cardCapabilities->Dl == 1) || dev->pps_complete == 1) {
        //
        // If the card supports only one protocol (or T=0 as default)
        // and only standard parameters then PTS selection is not available
        //
        dev->CurrentState = SCLIB_CARD_SPECIFIC; // modified by SM
    } else {

        dev->CurrentState = SCLIB_CARD_NEGOTIABLE; // modified by SM
    }

    // Now find protocol specific data

    if (TD[0] == 0) {          /* Protocol T=0 */

        cardCapabilities->Protocol.Supported |= SCLIB_PROTOCOL_T0;

        cardCapabilities->T0.WI = TC[1];

        cardCapabilities->T0.WT = 1 +
                                  ( cardCapabilities->T0.WI *
                                    960 * cardCapabilities->etu *
                                    BitRateAdjustment[cardCapabilities->Dl].DNumerator);   // modified by SM


        SCLIB_INFO("   T=0 Values from ATR:\n   WI = %ld\n", cardCapabilities->T0.WI);
        SCLIB_INFO("   T=0 Timing from ATR:\n   WT = %ld us\n", cardCapabilities->T0.WT );

    }

    /* Protocol T=1 */
    if (protocolTypes & SCLIB_PROTOCOL_T1) {
        for (i = 0; TD[i] != 1 && i < SCLIB_MAX_ATR_CODE; i++);
        for (; TD[i] == 1 && i < SCLIB_MAX_ATR_CODE; i++);

        if (i == SCLIB_MAX_ATR_CODE) {
            return SCLIB_ERR_ATR_UNRECOGNIZED;
        }

        cardCapabilities->Protocol.Supported |= SCLIB_PROTOCOL_T1;

        cardCapabilities->T1.IFSC = (TA[i] ? TA[i] : 32);
        cardCapabilities->T1.CWI = (TB[i] & 0x0f);
        cardCapabilities->T1.BWI = (TB[i] & 0xf0) >> 4;
        cardCapabilities->T1.EDC = (TC[i] & 0x01);
        cardCapabilities->T1.CWT = 1 + (Pow2(cardCapabilities->T1.CWI) + 11) * cardCapabilities->etu;

        cardCapabilities->T1.BWT = 1 + ((Pow2(cardCapabilities->T1.BWI) * 960 * BitRateAdjustment[cardCapabilities->Dl].DNumerator) + 11)
                                   * cardCapabilities->etu;

        cardCapabilities->T1.BGT = 22 * cardCapabilities->etu;  // added by SM

        SCLIB_INFO("   T=1 Values from ATR:\n   IFSC = %ld, CWI = %ld, BWI = %ld, EDC = %02x\n",
                   cardCapabilities->T1.IFSC,
                   cardCapabilities->T1.CWI,
                   cardCapabilities->T1.BWI,
                   cardCapabilities->T1.EDC
                  );

        SCLIB_INFO("   T=1 Timing from ATR:\n   CWT = %ld us, BWT = %ld us, BGT = %ld us\n",
                   cardCapabilities->T1.CWT,
                   cardCapabilities->T1.BWT,
                   cardCapabilities->T1.BGT
                  );
    }


    if (dev->CurrentState == SCLIB_CARD_SPECIFIC) {  // modified by SM

        if (TA2Present) {
            // TA2 is present in the ATR, so use the protocol indicated in the ATR
            cardCapabilities->Protocol.Selected = 1 << (TA[1]&0x0F);
        } else {
            // The card only supports one protocol so make that one protocol the current one to use
            cardCapabilities->Protocol.Selected = cardCapabilities->Protocol.Supported;
        }
        SCLIB_INFO("   Mode: Specific %s\n\n",  TA2Present ? "set by TA(2)" : "" );
    } else {
        SCLIB_INFO("   Mode: Negotiable\n\n");
    }

_exit:
    return status;
}


/**
  * @brief This API starts the PPS exchange procedure and return after exchange complete
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in]  Fi Clock rate conversion integer from card
  * @param[in]  Di Baud rate adjustment integer from card
  * @param[in]  T Transmission protocol
  * @return The result of PPS processes.
  * @retval \ref SCLIB_SUCCESS Smartcard warm reset success
  * @retval Others Smartcard warm reset failed
  */
static int32_t _SCLIB_ExchangePPS(uint32_t num, int32_t Fi, int32_t Di, int32_t T)
{
    uint8_t buf[4];
    uint8_t *rbuf;
    int32_t len = 4, rPck=0;
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];


    buf[0] = 0xff;                             /* PPSS: PPS request use 0xFF */
    buf[1] = 0x10 |(T & 0x0f);                 /* PPS0: PPS1 will be transmitted + T transmission protocol */
    buf[2] = ((Fi<<4)&0xf0) | (Di & 0x0f);     /* PPS1: */
    buf[3] = (buf[0] ^ buf[1] ^ buf[2]);       /* PCK */
    SCLIB_INFO("Sending PPS : %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3]);

    SC_ClearFIFO(sc);
    SC_StopAllTimer(sc);
    SCLIB_Delay();
    // Waiting time check, the waiting time of PPS is fixed at 9600 ETUs
    SC_StartTimer(sc, 0, SC_TMR_MODE_7, 9600);

    /* Start to do PPS exchange */
    dev->errno = 0;
    dev->snd_buf = buf;
    dev->rcv_buf = &g_RxBUF[0];
    dev->snd_len = len;
    dev->snd_pos = 0;
    dev->op_state = SCLIB_OP_WRITE;
    sc->IER  |= SC_IER_TBE_IE_Msk;
    while(dev->op_state == SCLIB_OP_WRITE && !(dev->errno));
    sc->IER  &= ~SC_IER_TBE_IE_Msk;
    dev->snd_buf = NULL;

    if(dev->errno != 0) {
        SCLIB_ERR("PPS Write..Error: %d \n", dev->errno);
        return dev->errno;
    }

    /* Read Procedure bytes */
    dev->errno = 0;
    rbuf = dev->rcv_buf;
    dev->rcv_pos = 0;
    dev->rcv_len = len;
    dev->rcv_cnt = 0;
    dev->op_state = SCLIB_OP_READ;
    while(dev->op_state == SCLIB_OP_READ && (dev->bCardRemoved == SCLIB_CARD_PRESENT) && !(dev->errno));

    SC_StopTimer(sc, 0);

    if(dev->errno != 0) {
        SCLIB_ERR("PPS Read..Error:%d \n", dev->errno);
        return dev->errno;
    }

    SCLIB_INFO("Received PPS : %02x %02x %02x %02x\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);

    if(rbuf[0] != buf[0])   /* PPSS */
        return SCLIB_ERR_PPS;

    rPck = rbuf[0];
    if((rbuf[1]&0x0f) == (buf[1] &0x0f) &&
            ((rbuf[1] & 0xf0) == 0x10 ||(rbuf[1] & 0xf0) == 0x00)) {
        rPck ^= rbuf[1];
        SCLIB_INFO("PPS Request Success \n");
    } else
        return SCLIB_ERR_PPS;

    if (rbuf[2] == buf[2])
        rPck ^= rbuf[2];
    else
        return SCLIB_ERR_PPS;

    if (rbuf[3] != rPck)  /* PCK */
        return SCLIB_ERR_PPS;

    return SCLIB_SUCCESS;
}


/**
  * @brief  Set the baudrate according to clock rate conversion integer and baud rate adjustment integer
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in] fi Clock rate conversion integer from card
  * @param[in] di Baud rate adjustment integer from card
  * @return None
  */
static void _SCLIB_SetBaudrate(uint32_t num, int32_t fi, int32_t di)
{
    SC_T *sc = _scBase[num];
    uint32_t etudiv;
    uint32_t fi_val, di_val, remainder;


    fi_val = ClockRateConversion[fi].F;
    di_val = BitRateAdjustment[di].DNumerator;
    remainder = ((fi_val * 10) / di_val) % 10;

    if(remainder >= 5) {
        etudiv = (ClockRateConversion[fi].F / BitRateAdjustment[di].DNumerator);
        sc->ETUCR = SC_ETUCR_COMPEN_EN_Msk | etudiv;
    } else {
        etudiv = (ClockRateConversion[fi].F / BitRateAdjustment[di].DNumerator) - 1;
        sc->ETUCR = etudiv;
    }

}

/**
  * @brief  Set Character Guard Time
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @return None
  */
static void _SCLIB_SetCharacterGuardTime(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];
    uint32_t gt = dev->CardCapabilities.GT;

    /* [EMV 2000] EMV in T=0, minimum GT is 12 */
    if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T0) {
        if(gt==11)
            gt = 12;
        else {
            sc->CTL &= ~SC_CTL_SLEN_Msk;        // 2 ETU stop bit
            sc->EGTR = (sc->EGTR & ~(SC_EGTR_EGT_Msk)) | (gt-12);
        }
    }

    if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T1) {
        if(gt==11) {
            sc->CTL |= SC_CTL_SLEN_Msk; // 1 ETU stop bit
            sc->EGTR = (sc->EGTR & ~(SC_EGTR_EGT_Msk));
        } else {
            sc->CTL |= SC_CTL_SLEN_Msk; // 1 ETU stop bit
            sc->EGTR = (sc->EGTR & ~(SC_EGTR_EGT_Msk)) | (gt-11);
        }
    }
}



/**
  * @brief  According ATR information, basic settings will be set the new value about ETU,
  *         character guard time,  and block guard time.
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @return None
  */
static void _SCLIB_SetReaderParameter(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    _SCLIB_SetBaudrate(num, dev->CardCapabilities.Fl, dev->CardCapabilities.Dl);
    _SCLIB_SetCharacterGuardTime(num);

    if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T0) {
        SC_SetBlockGuardTime(sc, 16);   //BGT = 16
    } else if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T1) {
        SC_SetBlockGuardTime(sc, 22);   //BGT = 22
        /* set send-sequence & more-data bit */
        dev->T1.SSN = 1;        // default value
        dev->T1.RSN = 0;        // default value
        dev->T1.IBLOCK_REC = 0; // default value
    }

    /* Tx & Rx Error Retry Settings */
    /* [EMV 2000] */
    if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T0) {
        SC_SetTxRetry(sc, 4);           // Tx retry 4 times
        SC_SetRxRetry(sc, 4);           // Rx retry 4 times
    }

}

/**
  * @brief  Parse ATR characters and check its integrity
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in]  u32ResetType Indicate reset type. Could be ether \ref SCLIB_COLDRESET or \ref SCLIB_WARMRESET
  * @return Parse ATR result
  * @retval SCLIB_SUCCESS Success
  * @retval SC_ERR_UNSUPPORTEDCARD      Card is not supported
  * @retval ATR_ERR_UNRECOGNIZED_MEDIA  Invalid to the length of ATR or TS byte or ATR checksum or Fi/Di
  * @retval SCLIB_ERR_ATR_INVALID_PARAM   Content of ATR has wrong parameter. Should start a warm reset is reset type is \ref SCLIB_WARMRESET
  */
int32_t _SCLIB_ParseATR(uint32_t num, uint32_t u32ResetType)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    int32_t i, retval;
    uint8_t td;
    uint8_t *atr = dev->CardCapabilities.ATR.Buffer;
    uint8_t T, Fi, Di;
    unsigned long protocolTypes = 0;

    struct {
        uint8_t data;
        uint8_t present;
    } ibyte[4][4];

    memset(&ibyte, 0, sizeof(ibyte));

    atr++;  // skip: TS->T0
    td = *atr++;

    i = 0;

    while(1) {
        /* TAi present */
        if( td & 0x10) {
            ibyte[i][ATR_INTERFACE_BYTE_TA].data = *atr++;
            ibyte[i][ATR_INTERFACE_BYTE_TA].present = 1;
        }

        /* TBi present */
        if( td & 0x20) {
            ibyte[i][ATR_INTERFACE_BYTE_TB].data = *atr++;
            ibyte[i][ATR_INTERFACE_BYTE_TB].present = 1;
        }

        /* TCi present */
        if( td & 0x40) {
            ibyte[i][ATR_INTERFACE_BYTE_TC].data = *atr++;
            ibyte[i][ATR_INTERFACE_BYTE_TC].present = 1;
        }

        /* TDi present */
        if( td & 0x80) {
            ibyte[i][ATR_INTERFACE_BYTE_TD].data = *atr++;
            ibyte[i][ATR_INTERFACE_BYTE_TD].present = 1;
            td = ibyte[i][ATR_INTERFACE_BYTE_TD].data;
            protocolTypes |= 1 << (td & 0x0F);
            i++;
        } else
            break;
    }

    if(dev->EMV == TRUE) {
        /**********************************************************************************************************************/
        /* Checking the integrity of ATR, this process meets EMV 4.2 specification */
        /* Reject ATR if TA1 is not int the range '11' to '13' */
        if(ibyte[0][ATR_INTERFACE_BYTE_TA].present == 1) {
            if(ibyte[0][ATR_INTERFACE_BYTE_TA].data < 0x11 || ibyte[0][ATR_INTERFACE_BYTE_TA].data > 0x13)
                return SCLIB_ERR_ATR_INVALID_PARAM;
        }

        /* In response to the cold-reset, TB1 only could be 0x00 */
        if(u32ResetType == SCLIB_COLDRESET) {
            if(ibyte[0][ATR_INTERFACE_BYTE_TB].present == 1)
                if(ibyte[0][ATR_INTERFACE_BYTE_TB].data != 0x00)
                    return SCLIB_ERR_ATR_INVALID_PARAM;
            if(ibyte[0][ATR_INTERFACE_BYTE_TB].present == 0)
                return SCLIB_ERR_ATR_INVALID_PARAM;
        }

        /* Reject ATR containing TB2 */
        if(ibyte[1][ATR_INTERFACE_BYTE_TB].present == 1)
            return SCLIB_ERR_ATR_INVALID_PARAM;


        /* ATR must contain TB3 in T=1 */
        if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T1) {
            if(ibyte[2][ATR_INTERFACE_BYTE_TB].present == 0)
                return SCLIB_ERR_ATR_INVALID_PARAM;
        }

        /* Bit [5] of TA2 must be equal to 0x0 */
        if(ibyte[1][ATR_INTERFACE_BYTE_TA].present == 1) {
            if((ibyte[1][ATR_INTERFACE_BYTE_TA].data & 0x10) == 0x10)
                return SCLIB_ERR_ATR_INVALID_PARAM;
        }

        /* Reject an ATR that TC2 is equal to 0x00 */
        if(ibyte[1][ATR_INTERFACE_BYTE_TC].present == 1 && ibyte[1][ATR_INTERFACE_BYTE_TC].data == 0x00)
            return SCLIB_ERR_ATR_INVALID_PARAM;


        /* TD1's l.s. nibble must be 0x0 or 0x1 */
        if(ibyte[0][ATR_INTERFACE_BYTE_TD].present == 1) {
            if((ibyte[0][ATR_INTERFACE_BYTE_TD].data & 0xF) > 0x1) {
                return SCLIB_ERR_ATR_INVALID_PARAM;
            }
        }

        /* TD2's l.s. nibble must be 0x1 or 0xE if TD1's l.s. nibble is 0x0 */
        if(ibyte[1][ATR_INTERFACE_BYTE_TD].present == 1) {
            if((ibyte[1][ATR_INTERFACE_BYTE_TD].data & 0xF)!=0x1 && (ibyte[1][ATR_INTERFACE_BYTE_TD].data & 0xF) != 0xE)
                return SCLIB_ERR_ATR_INVALID_PARAM;

            if((ibyte[1][ATR_INTERFACE_BYTE_TD].data & 0xF) == 0xE) {
                if((ibyte[0][ATR_INTERFACE_BYTE_TD].data & 0xF) != 0x0)
                    return SCLIB_ERR_ATR_INVALID_PARAM;
            }
        }

        /* Reject TA3 having a value in the range 0x0~0xF or 0xFF */
        if(ibyte[2][ATR_INTERFACE_BYTE_TA].present == 1) {
            if(ibyte[2][ATR_INTERFACE_BYTE_TA].data < 0x10 || ibyte[2][ATR_INTERFACE_BYTE_TA].data == 0xFF) {
                return SCLIB_ERR_ATR_INVALID_PARAM;
            }

        }

        /* Reject ATR not containing TB3 or BWI greater than 4 or CWI greater than 5 */
        /* And reject ATR if fitting the formula : 2 to the power of CWI is equal or less than (N+1) */
        if(dev->CardCapabilities.Protocol.Selected == SCLIB_PROTOCOL_T1) {
            if(ibyte[2][ATR_INTERFACE_BYTE_TB].present == 1) {
                if(((ibyte[2][ATR_INTERFACE_BYTE_TB].data & 0xF0) >> 4) > 0x4)
                    return SCLIB_ERR_ATR_INVALID_PARAM;

                if((ibyte[2][ATR_INTERFACE_BYTE_TB].data & 0xF) > 0x5)
                    return SCLIB_ERR_ATR_INVALID_PARAM;

                i = 1;
                retval = (ibyte[2][ATR_INTERFACE_BYTE_TB].data & 0xF);
                while(retval--)
                    i *= 2;
                /* if TC1 is equal to 0xFF, N as -1 that is always valid */
                if(ibyte[0][ATR_INTERFACE_BYTE_TC].data != 0xFF)
                    if( i <= (ibyte[0][ATR_INTERFACE_BYTE_TC].data + 1))
                        return SCLIB_ERR_ATR_INVALID_PARAM;

            } else
                return SCLIB_ERR_ATR_INVALID_PARAM;
        }

        /* Reject ATR if TC3 is not equal to 0x00 */
        if(ibyte[2][ATR_INTERFACE_BYTE_TC].present == 1) {
            if(ibyte[2][ATR_INTERFACE_BYTE_TC].data != 0x00) {
                return SCLIB_ERR_ATR_INVALID_PARAM;
            }
        }
        /* End of checking the integrity of ATR */
        /**********************************************************************************************************************/
    }

    Fi = Di = 1;

    /* set Fi and Di if TA1 present */
    if(ibyte[0][ATR_INTERFACE_BYTE_TA].present) {
        Fi = (ibyte[0][ATR_INTERFACE_BYTE_TA].data >> 4) & 0x0f;
        Di = ibyte[0][ATR_INTERFACE_BYTE_TA].data  & 0x0f;
    }


    T = 0;

    /* check TA2 indicates which mode in the card */
    /* in specific mode */
    if(dev->CurrentState == SCLIB_CARD_SPECIFIC) {
        T = ibyte[1][ATR_INTERFACE_BYTE_TA].data & 0x0f;

        if(ibyte[1][ATR_INTERFACE_BYTE_TA].data & 0x10) { /* Bit5 = 1, using default value */
            Fi = 1;
            Di= 1;          /* default value */
        }
    } else if(dev->CurrentState == SCLIB_CARD_NEGOTIABLE) {     /* in negotiable mode */
        if (ibyte[0][ATR_INTERFACE_BYTE_TD].present)     // use "first offered transmission protocol"
            T = ibyte[0][ATR_INTERFACE_BYTE_TD].data & 0x0f;
        else
            T = 0;          // means protocol T=0

        /* if the values are not default, exchange PPS  */
        if((Fi != 0) || (Di != 1)) {
            retval = _SCLIB_ExchangePPS(num, Fi, Di, T);               /* parameter and protocol select */
            if(retval != SCLIB_SUCCESS) {
                /* PPS failed */
                SCLIB_ERR("PPS failed!!\n");
                return(retval);
            } else {
                dev->pps_complete = 1;
                _SCLIB_UpdateCardCapabilities(num);
            }
        }

    }

    _SCLIB_SetReaderParameter(num);

    return SCLIB_SUCCESS;
}



/**
  * @brief Generate cold reset signal for specified smartcard interface
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @return None
  */
void _SCLIB_StartColdReset(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    // disable Timer1 interrupt and use polling method to check time-out happened
    sc->IER &= ~SC_IER_TMR1_IE_Msk;
    sc->CTL |= SC_CTL_DIS_RX_Msk;  
    // VCC high
    SC_SET_VCC_PIN(sc, SC_PIN_STATE_HIGH);
    // Clock
    SC_StartTimer(sc, 1, SC_TMR_MODE_0, 1 + vccDelay[num]);


    while(((sc->ISR & SC_ISR_TMR1_IS_Msk) == 0x00) && (dev->errno != SCLIB_ERR_CARD_REMOVED)) ;
    // clear timeout status
    sc->ISR = SC_ISR_TMR1_IS_Msk;

    // Start clock
    SC_SET_CLK_PIN(sc, SC_CLK_ON);
//    do {
//        uint32_t reg = (sc)->PINCSR;
//        if((reg & (SC_PINCSR_POW_EN_Msk | SC_PINCSR_POW_INV_Msk)) == 0 ||
//            (reg & (SC_PINCSR_POW_EN_Msk | SC_PINCSR_POW_INV_Msk) == (SC_PINCSR_POW_EN_Msk | SC_PINCSR_POW_INV_Msk)))
//            reg &= ~SC_PINCSR_POW_EN_Msk;
//        else
//            reg |= SC_PINCSR_POW_EN_Msk;
//        if(SC_CLK_ON)
//            (sc)->PINCSR = reg | SC_PINCSR_CLK_KEEP_Msk;
//        else\
//            (sc)->PINCSR = reg & ~SC_PINCSR_CLK_KEEP_Msk;
//    }while(0);
    // I/O pin high
    SC_SET_IO_PIN(sc, SC_PIN_STATE_HIGH);
//    do {\
//        uint32_t reg = (sc)->PINCSR;\
//        if((reg & (SC_PINCSR_POW_EN_Msk | SC_PINCSR_POW_INV_Msk) == 0) ||\
//            (reg & (SC_PINCSR_POW_EN_Msk | SC_PINCSR_POW_INV_Msk) == (SC_PINCSR_POW_EN_Msk | SC_PINCSR_POW_INV_Msk)))\
//            reg &= ~SC_PINCSR_POW_EN_Msk;\
//        else\
//            reg |= SC_PINCSR_POW_EN_Msk;\
//        if(SC_PIN_STATE_HIGH)\
//            (sc)->PINCSR = reg | SC_PINCSR_SC_DATA_O_Msk;\
//        else\
//            (sc)->PINCSR = reg & ~SC_PINCSR_SC_DATA_O_Msk;\
//    }while(0);
    // [2011.11.24]
    /* EMV Certification: low clock cycles number(39814) from clk high to rst high (cold reset)  */
    SC_StartTimer(sc, 1, SC_TMR_MODE_0, 109);   // 108*372 = 40176 clocks

    while(((sc->ISR & SC_ISR_TMR1_IS_Msk) == 0x00) && (dev->errno != SCLIB_ERR_CARD_REMOVED)) ;
    // clear timeout status
    sc->ISR = SC_ISR_TMR1_IS_Msk;

    // enable Timer1 interrupt
    sc->IER |= SC_IER_TMR1_IE_Msk;

    // RST pin high
    SC_SET_RST_PIN(sc, SC_PIN_STATE_HIGH);

    dev->errno = 0;
    dev->op_state = SCLIB_OP_ATR_READ;
    sc->CTL &= ~SC_CTL_DIS_RX_Msk;                            // enable RX

    // wait 42036 clock for ATR
    SC_StartTimer(sc, 0, SC_TMR_MODE_0, (42000/372) + 13);

}

/**
  * @brief Generate warm reset signal for specified smartcard interface
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @return None
  */
static void _SCLIB_StartWarmReset(uint32_t num)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    // Wait for Tx & Rx get into idle mode
    while((!(sc->TRSR & SC_TRSR_TX_EMPTY_F_Msk)) || (sc->TRSR & SC_TRSR_TX_ATV_Msk)) ;
    while((!(sc->TRSR & SC_TRSR_RX_EMPTY_F_Msk)) || (sc->TRSR & SC_TRSR_RX_ATV_Msk)) ;
    // Reset
    while(dev->op_state != SCLIB_OP_IDLE);       // wait for idle mode
    sc->CTL |= SC_CTL_DIS_RX_Msk;                // disable RX before RST falls, has seen garbage on I/O pin on some board
    SC_SET_RST_PIN(sc, SC_PIN_STATE_LOW);

    dev->errno = 0;

    SC_StartTimer(sc, 1, SC_TMR_MODE_0, (40000/(sc->ETUCR & SC_ETUCR_ETU_RDIV_Msk))+1);     // exceeds 40000 clocks

    while(dev->errno != SCLIB_ERR_TIME1OUT && dev->errno != SCLIB_ERR_CARD_REMOVED) ;
    SC_SET_RST_PIN(sc, SC_PIN_STATE_HIGH);
    sc->CTL &= ~SC_CTL_DIS_RX_Msk;                            // enable RX after RST rises

    dev->errno = 0;
    dev->op_state = SCLIB_OP_ATR_READ;

    // wait 42036 clock for ATR
    SC_StartTimer(sc, 0, SC_TMR_MODE_0, (42000/372) + 13);

}


/**
  * @brief To do the activation sequence and start receiving and parsing ATR information.
  * @param[in] num Smartcard interface number. From 0 ~ ( \ref SC_INTERFACE_NUM - 1)
  * @param[in] u32ResetType Indicate reset type. Could be ether \ref SCLIB_COLDRESET or \ref SCLIB_WARMRESET
  * @return  The result status of check ATR information
  * @retval SCLIB_SUCCESS  Card activation success
  * @retval Others Error occurred during card activation
  */
int32_t _SCLIB_ResetCard(uint32_t num, uint32_t u32ResetType)
{
    SCLIB_DEV_T *dev = &_scDev[num];
    SC_T *sc = _scBase[num];

    SC_ResetReader(sc);

    _SCLIB_SetBaudrate(num, 1, 1);        // default 1 ETU == 372 cycles

    memset(&dev->CardCapabilities, 0, sizeof(SCLIB_CARD_CAPABILITIES));
    memset(&dev->T0, 0, sizeof(T0_DATA));
    memset(&dev->T1, 0, sizeof(T1_DATA));

    // stop timer0-2
    SC_StopAllTimer(sc);


    /* reset the global variable for receiving ATR */
    atr_remain = 2; // TS & T0
    ifbyte_flag = -1;
    atr_time = SCLIB_ATR_TOTAL_TIME;
    atr_len = tck = ifcount = atr_check_time = atr_final_chk = atr_total_time_start_flag = 0;

    /* read ATR */
    //dev->CardCapabilities.ATR.Length = 0;

#if 0   // timer 0 configured in cold reset and warm reset APIs.
    SC_StartTimer(sc, 0, SC_TMR_MODE_3, (42000/372) + 1);
#endif

    if(u32ResetType == SCLIB_COLDRESET)
        _SCLIB_StartColdReset(num);
    else if(u32ResetType == SCLIB_WARMRESET)
        _SCLIB_StartWarmReset(num);

    while(dev->op_state==SCLIB_OP_ATR_READ ) {
        // [2011.11.28]
        /* EMV Certification */
        if(dev->errno == SCLIB_ERR_TIME2OUT || dev->errno == SCLIB_ERR_TIME0OUT)  // 2012-04-12 Add time0out avoid user insert card in wrong direction  --ya
            break;
    }

    if(dev->errno != SCLIB_SUCCESS) {
        // [2011.11.28]
        /* EMV Certification */
        if(dev->errno == SCLIB_ERR_TIME2OUT ||
                dev->errno == SCLIB_ERR_TIME0OUT ||
                dev->errno == SCLIB_ERR_CARD_REMOVED)
            SCLIB_Deactivate(num);
        SCLIB_ERR("Failed to read ATR..Error Msg:%d \n", dev->errno);
    }

    return dev->errno;
}

/*@}*/ /* end of group NUC400_SCLIB_PRIVATE_FUNCTIONS */

/*@}*/ /* end of group NUC400_SC_Library */

/*@}*/ /* end of group NUC400_Library */


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/




