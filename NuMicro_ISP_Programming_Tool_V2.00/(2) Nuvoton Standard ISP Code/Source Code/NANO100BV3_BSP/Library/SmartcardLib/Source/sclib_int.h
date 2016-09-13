/**************************************************************************//**
 * @file     sclib_int.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/07/31 7:25p $
 * @brief    Smartcard library header file used internally
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

//TODO: separate a internal header file for T1 protocol?

#ifndef __SCLIB_INT_H__
#define __SCLIB_INT_H__
#include "../Include/sclib.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define SCLIB_DBG_LEVEL 0

#if (SCLIB_DBG_LEVEL == 0)
#define SCLIB_ERR(...)
#define SCLIB_INFO(...)
#define SCLIB_T1INFO(...)
#elif (SCLIB_DBG_LEVEL == 1)
#define SCLIB_ERR       printf
#define SCLIB_INFO(...)
#define SCLIB_T1INFO(...)
#elif (SCLIB_DBG_LEVEL == 2)
#define SCLIB_ERR       printf
#define SCLIB_INFO      printf
#define SCLIB_T1INFO(...)
#else
#define SCLIB_ERR       printf
#define SCLIB_INFO      printf
#define SCLIB_T1INFO    printf
#endif

#define SCLIB_MAX_ATR_CODE              4
#define SCLIB_MAX_T1_RETRANSMIT_CNT     3
#define SCLIB_ATR_TOTAL_TIME            20050   ///< Receiving an ATR shall have a duration of less than or equal to initial ETUs

// Card status
#define SCLIB_CARD_UNKNOWN                   0x0001  /*!< Unknown state */
#define SCLIB_CARD_ABSENT                    0x0002  /*!< Card is absent */
#define SCLIB_CARD_PRESENT                   0x0004  /*!< Card is present */
//#define SCLIB_CARD_SWALLOWED                 0x0008  /*!< Card not powered */
//#define SCLIB_CARD_POWERED                   0x0010  /*!< Card is powered */
#define SCLIB_CARD_NEGOTIABLE                0x0020  /*!< Ready for PTS */
#define SCLIB_CARD_SPECIFIC                  0x0040  /*!< PTS has been set */

// Card OP state
#define SCLIB_OP_IDLE                      (0x00)
#define SCLIB_OP_ATR_READ                  (0x01)
#define SCLIB_OP_READ                      (0x02)
#define SCLIB_OP_WRITE                     (0x03)

// Reset type
#define SCLIB_COLDRESET                     (0x1)
#define SCLIB_WARMRESET                     (0x2)

#define ATR_INTERFACE_BYTE_TA           0       /* Interface byte TAi */
#define ATR_INTERFACE_BYTE_TB           1       /* Interface byte TBi */
#define ATR_INTERFACE_BYTE_TC           2       /* Interface byte TCi */
#define ATR_INTERFACE_BYTE_TD           3       /* Interface byte TDi */


// T = 1 related constants
#define SCLIB_T1_CRC_CHECK        1  ///< Error detection bit as defined by ISO
// PCB type
#define SCLIB_T1_BLOCK_I                0x00
#define SCLIB_T1_BLOCK_R                0x80
#define SCLIB_T1_BLOCK_S                0xC0

#define SCLIB_MAX_T1_BUFFER_SIZE        271
#define SCLIB_MAX_T1_BLOCK_SIZE         259
#define SCLIB_MAX_T1_BLOCK_INF_SIZE     254

#define SCLIB_T1_BLOCK_S_RESYNCH_REQ          0xC0
#define SCLIB_T1_BLOCK_S_RESYNCH_RES          0xE0
#define SCLIB_T1_BLOCK_S_IFS_REQ              0xC1
#define SCLIB_T1_BLOCK_S_IFS_RES              0xE1
#define SCLIB_T1_BLOCK_S_ABORT_REQ            0xC2
#define SCLIB_T1_BLOCK_S_ABORT_RES            0xE2
#define SCLIB_T1_BLOCK_S_WTX_REQ              0xC3
#define SCLIB_T1_BLOCK_S_WTX_RES              0xE3
#define SCLIB_T1_BLOCK_S_VPP_ERR              0xE4

// no gaurantee CLK API will be included, so implement our own tiny delay
#define SCLIB_Delay()   {int volatile ii; for(ii = 0; ii < 10; ii++);}

#if defined ( __CC_ARM  )
#pragma anon_unions
#endif

/** Clock rate conversion table according to ISO */
typedef struct {
    const unsigned long F;
    const unsigned long fs;
} CLOCK_RATE_CONVERSION;

static CLOCK_RATE_CONVERSION ClockRateConversion[] = {
    { 372,  4000000     },
    { 372,  5000000     },
    { 558,  6000000     },
    { 744,  8000000     },
    { 1116, 12000000    },
    { 1488, 16000000    },
    { 1860, 20000000    },
    { 0,    0            },
    { 0,    0            },
    { 512,  5000000     },
    { 768,  7500000     },
    { 1024, 10000000    },
    { 1536, 15000000    },
    { 2048, 20000000    },
    { 0,    0            },
    { 0,    0            }
};

/**
 * Bit rate adjustment factor
 * The layout of this table has been slightly modified due to
 * the unavailability of floating point math support in the kernel.
 * The value D has been divided into a numerator and a divisor.
 */
typedef struct {

    const unsigned long DNumerator;
    const unsigned long DDivisor;

} BIT_RATE_ADJUSTMENT;

static BIT_RATE_ADJUSTMENT BitRateAdjustment[] = {

    { 0,    0   },
    { 1,    1   },
    { 2,    1   },
    { 4,    1   },
    { 8,    1   },
    { 16,   1   },
    { 32,   1   },
    { 64,   1   },
    { 12,   1   },
    { 20,   1   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   }
};


typedef struct {

    uint8_t   IFSC;  ///< Current information field size that can be transmitted
    uint8_t   IFSD;  ///< Current information field size we can receive
    uint8_t   IBLOCK_REC; ///< Record if received I-block was sent correctly from ICC
    uint8_t   RSN;  ///< The 'number' of received I-Blocks
    uint8_t   SSN;  ///< The 'number' of sent I-Blocks as defined in ISO 7816-3
    uint8_t   WTX;   ///< Waiting time extension requested by the smart card, This value should be used by the driver to extend block waiting time
} T1_DATA;

typedef struct {
    uint32_t   Lc;    ///< Number of data bytes in this request
    uint32_t   Le;    ///< Number of expected bytes from the card

} T0_DATA;

/**
 * This struct holds information for the card currently in use
 * The driver must store a received ATR into the ATR struct which is
 * part of this struct. The lib will get all other information
 * out of the ATR.
 */
typedef struct {

    unsigned long   etu;    ///< Smartcard clock unit ; ETU = F/D * 1/f

    struct {
        uint8_t Buffer[32]; ///< Max ATR length is 32 bytes
        uint8_t Length;     ///< Total ATR length
        uint8_t HBLen;      ///< Historical bytes length
    } ATR;


    // !!! DO NOT MODIFY ANY OF THE BELOW VALUES
    // OTHERWISE THE LIBRARY WON'T WORK PROPERLY


    uint8_t Fl;       ///< Clock rate conversion
    uint8_t Dl;       ///< Bit rate adjustment
    uint8_t N;        ///< Extra guard time in ETU

    uint32_t GT;        ///< Guard Time in ETU

    struct {
        uint32_t Supported;    ///< This is a bit mask of the supported protocols
        uint32_t Selected;     ///< The currently selected protocol
    } Protocol;

    /** T=0 specific data */
    struct {
        uint8_t WI;         ///< Waiting integer
        uint32_t WT;        ///< Waiting time in micro seconds
    } T0;

    /** T=1 specific data */
    struct {
        uint8_t IFSC;       ///< Information field size of card
        uint8_t CWI;        ///< Character waiting integer
        uint8_t BWI;        ///< Block waiting integer
        uint8_t EDC;        ///< Error detection code
        uint32_t CWT;       ///< Character and block waiting time in micro seconds
        uint32_t BWT;       ///< Character and block waiting time in micro seconds
        uint32_t BGT;       ///< Block guarding time in micro seconds
    } T1;

} SCLIB_CARD_CAPABILITIES;

typedef struct {
    volatile uint32_t op_state; ///< Card operating state, one of following: SCLIB_OP_IDLE, SCLIB_OP_ATR_READ, SCLIB_OP_READ, SCLIB_OP_WRITE
    int32_t errno;              ///< Records latest error code on this slot
    uint32_t openflag;          ///< 1: this card has been activated. 0: this card is in deactivate state
    uint32_t EMV;               ///< Apply protocol check comply with EMV on this interface or not

    /** Capabilities of the current inserted card */
    SCLIB_CARD_CAPABILITIES CardCapabilities;

    /**
     * Current state of reader (card present/removed/activated)
     * Use OsData->SpinLock to access this member
     * (mandatory)
     * use for PPS
     */
    uint32_t   CurrentState;    ///< Card is in SCLIB_CARD_NEGOTIABLE mode or SCLIB_CARD_SPECIFIC mode.

    uint8_t *snd_buf, *rcv_buf;
    volatile uint32_t snd_pos, snd_len;
    volatile uint32_t rcv_len, rcv_pos;
    volatile uint32_t rcv_cnt;  // cnt is current received data number in write stage read index in read stage
    volatile uint32_t bCardRemoved;     ///< Card is ermoved from slot
    uint32_t pps_complete;  ///< PPS process complete.

    T0_DATA T0;             ///< Data for T=0
    T1_DATA T1;             ///< Data for T=1

} SCLIB_DEV_T;

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

extern SC_T *_scBase[SC_INTERFACE_NUM];
extern SCLIB_DEV_T _scDev[SC_INTERFACE_NUM];


int32_t _SCLIB_SendSBlock(uint32_t num, uint8_t control, uint8_t data);
int32_t _SCLIB_GetBlockType(uint8_t* block);
int32_t _SCLIB_T0Transmit(uint32_t num, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rspBuf, uint32_t *rspLen);
int32_t _SCLIB_T1Transmit(uint32_t num, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rspBuf, uint32_t *rspLen);
int32_t _SCLIB_UpdateCardCapabilities(uint32_t num);
int32_t _SCLIB_ParseATR(uint32_t num, uint32_t u32ResetType);
int32_t _SCLIB_ResetCard(uint32_t num, uint32_t u32ResetType);
uint8_t _SCLIB_GetLEN(uint8_t* block);
uint8_t _SCLIB_GetNAD(uint8_t* block);
uint32_t _SCLIB_GetInterfaceClock(uint32_t num);

#ifdef __cplusplus
}
#endif

#endif //__SCLIB_INT_H__

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
