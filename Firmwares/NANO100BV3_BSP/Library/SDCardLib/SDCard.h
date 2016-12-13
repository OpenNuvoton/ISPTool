/****************************************************************************//**
 * @file     sdcard.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/02/20 4:06p $
 * @brief    Nano100 series SD Card driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "nano100series.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NANO100_SDCARD SDCARD Library
  @{
*/

/** @addtogroup NANO100_SDCARD_EXPORTED_CONSTANTS SDCARD Library Exported Constants
  @{
*/

#define DBG_PRINTF(...)         /*!< Debug printf option, Off */
//#define DBG_PRINTF printf     /*!< Debug printf option, On */

#define PHYSICAL_BLOCK_SIZE 512    /*!< 512 Erase group size = 16 MMC FLASH sectors */

// Command table value definitions
// Used in the MMC_Command_Exec function to
// decode and execute MMC command requests
#define     EMPTY  0    /*!< Command value definitions: EMPTY */
#define     YES   1     /*!< Command value definitions: YES */
#define     NO    0     /*!< Command value definitions: NO */
#define     CMD   0     /*!< Command value definitions: CMD */
#define     RD    1     /*!< Command value definitions: RD */
#define     WR    2     /*!< Command value definitions: WR */
#define     RDB   3     /*!< Command value definitions: RDB */
#define     WDB   4     /*!< Command value definitions: WDB */
#define     R1    0     /*!< Command value definitions: R1 */
#define     R1b   1     /*!< Command value definitions: R1b */
#define     R2    2     /*!< Command value definitions: R2 */
#define     R3    3     /*!< Command value definitions: R3 */
#define     R7    4     /*!< Command value definitions: R7 */

// Start and stop data tokens for single and multiple
// block MMC data operations
#define     START_SBR      0xFE     /*!< START_SBR token */
#define     START_MBR      0xFE     /*!< START_MBR token */
#define     START_SBW      0xFE     /*!< START_SBW token */
#define     START_MBW      0xFC     /*!< START_MBW token */
#define     STOP_MBW       0xFD     /*!< STOP_SBR token */

// Mask for data response Token after an MMC write
#define     DATA_RESP_MASK 0x11     /*!< DATA_RESP_MASK mask */

// Mask for busy Token in R1b response
#define     BUSY_BIT       0x80     /*!< BUSY_BIT mask */

#define BACK_FROM_ERROR { SingleWrite(0xFF); SPI_SET_SS0_HIGH(SPI1); return FALSE;} /*!< macro for SPI write */

typedef union {                        // byte-addressable unsigned long
    uint32_t l;
    uint8_t b[4];
} UINT32;
typedef union {                        // byte-addressable unsigned int
    uint16_t i;
    uint8_t b[2];
} UINT16;

#define SD_SUCCESS  (0) /*!< success */
#define SD_FAIL     (1) /*!< fail */

// Command Table Index Constants:
// Definitions for each table entry in the command table.
// These allow the MMC_Command_Exec function to be called with a
// meaningful parameter rather than a number.
#define     SDv1    1<<1    /*!<Command table index SDv1 */
#define     SDv2    1<<2    /*!<Command table index SDv2 */
#define     MMCv3   1<<3    /*!<Command table index MMCv3 */
#define     SDBlock 1<<4    /*!<Command table index SDBlock */

#define     GO_IDLE_STATE               0/*!<CMD0*/
#define     SEND_OP_COND                1/*!<CMD1*/
#define     SEND_IF_COND                2/*!<CMD8*/
#define     SEND_CSD                    3/*!<CMD9*/
#define     SEND_CID                    4/*!<CMD10*/
#define     STOP_TRANSMISSION           5/*!<CMD12*/
#define     SEND_STATUS                 6/*!<CMD13*/
#define     SET_BLOCKLEN                7/*!<CMD16*/
#define     READ_SINGLE_BLOCK           8/*!<CMD17*/
#define     READ_MULTIPLE_BLOCK         9/*!<CMD18*/
#define     SET_BLOCK_COUNT             10/*!<CMD23*/
#define     WRITE_BLOCK                 11/*!<CMD24*/
#define     WRITE_MULTIPLE_BLOCK        12/*!<CMD25*/
#define     PROGRAM_CSD                 13/*!<CMD27*/
#define     SET_WRITE_PROT              14/*!<CMD28*/
#define     CLR_WRITE_PROT              15/*!<CMD29*/
#define     SEND_WRITE_PROT             16/*!<CMD30*/
#define     TAG_SECTOR_START            17/*!<CMD32*/
#define     TAG_SECTOR_END              18/*!<CMD33*/
#define     UNTAG_SECTOR                19/*!<CMD34*/
#define     TAG_ERASE_GROUP_START       20/*!<CMD35*/
#define     TAG_ERASE_GROUP_END         21/*!<CMD36*/
#define     UNTAG_ERASE_GROUP           22/*!<CMD37*/
#define     ERASE                       23/*!<CMD38*/
#define     LOCK_UNLOCK                 24/*!<CMD42*/
#define     APP_CMD                     25/*!<CMD55*/
#define     READ_OCR                    26/*!<CMD58*/
#define     CRC_ON_OFF                  27/*!<CMD59*/
#define     SD_SEND_STATUS              28/*!<ACMD13*/
#define     SD_SET_WR_BLK_ERASE_COUNT   29/*!<ACMD23*/
#define     SD_SEND_OP_COND             30/*!<ACMD41*/
/*@}*/ /* end of group NANO100_SDCARD_EXPORTED_CONSTANTS */

/** @addtogroup NANO100_SDCARD_EXPORTED_STRUCTS SDCARD Library Exported Structs
  @{
*/
// This structure defines entries into the command table;
typedef struct {
    uint8_t command_byte;      /*!< OpCode;*/
    uint8_t arg_required;      /*!< Indicates argument requirement;*/
    uint8_t CRC;               /*!< Holds CRC for command if necessary;*/
    uint8_t trans_type;        /*!< Indicates command transfer type;*/
    uint8_t response;          /*!< Indicates expected response;*/
    uint8_t var_length;        /*!< Indicates varialble length transfer;*/
} COMMAND;
/*@}*/ /* end of group NANO100_SDCARD_EXPORTED_STRUCTS */

/** @addtogroup NANO100_SDCARD_EXPORTED_FUNCTIONS SDCARD Library Exported Functions
  @{
*/
uint32_t SDCARD_Open(void);
void SDCARD_Close(void);
uint32_t SDCARD_GetVersion(void);
uint32_t MMC_Command_Exec (uint8_t cmd_loc, uint32_t argument,uint8_t *pchar, uint32_t* response);
uint32_t GetLogicSector(void);
uint32_t SDCARD_GetCardSize(uint32_t* pu32TotSecCnt);
void SpiRead(uint32_t addr, uint32_t size, uint8_t* buffer);
void SpiWrite(uint32_t addr, uint32_t size, uint8_t* buffer);

/*@}*/ /* end of group NANO100_SDCARD_EXPORTED_FUNCTIONS */


/*@}*/ /* end of group NANO100_SDCARD */

/*@}*/ /* end of group NANO100_Library */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/



