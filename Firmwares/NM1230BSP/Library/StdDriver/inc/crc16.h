/**************************************************************************//**
 * @file     crc16.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 2018/07/16 17:16 $
 * @brief    NM1230 series CRC16 driver header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CRC16_H__
#define __CRC16_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_CRC16_Driver CRC16 Driver
  @{
*/

/** @addtogroup NM1230_CRC16_EXPORTED_CONSTANTS CRC16 Exported Constants
  @{
*/


/*---------------------------------------------------------------------------------------------------------*/
/*  BYTEORD / BITORD of CRC16_CTL register definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define BYTE_ORD_BIG_ENDIAN              (0x0UL<<CRC16_CTL_BYTEORD_Pos)     /*!< Select big endian for byte order       */
#define BYTE_ORD_LITTLE_ENDIAN           (0x1UL<<CRC16_CTL_BYTEORD_Pos)     /*!< Select little endian for byte order    */
#define BIT_ORD_BIG_ENDIAN               (0x0UL<<CRC16_CTL_BITORD_Pos)      /*!< Select big endian for bit order        */
#define BIT_ORD_LITTLE_ENDIAN            (0x1UL<<CRC16_CTL_BITORD_Pos)      /*!< Select little endian for bit order     */

/*@}*/ /* end of group NM1230_CRC16_EXPORTED_CONSTANTS */


/** @addtogroup NM1230_CRC16_EXPORTED_FUNCTIONS CRC16 Exported Functions
  @{
*/


/**
  * @brief      Set CRC16 seed value
  * @param[in]  u16Seed   16 bits Seed value
  * @retval     None
  * @details    This macro is used to set CRC16 seed value.
  */
#define CRC16_SET_SEED(u16Seed)     { CRC16->CTL = (CRC16->CTL & ~CRC16_CTL_SEED_Msk) | (u16Seed); }


/**
  * @brief      Set CRC16 byte/bit order of the data in
  * @param[in]  byte_order
  *             - \ref BYTE_ORD_BIG_ENDIAN
  *             - \ref BYTE_ORD_LITTLE_ENDIAN
  * @param[in]  byte_order
  *             - \ref BIT_ORD_BIG_ENDIAN
  *             - \ref BIT_ORD_LITTLE_ENDIAN
  * @retval     None
  * @details    This macro is used to set CRC16 seed value.
  */
#define CRC16_SET_ORDER(byte_order, bit_order)     { CRC16->CTL = (CRC16->CTL & ~(CRC16_CTL_BYTEORD_Msk | CRC16_CTL_BITORD_Msk)) | (byte_order) | (bit_order); }


/**
  * @brief      Set CRC16 DIN length
  * @param[in]  din_len     data in length
  * @retval     None
  * @details    This macro is used to set CRC16 DIN length.
  */
#define CRC16_SET_DIN_LEN(din_len)     { CRC16->CTL = (CRC16->CTL & ~ CRC16_CTL_CNT_Msk) | ((din_len) << CRC16_CTL_CNT_Pos); }


/**
  * @brief      Write CRC16 data in value
  * @param[in]  u32Din   32 bits Data in value
  * @retval     None
  * @details    This macro is used to set CRC16 data in value.
  */
#define CRC16_WRITE_DATA(u32Din)     { CRC16->DIN = (u32Din); }


/**
  * @brief      Get CRC16 result value
  * @param[in]  u16Seed   16 bits Seed value
  * @retval     None
  * @details    This macro is used to set CRC16 seed value.
  */
#define CRC16_GET_RESULT     ( CRC16->OUT & CRC16_OUT_RESULT_Msk )


/*@}*/ /* end of group NM1230_CRC16_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_CRC16_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __CRC16_H__ */


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
