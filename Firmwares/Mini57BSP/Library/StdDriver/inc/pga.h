/**************************************************************************//**
 * @file     pga.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series PGA driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PGA_H__
#define __PGA_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_PGA_Driver PGA Driver
  @{
*/

/** @addtogroup Mini57_PGA_EXPORTED_CONSTANTS PGA Exported Constants
  @{
*/

#define PGA_GAIN_1  (7UL)   /*!< PGA Gain mask with gain level x1    */
#define PGA_GAIN_2  (0UL)   /*!< PGA Gain mask with gain level x2    */
#define PGA_GAIN_3  (1UL)   /*!< PGA Gain mask with gain level x3    */
#define PGA_GAIN_5  (2UL)   /*!< PGA Gain mask with gain level x5    */
#define PGA_GAIN_7  (3UL)   /*!< PGA Gain mask with gain level x7    */
#define PGA_GAIN_9  (4UL)   /*!< PGA Gain mask with gain level x9    */
#define PGA_GAIN_11 (5UL)   /*!< PGA Gain mask with gain level x11   */
#define PGA_GAIN_13 (6UL)   /*!< PGA Gain mask with gain level x13   */

/*@}*/ /* end of group Mini57_PGA_EXPORTED_CONSTANTS */


/** @addtogroup Mini57_PGA_EXPORTED_FUNCTIONS PGA Exported Functions
  @{
*/

/**
  * @brief      Set PGA gain selection
  * @param[in]  pga     The pointer of the specified PGA module
  * @param[in]  u32Gain is gain index. Including :
  *             - \ref PGA_GAIN_1
  *             - \ref PGA_GAIN_2
  *             - \ref PGA_GAIN_3
  *             - \ref PGA_GAIN_5
  *             - \ref PGA_GAIN_7
  *             - \ref PGA_GAIN_9
  *             - \ref PGA_GAIN_11
  *             - \ref PGA_GAIN_13
  * @return     None
  * @details    Set PGA gain selection
  */
#define PGA_SET_GAIN(pga, u32Gain)      {\
            if ((u32Gain) == PGA_GAIN_1) { (pga)->CTL &= (~PGA_CTL_PGAEN_Msk); } else { (pga)->CTL |= PGA_CTL_PGAEN_Msk; } \
            (pga)->CTL = ((pga)->CTL & ~PGA_CTL_GAIN_Msk) | ((u32Gain) << PGA_CTL_GAIN_Pos); \
            }

/**
  * @brief      Get PGA gain selection bit definition.
  * @param[in]  pga     The pointer of the specified PGA module
  * @return     PGA gain selection bit definition
  * @details    Get PGA gain selection bit definition
  */
#define PGA_GET_GAIN(pga)               (((pga)->CTL & PGA_CTL_GAIN_Msk) >> PGA_CTL_GAIN_Pos)

/**
  * @brief      Enable PGA output to PGA_O pin
  * @param[in]  pga     The pointer of the specified PGA module
  * @return     None
  * @details    This macro enable PGA output to PGA_O pin.
  */
#define PGA_ENABLE_OUTPUT_PGAO(pga)     (SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC3MFP_Msk)) | (SYS_GPC_MFP_PC3_PGA_O))

/**
  * @brief      Disable PGA output to PGA_O pin
  * @param[in]  pga     The pointer of the specified PGA module
  * @return     None
  * @details    This macro disable PGA output to PGA_O pin.
  */
#define PGA_DISABLE_OUTPUT_PGAO(pga)    (SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC3MFP_Msk)) | (SYS_GPC_MFP_PC3_GPIO))

void PGA_Open(PGA_T *pga, uint32_t u32Gain, uint32_t u32OutputMask);
void PGA_Close(PGA_T *pga);

/*@}*/ /* end of group Mini57_PGA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_PGA_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __PGA_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
