/**************************************************************************//**
 * @file     ACMP.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/04 16:25p $
 * @brief    ISD9100 Series ACMP Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __ACMP_H__
#define __ACMP_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_ACMP_Driver ACMP Driver
  @{
*/

/** @addtogroup ISD9100_ACMP_EXPORTED_CONSTANTS ACMP Exported Constants
  @{
*/  

/*---------------------------------------------------------------------------------------------------------*/
/* ACMP CTL0 And CTL1 Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#define ACMP_CMP0VNEG_VBG    (0x0ul << ACMP_CTL0_NEGSEL_Pos)       /*!<CMP0 Negative Input Selects VBG 1.2V */
#define ACMP_CMP0VNEG_VMID   (0x1ul << ACMP_CTL0_NEGSEL_Pos)       /*!<CMP0 Negative Input Selects VMID  */

#define ACMP_CMP1VNEG_GPB7   (0x0ul << ACMP_CTL1_NEGSEL_Pos)       /*!<CMP1 Negative Input Selects GPIOB[7] */
#define ACMP_CMP1VNEG_VBG    (0x1ul << ACMP_CTL1_NEGSEL_Pos)       /*!<CMP1 Negative Input Selects VBG 1.2V */

#define ACMP_CH0_POSPIN_GPB0  (0)          /*!<CMP0 Positive Input Selects GPB0 */  
#define ACMP_CH0_POSPIN_GPB1  (1)          /*!<CMP0 Positive Input Selects GPB1 */ 
#define ACMP_CH0_POSPIN_GPB2  (2)          /*!<CMP0 Positive Input Selects GPB2 */ 
#define ACMP_CH0_POSPIN_GPB3  (3)          /*!<CMP0 Positive Input Selects GPB3 */ 
#define ACMP_CH0_POSPIN_GPB4  (4)          /*!<CMP0 Positive Input Selects GPB4 */ 
#define ACMP_CH0_POSPIN_GPB5  (5)          /*!<CMP0 Positive Input Selects GPB5 */ 
#define ACMP_CH0_POSPIN_GPB6  (6)          /*!<CMP0 Positive Input Selects GPB6 */ 
#define ACMP_CH0_POSPIN_GPB7  (7)          /*!<CMP0 Positive Input Selects GPB7 */ 

/*@}*/ /* end of group ISD9100_ACMP_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/**
  * @brief     This macro is used to select ACMP negative input source
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @param[in] Src is comparator negative input selection source
  *            - \ref ACMP_CMP0VNEG_VBG
  *            - \ref ACMP_CMP0VNEG_VMID
  *            - \ref ACMP_CMP1VNEG_GPB7
  *            - \ref ACMP_CMP1VNEG_VBG
  * @note      VMID block must be powered up if using VMID, programmer can 
  *            call ADC_ENABLE_VMID macro to enable VMID.
  */
#define ACMP_SET_NEG_SRC(acmp, Ch, Src)  (acmp->CTL##Ch |= Src)

/**
  * @brief     This macro is used to enable interrupt
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return    None
  */
#define ACMP_ENABLE_INT(acmp, Ch) (acmp->CTL##Ch |= ACMP_CTL##Ch##_NEGSEL_Msk)

/**
  * @brief     This macro is used to disable interrupt
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return None
  */
#define ACMP_DISABLE_INT(acmp, Ch) (acmp->CTL##Ch &= ~ACMP_CTL##Ch##_NEGSEL_Msk)

/**
  * @brief     This macro is used to enable ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return None
  */
#define ACMP_ENABLE(acmp, Ch) (acmp->CTL##Ch |= ACMP_CTL##Ch##_ACMPEN_Msk)

/**
  * @brief     This macro is used to disable ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return None
  */
#define ACMP_DISABLE(acmp, Ch) (acmp->CTL##Ch &= ~ACMP_CTL##Ch##_ACMPEN_Msk)

/**
  * @brief     This macro is used to get ACMP output value
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return    1 or 0
  */
#define ACMP_GET_OUTPUT(acmp, Ch) ((acmp->STATUS & ACMP_STATUS_ACMPO##Ch_Msk)?1:0)

/**
  * @brief     This macro is used to get ACMP interrupt flag
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return    ACMP interrupt occurred or not
  */
#define ACMP_GET_INT_FLAG(acmp, Ch) ((acmp->STATUS & ACMP_STATUS_ACMPIF##Ch_Msk)?1:0)

/**
  * @brief     This macro is used to clear ACMP interrupt flag
  * @param[in] acmp The base address of ACMP module
  * @param[in] Ch is ACMP number, could 0 or 1.
  * @return    None
  */
#define ACMP_CLR_INT_FLAG(acmp, Ch) (acmp->STATUS |= ACMP_STATUS_ACMPIF##Ch_Msk)

/**
  * @brief     This macro is used to select the V+ pin of ACMP only for CMP0
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32Pin CMP0 positive input pin
  *            - \ref ACMP_CH0_POSPIN_GPB0
  *            - \ref ACMP_CH0_POSPIN_GPB1
  *            - \ref ACMP_CH0_POSPIN_GPB2
  *            - \ref ACMP_CH0_POSPIN_GPB3
  *            - \ref ACMP_CH0_POSPIN_GPB4
  *            - \ref ACMP_CH0_POSPIN_GPB5
  *            - \ref ACMP_CH0_POSPIN_GPB6
  *            - \ref ACMP_CH0_POSPIN_GPB7
  * @return    None
  * @note      Multi-function pin needs to be configured and set as input mode        
  */
#define ACMP_CH0SELECT_P(acmp, u32Pin)  (acmp->POSSEL = u32Pin)

void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32PosPin);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);


/*@}*/ /* end of group ISD9100_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_ACMP_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ACMP_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

