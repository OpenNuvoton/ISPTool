/**************************************************************************//**
 * @file     BIQ.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/04 11:27a $
 * @brief    ISD9100 Series BIQ Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __BIQ_H__
#define __BIQ_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_BIQ_Driver BIQ Driver
  @{
*/

/** @addtogroup ISD9100_BIQ_EXPORTED_CONSTANTS BIQ Exported Constants
  @{
*/  

/*---------------------------------------------------------------------------------------------------------*/
/* BIQ CTL Constant Definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

#define BIQ_CTL_ADCPATH        (0x0ul << BIQ_CTL_PATHSEL_Pos)       /*!< BIQ is used in ADC path  */     
#define BIQ_CTL_DPWMPATH       (0x1ul << BIQ_CTL_PATHSEL_Pos)       /*!< BIQ is used in DPWM path  */ 

/*@}*/ /* end of group ISD9100_BIQ_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_BIQ_EXPORTED_FUNCTIONS BIQ Exported Functions
  @{
*/

/**
  * @brief     Set BIQ in ADC path.
  * @param     biq Base address of BIQ module.
  * @param     u16SRDiv is sample rate divisor.
  * @param     u8DownRation is down sample ration
  * @return    None
  * @details   The sample rate is defined as (HCLK/(u16SRDiv+1).
  *            Default value is 3071, so the sampling rate is 16K when HCLK is 49.152MHz.
  */
#define BIQ_SET_ADCPATH(biq, \
                        u16SRDiv, \
						u8DownRation)        (biq->CTL = (biq->CTL&~(BIQ_CTL_SRDIV_Msk|BIQ_CTL_DPWMPUSR_Msk))|(BIQ_CTL_ADCPATH|((u16SRDiv&0x1fff) << BIQ_CTL_SRDIV_Pos)|((u8DownRation&0x7) << BIQ_CTL_DPWMPUSR_Pos)))

/**
  * @brief     Set BIQ in DPWM path.
  * @param     biq Base address of BIQ module.
  * @param     u16SRDiv is sample rate divisor.
  * @param     u8UpRation is up Sample ration.
  * @return    None
  * @details   The DPWM sample rate is defined as (u16SRDiv+1)*HCLK/(u16SRDiv+1).
  *            Default value is 3 for u8UpRation, up sample x4.
  */
#define BIQ_SET_DPWMPATH(biq, \
                         u16SRDiv, \
                         u8UpRation)     (biq->CTL = (biq->CTL&~(BIQ_CTL_SRDIV_Msk|BIQ_CTL_DPWMPUSR_Msk))|(BIQ_CTL_DPWMPATH|((u16SRDiv&0x1fff) << BIQ_CTL_SRDIV_Pos)|((u8UpRation&0x7) << BIQ_CTL_DPWMPUSR_Pos)))

/**
  * @brief     BIQ filter start to run.
  * @param     biq Base address of BIQ module.
  * @return    None
  */
#define BIQ_START_RUN(biq)      (biq->CTL |= BIQ_CTL_BIQEN_Msk)

/**
  * @brief     BIQ filter stop to run.
  * @param     biq Base address of BIQ module.
  * @return    None
  */
#define BIQ_STOP_RUN(biq)      (biq->CTL |= (~BIQ_CTL_BIQEN_Msk))

void BIQ_SetCoeff(uint32_t u32BiqCoeff[15]);
void BIQ_Reset(void);

/*@}*/ /* end of group ISD9100_BIQ_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_BIQ_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__BIQ_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/    
