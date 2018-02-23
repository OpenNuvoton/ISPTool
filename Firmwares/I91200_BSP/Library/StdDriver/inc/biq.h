/**************************************************************************//**
 * @file     BIQ.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 17/08/01 11:27a $
 * @brief    I91200 Series BIQ Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __BIQ_H__
#define __BIQ_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_BIQ_Driver BIQ Driver
  @{
*/

/** @addtogroup I91200_BIQ_EXPORTED_CONSTANTS BIQ Exported Constants
  @{
*/  

/*---------------------------------------------------------------------------------------------------------*/
/* BIQ CTL Constant Definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

#define BIQ_CTL_SDADCPATH      (0x0ul << BIQ_CTL_PATHSEL_Pos)     /*!< BIQ is used in SDADC path  */     
#define BIQ_CTL_DPWMPATH       (0x1ul << BIQ_CTL_PATHSEL_Pos)       /*!< BIQ is used in DPWM path  */ 

#define BIQ_CTL_DPWM_UPSAMPLE1X	(0x1ul << BIQ_CTL_DPWMPUSR_Pos)		/*!< DPWM Path Sample Rate: up 1x */ 
#define BIQ_CTL_DPWM_UPSAMPLE2X	(0x2ul << BIQ_CTL_DPWMPUSR_Pos)		/*!< DPWM Path Sample Rate: up 2x */ 
#define BIQ_CTL_DPWM_UPSAMPLE3X	(0x3ul << BIQ_CTL_DPWMPUSR_Pos)		/*!< DPWM Path Sample Rate: up 3x */ 
#define BIQ_CTL_DPWM_UPSAMPLE4X	(0x4ul << BIQ_CTL_DPWMPUSR_Pos)		/*!< DPWM Path Sample Rate: up 4x */ 
#define BIQ_CTL_DPWM_UPSAMPLE6X	(0x6ul << BIQ_CTL_DPWMPUSR_Pos)		/*!< DPWM Path Sample Rate: up 6x */ 

#define BIQ_CTL_SDADC_DOWNSAMPLE1X	(0x1ul << BIQ_CTL_SDADCWNSR_Pos)		/*!< SDADC Path Sample Rate: down 1x */ 
#define BIQ_CTL_SDADC_DOWNSAMPLE2X	(0x2ul << BIQ_CTL_SDADCWNSR_Pos)		/*!< SDADC Path Sample Rate: down 2x */ 
#define BIQ_CTL_SDADC_DOWNSAMPLE3X	(0x3ul << BIQ_CTL_SDADCWNSR_Pos)		/*!< SDADC Path Sample Rate: down 3x */ 
#define BIQ_CTL_SDADC_DOWNSAMPLE4X	(0x4ul << BIQ_CTL_SDADCWNSR_Pos)		/*!< SDADC Path Sample Rate: down 4x */ 
#define BIQ_CTL_SDADC_DOWNSAMPLE6X	(0x6ul << BIQ_CTL_SDADCWNSR_Pos)		/*!< SDADC Path Sample Rate: down 6x */ 

#define BIQ_CTL_6STAGES_NUM		(0x0ul << BIQ_CTL_STAGE_Pos)		/*!< 6 stages BIQ*/ 
#define BIQ_CTL_5STAGES_NUM		(0x1ul << BIQ_CTL_STAGE_Pos)		/*!< 5 stages BIQ*/ 

/*@}*/ /* end of group I91200_BIQ_EXPORTED_CONSTANTS */

/** @addtogroup I91200_BIQ_EXPORTED_FUNCTIONS BIQ Exported Functions
  @{
*/

/**
  * @brief     Set BIQ in SDADC path.
  * @param     biq Base address of BIQ module.
  * @param     u32DownRatio is down sample ratio:
  *            - \ref BIQ_CTL_SDADC_DOWNSAMPLE1X
  *            - \ref BIQ_CTL_SDADC_DOWNSAMPLE2X
  *            - \ref BIQ_CTL_SDADC_DOWNSAMPLE3X
  *            - \ref BIQ_CTL_SDADC_DOWNSAMPLE4X
  *            - \ref BIQ_CTL_SDADC_DOWNSAMPLE6X
  * @return    None
  */
#define BIQ_SET_SDADCPATH(biq, \
						  u32DownRatio)    (biq->CTL = (biq->CTL&~(BIQ_CTL_SRDIV_Msk|BIQ_CTL_SDADCWNSR_Msk|BIQ_CTL_PATHSEL_Msk))|u32DownRatio)

/**
  * @brief     Set BIQ in DPWM path.
  * @param     biq Base address of BIQ module.
  * @param     u16SRDiv is sample rate divisor.
  * @param     u32UpRatio is up Sample ratio:
  *            - \ref BIQ_CTL_DPWM_UPSAMPLE1X
  *            - \ref BIQ_CTL_DPWM_UPSAMPLE2X
  *            - \ref BIQ_CTL_DPWM_UPSAMPLE3X
  *            - \ref BIQ_CTL_DPWM_UPSAMPLE4X
  *            - \ref BIQ_CTL_DPWM_UPSAMPLE6X
  * @return    None
  * @details   The DPWM sample rate is defined as (u16SRDiv+1)*HCLK/(u16SRDiv+1).
  *            Default value is 3 for u8UpRation, up sample x4.
  */
#define BIQ_SET_DPWMPATH(biq, \
                         u16SRDiv, \
                         u32UpRatio)     (biq->CTL = (biq->CTL&~(BIQ_CTL_SRDIV_Msk|BIQ_CTL_DPWMPUSR_Msk))|(BIQ_CTL_DPWMPATH|((u16SRDiv&BIQ_CTL_SRDIV_Msk) << BIQ_CTL_SRDIV_Pos)|u32UpRatio))

/**
  * @brief     Set BIQ Stage Number.
  * @param     biq Base address of BIQ module.
  * @param     u32Stages is BIQ stages:
  *            - \ref BIQ_CTL_6STAGES_NUM
  *            - \ref BIQ_CTL_5STAGES_NUM
  * @return    None
  */
#define BIQ_SET_STAGE_NUM(biq, \
						  u32Stages)    (biq->CTL = (biq->CTL&~BIQ_CTL_STAGE_Msk)| u32Stages)

/**
  * @brief     Enable high pass filter.
  * @param     biq Base address of BIQ module.
  * @return    None
  * @details   If high pass filter is enabled, BIQ only 5 stage left.
  *            SDADC path sixth stage coefficient is for HPF filter coefficient.
  *            DPWM path first stage coefficient is for HPF filter coefficient.
  */
#define BIQ_ENABLE_HIGHPASS_FILTER(biq)		(biq->CTL = (biq->CTL&~BIQ_CTL_STAGE_Msk)| BIQ_CTL_HPFON_Msk)

/**
  * @brief     Disable high pass filter.
  * @param     biq Base address of BIQ module.
  * @return    None
  */
#define BIQ_DISABLE_HIGHPASS_FILTER(biq) 	(biq->CTL &= ~BIQ_CTL_HPFON_Msk)

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

void BIQ_SetCoeff(uint32_t u32BiqCoeff[30]);
void BIQ_Reset(void);
void BIQ_LoadDefaultCoeff(void);

/*@}*/ /* end of group I91200_BIQ_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_BIQ_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__BIQ_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/    
