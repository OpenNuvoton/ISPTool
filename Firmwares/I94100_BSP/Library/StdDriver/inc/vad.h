/******************************************************************************
 * @file     VAD.h
 * @version  V1.0
 * $Revision  1 $
 * $Date: 17/12/28 05:37p $
 * @brief    I94100 Series VAD Header File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __VAD_H__
#define __VAD_H__

 #ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_VAD_Driver VAD Driver
  @{
*/

/** @addtogroup I94100_VAD_EXPORTED_CONSTANTS VAD Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* VAD SINCCTL Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define VAD_SINC_CTL_DOWNSAMPLE_48    (0x0UL<<VAD_SINC_CTL_SINCOSR_Pos)          /*!< VAD SINC Filter Down Sample Rate 48 */
#define VAD_SINC_CTL_DOWNSAMPLE_64    (0x1UL<<VAD_SINC_CTL_SINCOSR_Pos)          /*!< VAD SINC Filter Down Sample Rate 64 */
#define VAD_SINC_CTL_DOWNSAMPLE_96    (0x2UL<<VAD_SINC_CTL_SINCOSR_Pos)          /*!< VAD SINC Filter Down Sample Rate 96 */

/*---------------------------------------------------------------------------------------------------------*/
/* VAD CTL Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define VAD_CTL0_STAT_16MS            (0x99UL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 16 ms */
#define VAD_CTL0_STAT_8MS             (0xaaUL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 8 ms */
#define VAD_CTL0_STAT_4MS             (0xbbUL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 4 ms */
#define VAD_CTL0_STAT_2MS             (0xccUL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 2 ms */

#define VAD_CTL0_LTAT_512MS           (0x4UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 512 ms */
#define VAD_CTL0_LTAT_256MS           (0x5UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 256 ms */
#define VAD_CTL0_LTAT_128MS           (0x6UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 128 ms */
#define VAD_CTL0_LTAT_64MS            (0x7UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 64 ms */

/*---------------------------------------------------------------------------------------------------------*/
/* VAD Biquad Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define VAD_COEFF_B0                  (0)                                        /*!< VAD Biquad coefficient b0		*/
#define VAD_COEFF_B1                  (1)                                        /*!< VAD Biquad coefficient b1		*/
#define VAD_COEFF_B2                  (2)                                        /*!< VAD Biquad coefficient b2		*/
#define VAD_COEFF_A1                  (3)                                        /*!< VAD Biquad coefficient a0		*/
#define VAD_COEFF_A2                  (4)                                        /*!< VAD Biquad coefficient a1		*/

/*---------------------------------------------------------------------------------------------------------*/
/* VAD Power Threshold Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define VAD_POWERTHRE_0DB             (0x7FFFUL)                                 /*!< VAD power threshold 0DB   	*/
#define VAD_POWERTHRE_10DB            (0x2879UL)                                 /*!< VAD power threshold 10DB  	*/
#define VAD_POWERTHRE_20DB            (0x0CCCUL)                                 /*!< VAD power threshold 20DB   	*/
#define VAD_POWERTHRE_30DB            (0x040CUL)                                 /*!< VAD power threshold 30DB   	*/
#define VAD_POWERTHRE_40DB            (0x0147UL)                                 /*!< VAD power threshold 40DB   	*/
#define VAD_POWERTHRE_50DB            (0x0067UL)                                 /*!< VAD power threshold 50DB   	*/
#define VAD_POWERTHRE_60DB            (0x0020UL)                                 /*!< VAD power threshold 60DB   	*/
#define VAD_POWERTHRE_70DB            (0x000AUL)                                 /*!< VAD power threshold 70DB   	*/
#define VAD_POWERTHRE_80DB            (0x0003UL)                                 /*!< VAD power threshold 80DB   	*/
#define VAD_POWERTHRE_90DB            (0x0001UL)                                 /*!< VAD power threshold 90DB   	*/

/*@}*/ /* end of group I94100_VAD_EXPORTED_CONSTANTS */


/** @addtogroup I94100_VAD_EXPORTED_FUNCTIONS VAD Exported Functions
  @{
*/

/**
  * @brief      Enable VAD down sample rate.
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Value Down sample rate value.
  *             - \ref VAD_SINC_CTL_DOWNSAMPLE_48									
  *             - \ref VAD_SINC_CTL_DOWNSAMPLE_64
  *             - \ref VAD_SINC_CTL_DOWNSAMPLE_96									
  * @return     None
  * @details    Enable VAD down sample rate funciton and set down sample value. 	
  */
#define VAD_ENABLE_DOWMSAMPLE(vad,u32Value)         ((vad)->SINC_CTL = ((vad)->SINC_CTL & ~VAD_SINC_CTL_SINCOSR_Msk)|u32Value)

/**
  * @brief      Disable VAD down sample rate.
  * @param[in]  vad The base address of VAD module
  * @return     None
  * @details    Disable VAD down sample rate funciton. 	
  */
#define VAD_DISABLE_DOWMSAMPLE(vad)                 ((vad)->SINC_CTL |= VAD_SINC_CTL_SINCOSR_Msk)

/**
  * @brief      Enable VAD function. 
  * @param[in]  vad The base address of VAD module
  * @return     None
  * @details    Start to detect voice from DMIC0 	
  */
#define VAD_ENABLE(vad)                             ((vad)->SINC_CTL |= VAD_SINC_CTL_VADEN_Msk)

/**
  * @brief      Disable VAD function.
  * @param[in]  vad The base address of VAD module
  * @return     None
  * @details    Stop to detect voice.		
  */
#define VAD_DISABLE(vad)                            ((vad)->SINC_CTL &= ~VAD_SINC_CTL_VADEN_Msk)

/**
  * @brief      Set Short Term Attack Time
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Time Short Term Attack Time.
  *             - \ref VAD_CTL0_STAT_16MS									
  *             - \ref VAD_CTL0_STAT_8MS
  *             - \ref VAD_CTL0_STAT_4MS
  *             - \ref VAD_CTL0_STAT_2MS  
  * @return     None
  */
#define VAD_SET_STAT(vad,u32Time)                   ((vad)->CTL0 = ((vad)->CTL0 & ~VAD_CTL0_STAT_Msk)|u32Time)

/**
  * @brief      Set Long Term Attack Time
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Time Long Term Attack Time.
  *             - \ref VAD_CTL0_LTAT_512MS									
  *             - \ref VAD_CTL0_LTAT_256MS
  *             - \ref VAD_CTL0_LTAT_128MS
  *             - \ref VAD_CTL0_LTAT_64MS  
  * @return     None
  */
#define VAD_SET_LTAT(vad,u32Time)                   ((vad)->CTL0 = ((vad)->CTL0 & ~VAD_CTL0_LTAT_Msk)|u32Time)

/**
  * @brief      Set Short Term Power Threshold.
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Lower Lower Limit Threshold
  * @param[in]  u32Upper Upper Limit Threshold 
  * @return     None
  */
#define VAD_SET_STTHRE(vad,u32Lower,u32Upper)       ((vad)->CTL1 = (u32Lower<<VAD_CTL1_STTHRELWM_Pos)|(u32Upper&VAD_CTL1_STTHREHWM_Msk))

/**
  * @brief      Set Deviation Threshold
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Value Deviation Threshold
  * @return     None
  */
#define VAD_SET_DEVTHRE(vad,u32Value)                ((vad)->CTL3 = ((vad)->CTL3&~VAD_CTL3_DEVTHRE_Msk)|(u32Value&VAD_CTL3_DEVTHRE_Msk))

/**
  * @brief     	Enable VAD's BIQ filter.
  * @param[in] 	vad The base address of VAD module
  * @return   	None.
  * @details   	Before you enable this function, please make sure your BIQ coefficient correct.
  */
#define VAD_ENABLE_BIQ(vad)                         ((vad)->BIQ_CTL2 |= VAD_BIQ_CTL2_BIQEN_Msk)

/**
  * @brief     	Disable VAD's BIQ filter.
  * @param[in] 	vad The base address of VAD module
  * @return    	None.
  */
#define VAD_DISABLE_BIQ(vad)                        ((vad)->BIQ_CTL2 &= ~VAD_BIQ_CTL2_BIQEN_Msk)

void VAD_Open(VAD_T *vad);

void VAD_Close(VAD_T *vad);

uint32_t VAD_SetSampleRate(VAD_T *vad, uint32_t u32SampleRate);

void VAD_WriteBIQCoeff(VAD_T *vad, uint8_t u8Coeff, uint32_t u32Value);

/*@}*/ /* end of group I94100_VAD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_VAD_Driver */

/*@}*/ /* end of group I94100_VAD_Driver */

#ifdef __cplusplus
}
#endif

#endif //__VAD_H__

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

