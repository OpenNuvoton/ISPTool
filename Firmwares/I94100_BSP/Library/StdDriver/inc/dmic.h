/******************************************************************************
 * @file     DMIC.h
 * @version  V1.0
 * $Revision  1 $
 * $Date: 17/12/26 05:37p $
 * @brief    I94100 Series DMIC Header File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __DMIC_H__
#define __DMIC_H__

 #ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_DMIC_Driver DMIC Driver
  @{
*/

/** @addtogroup I94100_DMIC_EXPORTED_CONSTANTS DMIC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* DMIC CTL Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DMIC_CTL_CH0                  (DMIC_CTL_CHEN0_Msk)               /*!< DMIC Channel 0 */
#define DMIC_CTL_CH1                  (DMIC_CTL_CHEN1_Msk)               /*!< DMIC Channel 1 */
#define DMIC_CTL_CH2                  (DMIC_CTL_CHEN2_Msk)               /*!< DMIC Channel 2 */
#define DMIC_CTL_CH3                  (DMIC_CTL_CHEN3_Msk)               /*!< DMIC Channel 3 */

#define DMIC_CTL_DOWNSAMPLE_32        (0x0UL<<DMIC_CTL_OSR_Pos)          /*!< DMIC Down Sample Rate 32 */
#define DMIC_CTL_DOWNSAMPLE_64        (0x1UL<<DMIC_CTL_OSR_Pos)          /*!< DMIC Down Sample Rate 64 */
#define DMIC_CTL_DOWNSAMPLE_128       (0x2UL<<DMIC_CTL_OSR_Pos)          /*!< DMIC Down Sample Rate 128 */
#define DMIC_CTL_DOWNSAMPLE_256       (0x3UL<<DMIC_CTL_OSR_Pos)          /*!< DMIC Down Sample Rate 256 */
#define DMIC_CTL_DOWNSAMPLE_100_50    (0x4UL<<DMIC_CTL_OSR_Pos)          /*!< DMIC Down Sample Rate 100 or 50 */

#define DMIC_CTL_LATCHDATA_CH01F      (0x0UL<<DMIC_CTL_LCHEDGE01_Pos)    /*!< DMIC Channel 01 Data Latch Falling Edge */
#define DMIC_CTL_LATCHDATA_CH01R      (0x1UL<<DMIC_CTL_LCHEDGE01_Pos)    /*!< DMIC Channel 01 Data Latch Rising Edge  */
#define DMIC_CTL_LATCHDATA_CH23F      (0x0UL<<DMIC_CTL_LCHEDGE23_Pos)    /*!< DMIC Channel 23 Data Latch Falling Edge */
#define DMIC_CTL_LATCHDATA_CH23R      (0x1UL<<DMIC_CTL_LCHEDGE23_Pos)    /*!< DMIC Channel 23 Data Latch Rising Edge  */
	
/*@}*/ /* end of group I94100_DMIC_EXPORTED_CONSTANTS */


/** @addtogroup I94100_DMIC_EXPORTED_FUNCTIONS DMIC Exported Functions
  @{
*/

/**
  * @brief      Enable DMIC FIFO threshold interrupt.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u8Value: FIFO buffer threshold count
  * @return     None
  * @details    DMIC FIFO threshold interrupt Enabled.
  */
#define DMIC_ENABLE_FIFOTHRESHOLDINT(dmic,u8Value)    ((dmic)->DIV = (((dmic)->DIV&~DMIC_DIV_TH_Msk)|((((uint32_t)u8Value)<<DMIC_DIV_TH_Pos)&DMIC_DIV_TH_Msk))|DMIC_DIV_THIE_Msk)

/**
  * @brief      Disable DMIC FIFO threshold interrupt.
  * @param[in]  dmic The base address of DMIC module
  * @return     None
  * @details    DMIC FIFO threshold interrupt Disabled.			
  */
#define DMIC_DISABLE_FIFOTHRESHOLDINT(dmic)           ((dmic)->DIV &= (~DMIC_DIV_THIE_Msk))

/**
  * @brief      Set DMIC channel data latch edge.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Value Config Channle and edge state(falliing or rising)
  *             - \ref DMIC_CTL_LATCHDATA_CH01F									
  *             - \ref DMIC_CTL_LATCHDATA_CH01R
  *             - \ref DMIC_CTL_LATCHDATA_CH23F									
  *             - \ref DMIC_CTL_LATCHDATA_CH23R
  * @return     None
  * @details    Channel latched on rising or falling edge of DMIC_CLK.	
  */
#define DMIC_SET_LATCHDATA(dmic,u32Value)             ((dmic)->CTL = ((dmic)->CTL & ~(DMIC_CTL_LCHEDGE01_Msk|DMIC_CTL_LCHEDGE23_Msk))|u32Value)

/**
  * @brief      Enable DMIC down sample rate.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Value Down sample rate value.
  *             - \ref DMIC_CTL_DOWNSAMPLE_32									
  *             - \ref DMIC_CTL_DOWNSAMPLE_64
  *             - \ref DMIC_CTL_DOWNSAMPLE_128									
  *             - \ref DMIC_CTL_DOWNSAMPLE_256
  *             - \ref DMIC_CTL_DOWNSAMPLE_100_50 
  * @return     None
  * @details    Enable DMIC down sample rate funciton and set down sample value. 	
  */
#define DMIC_ENABLE_DOWMSAMPLE(dmic,u32Value)         ((dmic)->CTL = ((dmic)->CTL & ~DMIC_CTL_OSR_Msk)|u32Value)

/**
  * @brief      Disable DMIC down sample rate.
  * @param[in]  dmic The base address of DMIC module
  * @return     None
  * @details    Disable DMIC down sample rate funciton. 	
  */
#define DMIC_DISABLE_DOWMSAMPLE(dmic)                 ((dmic)->CTL |= DMIC_CTL_OSR_Msk)

/**
  * @brief      Enable DMIC's channel 
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Ch Enable channle.
  *             - \ref DMIC_CTL_CH0									
  *             - \ref DMIC_CTL_CH1
  *             - \ref DMIC_CTL_CH2									
  *             - \ref DMIC_CTL_CH3
  * @return     None
  * @details    Enable channle to start input data. 	
  */
#define DMIC_ENABLE_CHANNEL(dmic,u32Ch)               ((dmic)->CTL |= u32Ch)

/**
  * @brief      Disable DMIC's channel 
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Ch Enable channle.
  *             - \ref DMIC_CTL_CH0									
  *             - \ref DMIC_CTL_CH1
  *             - \ref DMIC_CTL_CH2									
  *             - \ref DMIC_CTL_CH3
  * @return     None
  * @details    Disable channle to start input data. 	
  */
#define DMIC_DISABLE_CHANNEL(dmic,u32Ch)              ((dmic)->CTL &= ~(u32Ch))

/**
  * @brief     	Enable DMIC PDMA function.
  * @param[in] 	dmic The base address of DMIC module
  * @return   	None.
  * @details   	DMIC will request data to PDMA controller whenever there is space in FIFO.
  */
#define DMIC_ENABLE_PDMA(dmic)                        ((dmic)->PDMACTL |= DMIC_PDMACTL_PDMAEN_Msk)

/**
  * @brief     	Disable DMIC PDMA function.
  * @param[in] 	dmic The base address of DMIC module
  * @return    	None.
  */
#define DMIC_DISABLE_PDMA(dmic)                       ((dmic)->PDMACTL &= ~DMIC_PDMACTL_PDMAEN_Msk)

/**
  * @brief      Check DMIC FIFO empty or not
  * @param[in]  dmic The base address of DMIC module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define DMIC_IS_FIFOEMPTY(dmic)                       ((dmic)->STATUS&DMIC_STATUS_EMPTY_Msk)

/**
  * @brief      Check DPWM FIFO full or not
  * @param[in]  dmic The base address of DMIC module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define DMIC_IS_FIFOFULL(dmic)                        ((dmic)->STATUS&DMIC_STATUS_FULL_Msk)

/**
  * @brief     	Read DMIC FIFO Audio Data Input.
  * @param[in]  dmic The base address of DMIC module
  * @return    	None.
  * @details   	A read function to this register pop data from the DMIC FIFO.
  */
#define DMIC_READ_DATA(dmic)                          ((dmic)->FIFO)

void DMIC_Open(DMIC_T *dmic);

void DMIC_Close(DMIC_T *dmic);

uint32_t DMIC_SetSampleRate(DMIC_T* dmic,uint32_t u32SampleRate);

/*@}*/ /* end of group I94100_DMIC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_DMIC_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DMIC_H__

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

