/**************************************************************************//**
 * @file     pdma.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/06/14 10:23a $
 * @brief    I94100 series PDMA driver header file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PDMA_H__
#define __PDMA_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_PDMA_Driver PDMA Driver
  @{
*/

/** @addtogroup I94100_PDMA_EXPORTED_CONSTANTS PDMA Exported Constants
  @{
*/

#define PDMA_CH_MAX    16   /*!< Specify Maximum Channels of PDMA  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Define PDMA Channel Mask			                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_CH0_MASK	(BIT0)						/*!<DMA Channel0 Mask Bit  \hideinitializer */
#define PDMA_CH1_MASK	(BIT1)						/*!<DMA Channel1 Mask Bit  \hideinitializer */
#define PDMA_CH2_MASK	(BIT2)						/*!<DMA Channel2 Mask Bit  \hideinitializer */
#define PDMA_CH3_MASK	(BIT3)						/*!<DMA Channel3 Mask Bit  \hideinitializer */
#define PDMA_CH4_MASK	(BIT4)						/*!<DMA Channel4 Mask Bit  \hideinitializer */
#define PDMA_CH5_MASK	(BIT5)						/*!<DMA Channel5 Mask Bit  \hideinitializer */
#define PDMA_CH6_MASK	(BIT6)						/*!<DMA Channel6 Mask Bit  \hideinitializer */
#define PDMA_CH7_MASK	(BIT7)						/*!<DMA Channel7 Mask Bit  \hideinitializer */
#define PDMA_CH8_MASK	(BIT8)						/*!<DMA Channel8 Mask Bit  \hideinitializer */
#define PDMA_CH9_MASK	(BIT9)						/*!<DMA Channel9 Mask Bit  \hideinitializer */
#define PDMA_CH10_MASK	(BIT10)						/*!<DMA Channel10 Mask Bit  \hideinitializer */
#define PDMA_CH11_MASK	(BIT11)						/*!<DMA Channel11 Mask Bit  \hideinitializer */ 
#define PDMA_CH12_MASK	(BIT12)						/*!<DMA Channel12 Mask Bit  \hideinitializer */ 
#define PDMA_CH13_MASK	(BIT13)						/*!<DMA Channel13 Mask Bit  \hideinitializer */ 
#define PDMA_CH14_MASK	(BIT14)						/*!<DMA Channel14 Mask Bit  \hideinitializer */ 
#define PDMA_CH15_MASK	(BIT15)						/*!<DMA Channel15 Mask Bit  \hideinitializer */ 

/*---------------------------------------------------------------------------------------------------------*/
/*  Operation Mode Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_OP_STOP        0x00000000UL            /*!<DMA Stop Mode  \hideinitializer */
#define PDMA_OP_BASIC       0x00000001UL            /*!<DMA Basic Mode  \hideinitializer */
#define PDMA_OP_SCATTER     0x00000002UL            /*!<DMA Scatter-gather Mode  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Data Width Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_WIDTH_8        0x00000000UL            /*!<DMA Transfer Width 8-bit  \hideinitializer */
#define PDMA_WIDTH_16       0x00001000UL            /*!<DMA Transfer Width 16-bit  \hideinitializer */
#define PDMA_WIDTH_32       0x00002000UL            /*!<DMA Transfer Width 32-bit  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Address Attribute Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SAR_INC        0x00000000UL            /*!<DMA SAR increment  \hideinitializer */
#define PDMA_SAR_FIX        0x00000300UL            /*!<DMA SAR fix address  \hideinitializer */
#define PDMA_DAR_INC        0x00000000UL            /*!<DMA DAR increment  \hideinitializer */
#define PDMA_DAR_FIX        0x00000C00UL            /*!<DMA DAR fix address  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Burst Mode Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_REQ_SINGLE     0x00000004UL            /*!<DMA Single Request  \hideinitializer */
#define PDMA_REQ_BURST      0x00000000UL            /*!<DMA Burst Request  \hideinitializer */

#define PDMA_BURST_128      0x00000000UL            /*!<DMA Burst 128 Transfers  \hideinitializer */
#define PDMA_BURST_64       0x00000010UL            /*!<DMA Burst 64 Transfers  \hideinitializer */
#define PDMA_BURST_32       0x00000020UL            /*!<DMA Burst 32 Transfers  \hideinitializer */
#define PDMA_BURST_16       0x00000030UL            /*!<DMA Burst 16 Transfers  \hideinitializer */
#define PDMA_BURST_8        0x00000040UL            /*!<DMA Burst 8 Transfers  \hideinitializer */
#define PDMA_BURST_4        0x00000050UL            /*!<DMA Burst 4 Transfers  \hideinitializer */
#define PDMA_BURST_2        0x00000060UL            /*!<DMA Burst 2 Transfers  \hideinitializer */
#define PDMA_BURST_1        0x00000070UL            /*!<DMA Burst 1 Transfers  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Peripheral Transfer Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_MEM           0                       /*!<DMA Connect to Memory  \hideinitializer */
#define PDMA_USB_TX        2          			   /*!<DMA Connect to USB TX  \hideinitializer */
#define PDMA_USB_RX        3			           /*!<DMA Connect to USB RX  \hideinitializer */
#define PDMA_UART0_TX      4          			   /*!<DMA Connect to UART0 TX  \hideinitializer */
#define PDMA_UART0_RX      5			           /*!<DMA Connect to UART0 RX  \hideinitializer */
#define PDMA_SPI0_TX       20                      /*!<DMA Connect to SPI0 TX  \hideinitializer */
#define PDMA_SPI0_RX       21                      /*!<DMA Connect to SPI0 RX  \hideinitializer */
#define PDMA_SPI1_TX       22                      /*!<DMA Connect to SPI1 TX  \hideinitializer */
#define PDMA_SPI1_RX       23                      /*!<DMA Connect to SPI1 RX  \hideinitializer */
#define PDMA_SPI2_TX       24                      /*!<DMA Connect to SPI2 TX  \hideinitializer */
#define PDMA_SPI2_RX       25                      /*!<DMA Connect to SPI2 RX  \hideinitializer */
#define PDMA_DMIC_RX       27					   /*!<DMA Connect to DMIC RX  \hideinitializer */
#define PDMA_DPWM_TX	   28					   /*!<DMA Connect to DPWM TX  \hideinitializer */
#define PDMA_PWM0_P1_RX    32           		   /*!<DMA Connect to PWM0 P1 RX  \hideinitializer */
#define PDMA_PWM0_P2_RX    33           		   /*!<DMA Connect to PWM0 P2 RX  \hideinitializer */
#define PDMA_PWM0_P3_RX    34         			   /*!<DMA Connect to PWM0 P3 RX  \hideinitializer */
#define PDMA_I2C0_TX       38		               /*!<DMA Connect to I2C0 TX  \hideinitializer */
#define PDMA_I2C0_RX       39		               /*!<DMA Connect to I2C0 RX  \hideinitializer */
#define PDMA_I2C1_TX       40		               /*!<DMA Connect to I2C1 TX  \hideinitializer */
#define PDMA_I2C1_RX       41		               /*!<DMA Connect to I2C1 RX  \hideinitializer */
#define PDMA_I2S0_TX       44		               /*!<DMA Connect to I2S0 TX  \hideinitializer */
#define PDMA_I2S0_RX       45		               /*!<DMA Connect to I2S0 RX  \hideinitializer */
#define PDMA_TMR0	       46		               /*!<DMA Transfer is triggered by Timer0 \hideinitializer */
#define PDMA_TMR1	       47		               /*!<DMA Transfer is triggered by Timer1 \hideinitializer */
#define PDMA_TMR2	       48		               /*!<DMA Transfer is triggered by Timer2 \hideinitializer */
#define PDMA_TMR3	       49		               /*!<DMA Transfer is triggered by Timer3 \hideinitializer */
#define PDMA_EADC_RX       50			           /*!<DMA Connect to EADC RX  \hideinitializer */
#define PDMA_PWM0_CH0_TX   53		        	   /*!<DMA Connect to PWM0 CH0 TX  \hideinitializer */
#define PDMA_PWM0_CH1_TX   54		        	   /*!<DMA Connect to PWM0 CH1 TX  \hideinitializer */
#define PDMA_PWM0_CH2_TX   55		        	   /*!<DMA Connect to PWM0 CH2 TX  \hideinitializer */
#define PDMA_PWM0_CH3_TX   56		        	   /*!<DMA Connect to PWM0 CH3 TX  \hideinitializer */
#define PDMA_PWM0_CH4_TX   57		        	   /*!<DMA Connect to PWM0 CH4 TX  \hideinitializer */
#define PDMA_PWM0_CH5_TX   58		        	   /*!<DMA Connect to PWM0 CH5 TX  \hideinitializer */

#define PDMA_DPWM		   PDMA_DPWM_TX            // Backward compatible.

/*---------------------------------------------------------------------------------------------------------*/
/*  Interrupt Type Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_INT_TRANS_DONE 0x00000001UL            /*!<Transfer Done Interrupt  \hideinitializer */
#define PDMA_INT_TIMEOUT    0x00000002UL            /*!<Timeout Interrupt  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMA Status Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_STATUS_ABTIF 			(PDMA_INTSTS_ABTIF_Msk)            	/*!<Abortion Flag  \hideinitializer */
#define PDMA_STATUS_TDIF    		(PDMA_INTSTS_TDIF_Msk)            	/*!<Transfer Done Flag  \hideinitializer */
#define PDMA_STATUS_ALIGNF    	(PDMA_INTSTS_ALIGNF_Msk)            /*!<Transfer Alignment Flag  \hideinitializer */
#define PDMA_STATUS_REQTOF0    	(PDMA_INTSTS_REQTOF0_Msk)           /*!<Request Time-out Channel-0 Flag  \hideinitializer */
#define PDMA_STATUS_REQTOF1    	(PDMA_INTSTS_REQTOF1_Msk)           /*!<Request Time-out Channel-1 Flag  \hideinitializer */

/*@}*/ /* end of group I94100_PDMA_EXPORTED_CONSTANTS */

/** @addtogroup I94100_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
  * @brief      Enable PDMA channel
  * @param      u32Mask channel mask to enable 
  * @return     None
  */
#define PDMA_ENABLE_CHANNEL(u32Mask)                (PDMA->CHCTL |= u32Mask)

/**
  * @brief      Disable PDMA channel
  * @param      u32Mask channel mask to disable
  * @return     None
  */
#define PDMA_DISABLE_CHANNEL(u32Mask)                (PDMA->CHCTL &= ~u32Mask)

/**
 * @brief       Get PDMA Interrupt Status
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This macro gets the interrupt status.
 */
#define PDMA_GET_INT_STATUS() ((uint32_t)(PDMA->INTSTS))

/**
 * @brief       Get Transfer Done Interrupt Status
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Get the transfer done Interrupt status.
 */
#define PDMA_GET_TD_STS() ((uint32_t)(PDMA->TDSTS))

/**
 * @brief       Clear Transfer Done Interrupt Status
 *
 * @param[in]   u32Mask     The channel mask: PDMA_CH0_MASK~PDMA_CH15_MASK
 *
 * @return      None
 *
 * @details     Clear the transfer done Interrupt status.
 */
#define PDMA_CLR_TD_FLAG(u32Mask) ((uint32_t)(PDMA->TDSTS = (u32Mask)))

/**
 * @brief       Get Target Abort Interrupt Status
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Get the target abort Interrupt status.
 */
#define PDMA_GET_ABORT_STS() ((uint32_t)(PDMA->ABTSTS))

/**
 * @brief       Clear Target Abort Interrupt Status
 *
 * @param[in]   u32Mask     The channel mask: PDMA_CH0_MASK~PDMA_CH15_MASK
 *
 * @return      None
 *
 * @details     Clear the target abort Interrupt status.
 */
#define PDMA_CLR_ABORT_FLAG(u32Mask) ((uint32_t)(PDMA->ABTSTS = (u32Mask)))

/**
 * @brief       Clear Timeout Interrupt Status
 *
 * @param[in]   u32Ch     Only support 0(CH0), 1(CH1)
 *
 * @return      None
 *
 * @details     Clear the selected channel timeout interrupt status.
 * @note        This function is only supported in M45xD/M45xC.
 */
#define PDMA_CLR_TMOUT_FLAG(u32Ch) ((uint32_t)(PDMA->INTSTS = (1 << ((u32Ch) + 8))))

/**
 * @brief       Check Channel Status
 *
 * @param[in]   u32Ch     The selected channel: 0~15
 *
 * @retval      0 Idle state
 * @retval      1 Busy state
 *
 * @details     Check the selected channel is busy or not.
 */
#define PDMA_IS_CH_BUSY(u32Ch) ((uint32_t)(PDMA->TRGSTS & (1 << (u32Ch)))? 1 : 0)

/**
 * @brief       Set Source Address
 *
 * @param[in]   u32Ch     The selected channel: 0~15
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel source address.
 */
#define PDMA_SET_SRC_ADDR(u32Ch, u32Addr) ((uint32_t)(PDMA->DSCT[(u32Ch)].SA = (u32Addr)))


/**
 * @brief       Set Destination Address
 *
 * @param[in]   u32Ch     The selected channel: 0~15
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel destination address.
 */
#define PDMA_SET_DST_ADDR(u32Ch, u32Addr) ((uint32_t)(PDMA->DSCT[(u32Ch)].DA = (u32Addr)))

/**
 * @brief       Set Transfer Count
 *
 * @param[in]   u32Ch          The selected channel: 0~15
 * @param[in]   u32TransCount  Transfer Count
 *
 * @return      None
 *
 * @details     This macro set the selected channel transfer count.
 */
#define PDMA_SET_TRANS_CNT(u32Ch, u32TransCount) ((uint32_t)(PDMA->DSCT[(u32Ch)].CTL=(PDMA->DSCT[(u32Ch)].CTL&~PDMA_DSCT_CTL_TXCNT_Msk)|((u32TransCount-1) << PDMA_DSCT_CTL_TXCNT_Pos)))

/**
 * @brief       Set Scatter-gather descriptor Address
 *
 * @param[in]   u32Ch     The selected channel: 0~15
 * @param[in]   u32Addr   The descriptor address
 *
 * @return      None
 *
 * @details     This macro set the selected channel scatter-gather descriptor address.
 */
#define PDMA_SET_SCATTER_DESC(u32Ch, u32Addr) ((uint32_t)(PDMA->DSCT[(u32Ch)].NEXT = (u32Addr) - (PDMA->SCATBA)))

/**
 * @brief       Stop the channel
 *
 * @param[in]   u32Ch     The selected channel: 0~15
 *
 * @return      None
 *
 * @details     This macro stop the selected channel.
 */
#define PDMA_STOP(u32Ch) ((uint32_t)(PDMA->STOP = (1 << (u32Ch))))

/*---------------------------------------------------------------------------------------------------------*/
/* Define PWM functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetBurstType(uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void PDMA_EnableTimeout(uint32_t u32Mask);
void PDMA_DisableTimeout(uint32_t u32Mask);
void PDMA_SetTimeOut(uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);


/*@}*/ /* end of group I94100_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_PDMA_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__PDMA_H__

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
