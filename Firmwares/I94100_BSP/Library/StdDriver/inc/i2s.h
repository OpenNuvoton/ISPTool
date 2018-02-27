/****************************************************************************
 * @file     i2s.h
 * @version  V3.00
 * $Revision: 11 $
 * $Date: 17/08/24 04:26p $
 * @brief    I94100 series I2S driver header file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __I2S_H__
#define __I2S_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Include related headers                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#include "Platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_I2S_Driver I2S Driver
  @{
*/

/** @addtogroup I94100_I2S_EXPORTED_CONSTANTS I2S Exported Constants
  @{
*/
/* I2S TDM Channel Number */
#define I2S_TDMCHNUM_2CH		(0 << I2S_CTL0_TDMCHNUM_Pos)		/*!< I2S TDM Channel Number is 2-Channel */
#define I2S_TDMCHNUM_4CH		(1 << I2S_CTL0_TDMCHNUM_Pos)		/*!< I2S TDM Channel Number is 2-Channel */
#define I2S_TDMCHNUM_6CH		(2 << I2S_CTL0_TDMCHNUM_Pos)		/*!< I2S TDM Channel Number is 2-Channel */
#define I2S_TDMCHNUM_8CH		(3 << I2S_CTL0_TDMCHNUM_Pos)		/*!< I2S TDM Channel Number is 2-Channel */

/* I2S Channel Width */
#define I2S_CHWIDTH_8			(0 << I2S_CTL0_CHWIDTH_Pos)			/*!< I2S Channel Width is 8-bit */
#define I2S_CHWIDTH_16			(1 << I2S_CTL0_CHWIDTH_Pos)			/*!< I2S Channel Width is 16-bit */
#define I2S_CHWIDTH_24			(2 << I2S_CTL0_CHWIDTH_Pos)			/*!< I2S Channel Width is 24-bit */
#define I2S_CHWIDTH_32			(3 << I2S_CTL0_CHWIDTH_Pos)			/*!< I2S Channel Width is 32-bit */

/* I2S PCM Synchronization Pulse Length Selection */
#define I2S_PCMSYNC_BCLK		(0 << I2S_CTL0_PCMSYNC_Pos)			/*!< I2S PCM Synchronization Pulse Length is one BCLK period. */
#define I2S_PCMSYNC_CHAN		(1 << I2S_CTL0_PCMSYNC_Pos)			/*!< I2S PCM Synchronization Pulse Length is one channel period. */

/* I2S Data Format */
#define I2S_FORMAT_I2S          (0<<I2S_CTL0_FORMAT_Pos)           	/*!< I2S data format */
#define I2S_FORMAT_MSB          (1<<I2S_CTL0_FORMAT_Pos)            /*!< MSB justified data format */
#define I2S_FORMAT_LSB          (2<<I2S_CTL0_FORMAT_Pos)            /*!< LSB justified data format */
#define I2S_FORMAT_PCM         	(4<<I2S_CTL0_FORMAT_Pos)            /*!< PCM standard mode data format */
#define I2S_FORMAT_PCMMSB       (5<<I2S_CTL0_FORMAT_Pos)            /*!< PCM MSB mode data format */
#define I2S_FORMAT_PCMLSB       (6<<I2S_CTL0_FORMAT_Pos)            /*!< PCM LSB mode B data format */

/* I2S Record Channel */
#define I2S_MONO_RX_RIGHT          0                                  	/*!< Record mono right channel */
#define I2S_MONO_RX_LEFT           I2S_CTL0_RXLCH_Msk                  /*!< Record mono left channel */

/* I2S Operation mode */
#define I2S_SLAVE               I2S_CTL0_SLAVE_Msk                  /*!< As slave mode */
#define I2S_MASTER              0                                  	/*!< As master mode */

/* I2S PCM Synchronization Pulse Length Selection */
#define I2S_ORDER_EVENHIGH		(0 << I2S_CTL0_ORDER_Pos)			/*!< I2S Even channel data at high byte in 8-bit/16-bit data width. */
#define I2S_ORDER_EVENLOW		(1 << I2S_CTL0_ORDER_Pos)			/*!< I2S Even channel data at low byte in 8-bit/16-bit data width. */

/* I2S Audio Format */
#define I2S_MONO                I2S_CTL0_MONO_Msk                   /*!< Monaural channel */
#define I2S_STEREO              0                                  	/*!< Stereo channel */

/* I2S Data Width */
#define I2S_DATABIT_8           (0 << I2S_CTL0_DATWIDTH_Pos)        /*!< I2S data width is 8-bit */
#define I2S_DATABIT_16          (1 << I2S_CTL0_DATWIDTH_Pos)        /*!< I2S data width is 16-bit */
#define I2S_DATABIT_24          (2 << I2S_CTL0_DATWIDTH_Pos)        /*!< I2S data width is 24-bit */
#define I2S_DATABIT_32          (3 << I2S_CTL0_DATWIDTH_Pos)        /*!< I2S data width is 32-bit */

/* I2S FIFO Read/Write Order in 16-bit Width of Peripheral Bus */
#define I2S_PB16ORD_LOW			(0 << I2S_CTL1_PB16ORD_Pos)			/*!< I2S Low 16-bit read/write access first. */
#define I2S_PB16ORD_High		(1 << I2S_CTL1_PB16ORD_Pos)			/*!< I2S High 16-bit read/write access first. */

/* I2S Peripheral Bus Data Width Selection */
#define	I2S_PBWIDTH_32			(0 << I2S_CTL1_PBWIDTH_Pos)			/*!< I2S Peripheral Bus 32 bits data width. */
#define I2S_PBWIDTH_16			(1 << I2S_CTL1_PBWIDTH_Pos)			/*!< I2S Peripheral Bus 16 bits data width. */

/* I2S TX FIFO Threshold */
#define I2S_FIFO_TX_LEVEL_WORD_0    0                              	/*!< TX threshold is 0 word */
#define I2S_FIFO_TX_LEVEL_WORD_1    (1 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 1 word */
#define I2S_FIFO_TX_LEVEL_WORD_2    (2 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 2 words */
#define I2S_FIFO_TX_LEVEL_WORD_3    (3 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 3 words */
#define I2S_FIFO_TX_LEVEL_WORD_4    (4 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 4 words */
#define I2S_FIFO_TX_LEVEL_WORD_5    (5 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 5 words */
#define I2S_FIFO_TX_LEVEL_WORD_6    (6 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 6 words */
#define I2S_FIFO_TX_LEVEL_WORD_7    (7 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 7 words */
#define I2S_FIFO_TX_LEVEL_WORD_8    (8 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 3 words */
#define I2S_FIFO_TX_LEVEL_WORD_9    (9 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 4 words */
#define I2S_FIFO_TX_LEVEL_WORD_10   (10 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 10 words */
#define I2S_FIFO_TX_LEVEL_WORD_11   (11 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 11 words */
#define I2S_FIFO_TX_LEVEL_WORD_12   (12 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 12 words */
#define I2S_FIFO_TX_LEVEL_WORD_13   (13 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 13 words */
#define I2S_FIFO_TX_LEVEL_WORD_14   (14 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 14 words */
#define I2S_FIFO_TX_LEVEL_WORD_15   (15 << I2S_CTL1_TXTH_Pos)    	/*!< TX threshold is 15 words */

/* I2S RX FIFO Threshold */
#define I2S_FIFO_RX_LEVEL_WORD_1    0                              	/*!< RX threshold is 1 word */
#define I2S_FIFO_RX_LEVEL_WORD_2    (1 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 2 words */
#define I2S_FIFO_RX_LEVEL_WORD_3    (2 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 3 words */
#define I2S_FIFO_RX_LEVEL_WORD_4    (3 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 4 words */
#define I2S_FIFO_RX_LEVEL_WORD_5    (4 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 5 words */
#define I2S_FIFO_RX_LEVEL_WORD_6    (5 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 6 words */
#define I2S_FIFO_RX_LEVEL_WORD_7    (6 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 7 words */
#define I2S_FIFO_RX_LEVEL_WORD_8    (7 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 8 words */
#define I2S_FIFO_RX_LEVEL_WORD_9    (8 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 9 words */
#define I2S_FIFO_RX_LEVEL_WORD_10   (9 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 10 words */
#define I2S_FIFO_RX_LEVEL_WORD_11   (10 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 11 words */
#define I2S_FIFO_RX_LEVEL_WORD_12   (11 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 12 words */
#define I2S_FIFO_RX_LEVEL_WORD_13   (12 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 13 words */
#define I2S_FIFO_RX_LEVEL_WORD_14   (13 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 14 words */
#define I2S_FIFO_RX_LEVEL_WORD_15   (14 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 15 words */
#define I2S_FIFO_RX_LEVEL_WORD_16   (15 << I2S_CTL1_RXTH_Pos)    	/*!< RX threshold is 16 words */

/* Channel Zero-Cross Detection Mask*/
#define I2S_CH7ZCD_MASK				(I2S_STATUS1_CH7ZCIF_Msk)		/*!< Channl 7 zero-cross detection mask */
#define I2S_CH6ZCD_MASK				(I2S_STATUS1_CH6ZCIF_Msk)		/*!< Channl 6 zero-cross detection mask */
#define I2S_CH5ZCD_MASK				(I2S_STATUS1_CH5ZCIF_Msk)		/*!< Channl 5 zero-cross detection mask */
#define I2S_CH4ZCD_MASK				(I2S_STATUS1_CH4ZCIF_Msk)		/*!< Channl 4 zero-cross detection mask */
#define I2S_CH3ZCD_MASK				(I2S_STATUS1_CH3ZCIF_Msk)		/*!< Channl 3 zero-cross detection mask */
#define I2S_CH2ZCD_MASK				(I2S_STATUS1_CH2ZCIF_Msk)		/*!< Channl 2 zero-cross detection mask */
#define I2S_CH1ZCD_MASK				(I2S_STATUS1_CH1ZCIF_Msk)		/*!< Channl 1 zero-cross detection mask */
#define I2S_CH0ZCD_MASK				(I2S_STATUS1_CH0ZCIF_Msk)		/*!< Channl 0 zero-cross detection mask */

/* I2S Interrupt Mask */
#define I2S_TXTH_INT_MASK  			(I2S_IEN_TXTHIEN_Msk)			/*!< TX FIFO threshold interrupt mask */
#define I2S_RXTH_INT_MASK       	(I2S_IEN_RXTHIEN_Msk)       	/*!< RX FIFO threshold interrupt mask */
#define I2S_RXOV_INT_MASK       	(I2S_IEN_RXOVFIEN_Msk)			/*!< RX FIFO overrun interrupt mask */
#define I2S_TXOV_INT_MASK       	(I2S_IEN_TXOVFIEN_Msk)			/*!< TX FIFO overrun interrupt mask */
#define I2S_TXUF_INT_MASK           (I2S_IEN_TXUDFIEN_Msk)         	/*!< TX FIFO underflow interrupt mask */
#define I2S_RXUF_INT_MASK           (I2S_IEN_RXUDFIEN_Msk)         	/*!< RX FIFO underflow interrupt mask */
#define I2S_CH0ZC_INT_MASK          (I2S_IEN_CH0ZCIEN_Msk)          /*!< Channel 0 zero cross interrupt mask */
#define I2S_CH1ZC_INT_MASK          (I2S_IEN_CH1ZCIEN_Msk)          /*!< Channel 1 zero cross interrupt mask */
#define I2S_CH2ZC_INT_MASK          (I2S_IEN_CH2ZCIEN_Msk)          /*!< Channel 2 zero cross interrupt mask */
#define I2S_CH3ZC_INT_MASK          (I2S_IEN_CH3ZCIEN_Msk)          /*!< Channel 3 zero cross interrupt mask */
#define I2S_CH4ZC_INT_MASK          (I2S_IEN_CH4ZCIEN_Msk)          /*!< Channel 4 zero cross interrupt mask */
#define I2S_CH5ZC_INT_MASK          (I2S_IEN_CH5ZCIEN_Msk)          /*!< Channel 5 zero cross interrupt mask */
#define I2S_CH6ZC_INT_MASK          (I2S_IEN_CH6ZCIEN_Msk)          /*!< Channel 6 zero cross interrupt mask */
#define I2S_CH7ZC_INT_MASK          (I2S_IEN_CH7ZCIEN_Msk)          /*!< Channel 7 zero cross interrupt mask */

/* I2S Interrupt Flag */
//#define I2S_ALL_INT_FLAG			(I2S_RXUD_INT_FLAG|I2S_RXOV_INT_FLAG|I2S_TXUD_INT_FLAG|I2S_TXOV_INT_FLAG)	/*!< I2S all interrupt flag */
#define I2S_I2SINT_INT_FLAG 		(I2S_STATUS0_I2SINT_Msk)		/*!< I2S interrupt flag */
#define I2S_RXINT_INT_FLAG       	(I2S_STATUS0_I2SRXINT_Msk)      /*!< I2S Receive Interrupt flag */
#define I2S_TXINT_INT_FLAG       	(I2S_STATUS0_I2STXINT_Msk)      /*!< I2S transmit Interrupt flag */
#define I2S_RXUD_INT_FLAG 			(I2S_STATUS0_RXUDIF_Msk)		/*!< Receive FIFO Underflow Interrupt Flag */
#define I2S_RXOV_INT_FLAG       	(I2S_STATUS0_RXOVIF_Msk)       	/*!< Receive FIFO Overrun Interrupt Flag */
#define I2S_RXTH_INT_FLAG       	(I2S_STATUS0_RXTHIF_Msk)       	/*!< Receive FIFO Threshold Interrupt Flag */
#define I2S_TXUD_INT_FLAG 			(I2S_STATUS0_TXUDIF_Msk)		/*!< Transmit FIFO Underflow Interrupt Flag */
#define I2S_TXOV_INT_FLAG       	(I2S_STATUS0_TXOVIF_Msk)       	/*!< Transmit FIFO Overrun Interrupt Flag */
#define I2S_TXTH_INT_FLAG       	(I2S_STATUS0_TXTHIF_Msk)       	/*!< Transmit FIFO Threshold Interrupt Flag */
#define I2S_CH0ZC_INT_FLAG       	(I2S_STATUS1_CH0ZCIF_Msk<<24)  	/*!< Channel0 Zero-cross Interrupt Flag */
#define I2S_CH1ZC_INT_FLAG       	(I2S_STATUS1_CH1ZCIF_Msk<<24)  	/*!< Channel1 Zero-cross Interrupt Flag */
#define I2S_CH2ZC_INT_FLAG       	(I2S_STATUS1_CH2ZCIF_Msk<<24)  	/*!< Channel2 Zero-cross Interrupt Flag */
#define I2S_CH3ZC_INT_FLAG       	(I2S_STATUS1_CH3ZCIF_Msk<<24)  	/*!< Channel3 Zero-cross Interrupt Flag */
#define I2S_CH4ZC_INT_FLAG       	(I2S_STATUS1_CH4ZCIF_Msk<<24)  	/*!< Channel4 Zero-cross Interrupt Flag */
#define I2S_CH5ZC_INT_FLAG       	(I2S_STATUS1_CH5ZCIF_Msk<<24)  	/*!< Channel5 Zero-cross Interrupt Flag */
#define I2S_CH6ZC_INT_FLAG       	(I2S_STATUS1_CH6ZCIF_Msk<<24)  	/*!< Channel6 Zero-cross Interrupt Flag */
#define I2S_CH7ZC_INT_FLAG       	(I2S_STATUS1_CH7ZCIF_Msk<<24)  	/*!< Channel7 Zero-cross Interrupt Flag */


/*@}*/ /* end of group I2S_EXPORTED_CONSTANTS */

/** @addtogroup I94100_I2S_EXPORTED_FUNCTIONS I2S Exported Functions
  @{
*/

/**
  * @brief  	Set I2S TDM channel number.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32ChNum The number of the TDM channel.
  *                - \ref I2S_TDMCHNUM_2CH
  *                - \ref I2S_TDMCHNUM_4CH
  *                - \ref I2S_TDMCHNUM_6CH
  *                - \ref I2S_TDMCHNUM_8CH
  * @return 	None
  * @detail		This bit fields are used to define the TDM channel number in one audio frame while PCM mode (FORMAT[2] = 1).
  */
#define I2S_SET_TDMCHNUM(i2s, u32ChNum)		((i2s)->CTL0 = (i2s)->CTL0&(~I2S_CTL0_TDMCHNUM_Msk) | u32ChNum)

/**
  * @brief  	Set I2S Channel Width.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Width The bit-width of the audio channel.
  *                - \ref I2S_CHWIDTH_8
  *                - \ref I2S_CHWIDTH_16
  *                - \ref I2S_CHWIDTH_24
  *                - \ref I2S_CHWIDTH_32
  * @return 	None
  * @detail		This bit fields are used to define the length of audio channel. 
  *				If CHWIDTH < DATWIDTH, the hardware will set the real channel length as the bit-width of audio data which is defined by DATWIDTH.
  */
#define I2S_SET_CHWIDTH(i2s, u32Width)		((i2s)->CTL0 = (i2s)->CTL0&(~I2S_CTL0_CHWIDTH_Msk) | u32Width)

/**
  * @brief  	Set I2S PCM Synchronization Pulse Length.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Period The pulse length of frame synchronization signal.
  *                - \ref I2S_PCMSYNC_BCLK
  *                - \ref I2S_PCMSYNC_CHAN
  * @return 	None
  * @detail		This bit field is used to select the high pulse length of frame synchronization signal in PCM protocol.
  */
#define	I2S_SET_PCMSYNC(i2s, u32Period)		((i2s)->CTL0 = (i2s)->CTL0 & (~I2S_CTL0_PCMSYNC_Msk) | u32Period)

/**
  * @brief  	This function sets the recording source channel when mono mode is used.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Ch left or right channel. Valid values are:
  *             - \ref I2S_MONO_RX_LEFT
  *             - \ref I2S_MONO_RX_RIGHT
  * @return 	None
  * @details 	This function selects the recording source channel of monaural mode.
  */
static __INLINE void I2S_SET_MONO_RX_CHANNEL(I2S_T *i2s, uint32_t u32Ch)
{
    u32Ch == I2S_MONO_RX_LEFT ?
    (i2s->CTL0 |= I2S_CTL0_RXLCH_Msk) :
    (i2s->CTL0 &= ~I2S_CTL0_RXLCH_Msk);
}

/**
  * @brief  	Enable I2S TX DMA function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will set TXPDMAEN bit of SPI_PDMACTL register to transmit data with PDMA.
  */
#define I2S_ENABLE_TXDMA(i2s)  				((i2s)->CTL0 |= I2S_CTL0_TXPDMAEN_Msk )

/**
  * @brief  	Disable I2S TX DMA function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear TXPDMAEN bit of SPI_PDMACTL register to disable TX DMA function.
  */
#define I2S_DISABLE_TXDMA(i2s) 				((i2s)->CTL0 &= ~I2S_CTL0_TXPDMAEN_Msk )

/**
  * @brief  	Enable I2S RX DMA function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will set RXPDMAEN bit of SPI_PDMACTL register to receive data with PDMA.
  */
#define I2S_ENABLE_RXDMA(i2s) 				((i2s)->CTL0 |= I2S_CTL0_RXPDMAEN_Msk )

/**
  * @brief  	Disable I2S RX DMA function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear RXPDMAEN bit of SPI_PDMACTL register to disable RX DMA function.
  */
#define I2S_DISABLE_RXDMA(i2s) 				((i2s)->CTL0 &= ~I2S_CTL0_RXPDMAEN_Msk )

/**
  * @brief  	Clear TX FIFO.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear TX FIFO. The internal TX FIFO pointer will be reset to FIFO start point.
  */
#define I2S_CLR_TX_FIFO(i2s) 				((i2s)->CTL0 |= I2S_CTL0_TXFBCLR_Msk )

/**
  * @brief  	Clear RX FIFO.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear RX FIFO. The internal RX FIFO pointer will be reset to FIFO start point.
  */
#define I2S_CLR_RX_FIFO(i2s) 				((i2s)->CTL0 |= I2S_CTL0_RXFBCLR_Msk )

/**
  * @brief  	Enable I2S Force Left Channel Zero Cross Data.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit set to 1, when channel (Ch0,Ch2,Ch4,Ch6) data sign bit changes or next shift data bits are all 0 
  *				then the channel ZCIF flag in I2S_STATUS1 register is set to 1 and channel data will force zero. 
  *				This function is only available in transmit operation.
  */
#define	I2S_ENABLE_FLZCD(i2s)				((i2s)->CTL0 |= I2S_CTL0_FLZCDEN_Msk)

/**
  * @brief  	Disable I2S Force Left Channel Zero Cross Data.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit set to 1, when channel (Ch0,Ch2,Ch4,Ch6) data sign bit changes or next shift data bits are all 0 
  *				then the channel ZCIF flag in I2S_STATUS1 register is set to 1 and channel data will force zero. 
  *				This function is only available in transmit operation.
  */
#define	I2S_DISABLE_FLZCD(i2s)				((i2s)->CTL0 |= I2S_CTL0_FLZCDEN_Msk)

/**
  * @brief  	Enable I2S Force Right Channel Zero Cross Data.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit set to 1, when channel (Ch1,Ch3,Ch5,Ch7) data sign bit changes or next shift data bits are all 0 
  *				then the channel ZCIF flag in I2S_STATUS1 register is set to 1 and channel data will force zero. 
  *				This function is only available in transmit operation.
  */
#define	I2S_ENABLE_FRZCD(i2s)				((i2s)->CTL0 |= I2S_CTL0_FRZCDEN_Msk)

/**
  * @brief  	Disable I2S Force Right Channel Zero Cross Data.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit set to 1, when channel (Ch1,Ch3,Ch5,Ch7) data sign bit changes or next shift data bits are all 0 
  *				then the channel ZCIF flag in I2S_STATUS1 register is set to 1 and channel data will force zero. 
  *				This function is only available in transmit operation.
  */
#define	I2S_DISABLE_FRZCD(i2s)				((i2s)->CTL0 |= I2S_CTL0_FRZCDEN_Msk)

/**
  * @brief  	Set I2S Stereo Data Order in FIFO.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Order Even channel data at high or low byte.
  *                - \ref I2S_ORDER_EVENHIGH
  *                - \ref I2S_ORDER_EVENLOW
  * @return 	None
  * @detail		In 8-bit/16-bit data width, this bit is used to select whether the even or odd channel data is stored in higher byte. 
  *				In 24-bit data width, this is used to select the left/right alignment method of audio data which is stored in data memory consisted of 32-bit FIFO entries.
  */
#define	I2S_SET_STEREOORDER(i2s, u32Order)	((i2s)->CTL0 = (i2s)->CTL0 & (~I2S_CTL0_ORDER_Msk) | u32Order)

/**
  * @brief  	Enable TX Mute function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will set MUTE bit of SPI_I2SCTL register to enable I2S TX mute function.
  */
#define I2S_ENABLE_TX_MUTE(i2s)  			((i2s)->CTL0 |= I2S_CTL0_MUTE_Msk )

/**
  * @brief  	Disable TX Mute function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear MUTE bit of SPI_I2SCTL register to disable I2S TX mute function.
  */
#define I2S_DISABLE_TX_MUTE(i2s) 			((i2s)->CTL0 &= ~I2S_CTL0_MUTE_Msk )

/**
  * @brief  	Enable I2S TX function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will set TXEN bit of SPI_I2SCTL register to enable I2S TX function.
  */
#define I2S_ENABLE_TX(i2s) 					((i2s)->CTL0 |= I2S_CTL0_TXEN_Msk )

/**
  * @brief  	Disable I2S TX function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear TXEN bit of SPI_I2SCTL register to disable I2S TX function.
  */
#define I2S_DISABLE_TX(i2s) 				((i2s)->CTL0 &= ~I2S_CTL0_TXEN_Msk )

/**
  * @brief  	Enable I2S RX function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will set RXEN bit of SPI_I2SCTL register to enable I2S RX function.
  */
#define I2S_ENABLE_RX(i2s) 					((i2s)->CTL0 |= I2S_CTL0_RXEN_Msk )

/**
  * @brief  	Disable I2S RX function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear RXEN bit of SPI_I2SCTL register to disable I2S RX function.
  */
#define I2S_DISABLE_RX(i2s) 				((i2s)->CTL0 &= ~I2S_CTL0_RXEN_Msk )

/**
  * @brief  	Enable I2S RX function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will set RXEN bit of SPI_I2SCTL register to enable I2S RX function.
  */
#define I2S_ENABLE(i2s) 					((i2s)->CTL0 |= I2S_CTL0_I2SEN_Msk )

/**
  * @brief  	Disable I2S RX function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	This macro will clear RXEN bit of SPI_I2SCTL register to disable I2S RX function.
  */
#define I2S_DISABLE(i2s) 					((i2s)->CTL0 &= ~I2S_CTL0_I2SEN_Msk )

/**
  * @brief 		Set FIFO Read/Write Order in 16-bit Width of Peripheral Bus.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Order The Transmit FIFO Threshold Level.
  *            	- \ref I2S_PB16ORD_LOW
  *            	- \ref I2S_PB16ORD_HIGH
  * @return 	None
  * @details 	When PBWIDTH = 1, the data FIFO will be increased or decreased by two peripheral bus access. 
  *				This bit is used to select the order of FIFO access operations to meet the 32-bit transmitting/receiving FIFO entries.
  * @note		This bit is available while PBWIDTH = 1.
  */
#define I2S_SET_PB16ORD(i2s, u32Order)		((i2s)->CTL1 = (i2s)->CTL1 & (~I2S_CTL1_PB16ORD_Msk) | u32Order)

/**
  * @brief 		Set Peripheral Bus Data Width.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Width The Transmit FIFO Threshold Level.
  *            	- \ref I2S_PBWIDTH_16
  *            	- \ref I2S_PBWIDTH_32
  * @return 	None
  * @details 	This bit is used to choice the available data width of APB bus. It must be set to 1 while PDMA function is enable and it is set to 16-bit transmission mode.
  * @note		If PBWIDTH=1, the low 16 bits of 32-bit data bus are available.
  * @note		If PBWIDTH=1, the transmitting FIFO level will be increased after two FIFO write operations.
  * @note		If PBWIDTH=1, the receiving FIFO level will be decreased after two FIFO read operations.
  */
#define I2S_SET_PBWIDTH(i2s, u32Width)		((i2s)->CTL1 = (i2s)->CTL1 & (~I2S_CTL1_PBWIDTH_Msk) | u32Width)

/**
  * @brief 		Set RX treshold.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Treshold The Transmit FIFO Threshold Level.
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_1
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_2
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_3
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_4
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_5
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_6
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_7
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_8
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_9
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_10
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_11
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_12
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_13
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_14
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_15
  *            	- \ref I2S_FIFO_RX_LEVEL_WORD_16
  * @return 	None
  * @details 	Note: If remain data word number in transmit FIFO is the same or less than threshold level then TXTHIF (I2S_STATUS0[18]) flag is set.
  */
#define I2S_SET_RXTH(i2s, u32TresholdMask)		((i2s)->CTL1 = (i2s)->CTL1 & (~I2S_CTL1_RXTH_Msk) | u32TresholdMask)

/**
  * @brief 		Set TX treshold.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Treshold The Receive FIFO Threshold Level.
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_0
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_1
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_2
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_3
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_4
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_5
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_6
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_7
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_8
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_9
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_10
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_11
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_12
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_13
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_14
  *            	- \ref I2S_FIFO_TX_LEVEL_WORD_15
  * @return 	None
  * @details 	When received data word number in receive buffer is greater than threshold level then RXTHIF (I2S_STATUS0[10]) flag is set.
  */
#define I2S_SET_TXTH(i2s, u32TresholdMask)		((i2s)->CTL1 = (i2s)->CTL1 & (~I2S_CTL1_TXTH_Msk) | u32TresholdMask)

/**
  * @brief 		Enable Channel Zero-cross Detection.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Mask The combination of the channel mask.
  *            	- \ref I2S_CH7ZCD_MASK
  *            	- \ref I2S_CH6ZCD_MASK
  *            	- \ref I2S_CH5ZCD_MASK
  *            	- \ref I2S_CH4ZCD_MASK
  *            	- \ref I2S_CH3ZCD_MASK
  *            	- \ref I2S_CH2ZCD_MASK
  *            	- \ref I2S_CH1ZCD_MASK
  *            	- \ref I2S_CH0ZCD_MASK
  * @return 	None
  * @details 	This bit is available while multi-channel PCM mode.
  */
#define I2S_ENABLE_CHZCD(i2s, u32Mask)			((i2s)->IEN |= u32Mask)

/**
  * @brief 		Enable interrupt function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Mask The combination of all related interrupt enable bits.
  *            	Each bit corresponds to a interrupt source. Valid values are listed below.
  *            	- \ref I2S_TXTH_INT_MASK
  *            	- \ref I2S_RXTH_INT_MASK
  *            	- \ref I2S_TXOV_INT_MASK
  *            	- \ref I2S_RXOV_INT_MASK
  *            	- \ref I2S_TXUF_INT_MASK
  *            	- \ref I2S_RXUF_INT_MASK
  *            	- \ref I2S_CH0ZC_INT_MASK
  *            	- \ref I2S_CH1ZC_INT_MASK
  *            	- \ref I2S_CH2ZC_INT_MASK
  *            	- \ref I2S_CH3ZC_INT_MASK
  *            	- \ref I2S_CH4ZC_INT_MASK
  *            	- \ref I2S_CH5ZC_INT_MASK
  *            	- \ref I2S_CH6ZC_INT_MASK
  *            	- \ref I2S_CH7ZC_INT_MASK
  * @return 	None
  * @details 	This function enables the interrupt according to the u32Mask parameter.
  */
#define I2S_ENABLE_INT(i2s, u32Mask)	((i2s)->IEN |= u32Mask)

/**
  * @brief 		Disable interrupt function.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Mask The combination of all related interrupt enable bits.
  *            	Each bit corresponds to a interrupt source. Valid values are listed below.
  *            	- \ref I2S_TXTH_INT_MASK
  *            	- \ref I2S_RXTH_INT_MASK
  *            	- \ref I2S_TXOV_INT_MASK
  *            	- \ref I2S_RXOV_INT_MASK
  *            	- \ref I2S_TXUF_INT_MASK
  *            	- \ref I2S_RXUF_INT_MASK
  *            	- \ref I2S_CH0ZC_INT_MASK
  *            	- \ref I2S_CH1ZC_INT_MASK
  *            	- \ref I2S_CH2ZC_INT_MASK
  *            	- \ref I2S_CH3ZC_INT_MASK
  *            	- \ref I2S_CH4ZC_INT_MASK
  *            	- \ref I2S_CH5ZC_INT_MASK
  *            	- \ref I2S_CH6ZC_INT_MASK
  *            	- \ref I2S_CH7ZC_INT_MASK
  * @return 	None
  * @details 	This function disables the interrupt according to the u32Mask parameter.
  * @note   	Only SPI1 and SPI2 support I2S mode.
  */
#define I2S_DISABLE_INT(i2s, u32Mask)	((i2s)->IEN &= ~u32Mask)

/**
  * @brief  	Get the interrupt flag.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Mask The mask value for all interrupt flags.
  * @return 	The interrupt flags specified by the u32mask parameter.
  * @details 	This macro will return the combination interrupt flags of SPI_I2SSTS register. The flags are specified by the u32mask parameter.
  */
static __INLINE uint32_t I2S_GET_INT_FLAG(I2S_T *i2s, uint32_t u32Mask)
{
	uint32_t u32Temp = u32Mask & 0xFF000000;
	u32Mask &= 0x00FFFFFF;
	if(u32Mask != 0)
	{
		return (i2s->STATUS0&u32Mask)?1:0;
	}
	else	
	{
		u32Temp >>=24 ;
		return (i2s->STATUS1&u32Temp)?1:0;	
	}
}
/**
  * @brief  	Clear the interrupt flag.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32Mask The mask value for all interrupt flags.
  *				- \ref I2S_ALL_INT_FLAG
  *				- \ref I2S_RXUD_INT_FLAG
  *				- \ref I2S_RXOV_INT_FLAG
  *				- \ref I2S_TXUD_INT_FLAG
  *				- \ref I2S_TXOV_INT_FLAG
  *				- \ref I2S_CH0ZC_INT_FLAG
  *				- \ref I2S_CH1ZC_INT_FLAG
  *				- \ref I2S_CH2ZC_INT_FLAG
  *				- \ref I2S_CH3ZC_INT_FLAG
  *				- \ref I2S_CH4ZC_INT_FLAG
  *				- \ref I2S_CH5ZC_INT_FLAG
  *				- \ref I2S_CH6ZC_INT_FLAG
  *				- \ref I2S_CH7ZC_INT_FLAG
  * @return 	None
  * @details 	This macro will clear the interrupt flags specified by the u32mask parameter.
  * @note 		Except TX and RX FIFO threshold interrupt flags, the other interrupt flags can be cleared by writing 1 to itself.
  */
static __INLINE void I2S_CLR_INT_FLAG(I2S_T *i2s, uint32_t u32Mask)
{
	uint32_t u32Temp = u32Mask & 0xFF000000;
	u32Mask &= 0x00FFFFFF;
	if(u32Mask != 0)
	{
		i2s->STATUS0 = u32Mask;
	}
	else	
	{
		u32Temp >>=24 ;
		i2s->STATUS1 = u32Temp;	
	}
}

/**
  * @brief  	Get TX is busy or not
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	TX is empty or not
  */
#define I2S_GET_TX_IS_BUSY(i2s) 		(((i2s)->STATUS0 & I2S_STATUS0_TXBUSY_Msk)?TRUE:FALSE )

/**
  * @brief  	Get TX is empty or not
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	TX is empty or not
  */
#define I2S_GET_TX_IS_EMPTY(i2s) 		(((i2s)->STATUS0 & I2S_STATUS0_TXEMPTY_Msk)?TRUE:FALSE )

/**
  * @brief  	Get RX is empty or not
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	Rx is empty or not
  */
#define I2S_GET_RX_IS_EMPTY(i2s) 		(((i2s)->STATUS0 & I2S_STATUS0_RXEMPTY_Msk)?TRUE:FALSE )

/**
  * @brief  	Get TX is full or not
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	TX is full or not
  */
#define I2S_GET_TX_IS_FULL(i2s) 		(((i2s)->STATUS0 & I2S_STATUS0_TXFULL_Msk)?TRUE:FALSE )

/**
  * @brief  	Get RX is full or not
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	Rx is full or not
  */
#define I2S_GET_RX_IS_FULL(i2s) 		(((i2s)->STATUS0 & I2S_STATUS0_RXFULL_Msk)?TRUE:FALSE )

/**
  * @brief  	Get which channel is in transmition.
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	Channel 
  */
#define I2S_GET_TXDATA_CH(i2s) 			((i2s)->STATUS0 & I2S_STATUS0_DATACH_Msk)
/**
  * @brief  	Get transmit FIFO level
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	TX FIFO level
  * @details 	This macro will return the number of available words in TX FIFO.
  */
#define I2S_GET_TX_FIFO_LEVEL(i2s) 		(((i2s)->STATUS1 & I2S_STATUS1_TXCNT_Msk) >> I2S_STATUS1_TXCNT_Pos  )

/**
  * @brief  	Get receive FIFO level
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @return 	RX FIFO level
  * @details 	This macro will return the number of available words in TX FIFO.
  */
#define I2S_GET_RX_FIFO_LEVEL(i2s) 		(((i2s)->STATUS1 & I2S_STATUS1_RXCNT_Msk) >> I2S_STATUS1_RXCNT_Pos )

/**
  * @brief  Write data to I2S TX FIFO.
  * @param[in] i2s The pointer of the specified I2S module.
  * @param[in] u32Data The value written to TX FIFO.
  * @return None
  * @details This macro will write a value to TX FIFO.
  */
#define I2S_WRITE_TX_FIFO(i2s, u32Data)  ( (i2s)->TXFIFO = (u32Data) )

/**
  * @brief  Read RX FIFO.
  * @param[in] i2s The pointer of the specified I2S module.
  * @return The value read from RX FIFO.
  * @details This function will return a value read from RX FIFO.
  */
#define I2S_READ_RX_FIFO(i2s) ( (i2s)->RXFIFO )

/* Function prototype declaration */
uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32Stereo, uint32_t u32DataFormat);
void I2S_Close(I2S_T *i2s);
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock);
void I2S_DisableMCLK(I2S_T *i2s);

/*@}*/ /* end of group I94100_I2S_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_I2S_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__I2S_H__

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
