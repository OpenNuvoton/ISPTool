/******************************************************************************
 * @file     DPWM.h
 * @version  V3.0
 * $Revision  1 $
 * $Date: 17/07/26 05:37p $
 * @brief    I94100 Series DPWM Header File
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
 #ifndef __DPWM_H__
 #define __DPWM_H__
 
 #ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_DPWM_Driver DPWM Driver
  @{
*/

/** @addtogroup I94100_DPWM_EXPORTED_CONSTANTS DPWM Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* DPWM CTL Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DPWM_CLKSET_512FS								(0x0ul << DPWM_CTL_CLKSET_Pos)			/*!< DPWM Clock Selection : choose 512 fs working clock	 */
#define	DPWM_CLKSET_500FS								(0x1ul << DPWM_CTL_CLKSET_Pos)			/*!< DPWM Clock Selection : choose 500 fs working clock	 */

#define DPWM_FLTINTBIT_INTEGER0							(0x0ul << DPWM_CTL_FLTINTBIT_Pos)		/*!< DPWM Flotating Integer Bits : Integer is 0, Data range +/- 0.999		*/
#define DPWM_FLTINTBIT_INTEGER1							(0x1ul << DPWM_CTL_FLTINTBIT_Pos)		/*!< DPWM Flotating Integer Bits : Integer is 1, Data range +/- 1.9999		*/
#define DPWM_FLTINTBIT_INTEGER2							(0x2ul << DPWM_CTL_FLTINTBIT_Pos)		/*!< DPWM Flotating Integer Bits : Integer is 2, Data range +/- 3.9999		*/

#define DPWM_FIFO_DATAWIDTH_MSB24BITS   				(0x0ul << DPWM_CTL_FIFOWIDTH_Pos)   	/*!< DPWM FIFO Data Width : 24 Bits and MSB[31:8]    */
#define DPWM_FIFO_DATAWIDTH_16BITS      				(0x1ul << DPWM_CTL_FIFOWIDTH_Pos)   	/*!< DPWM FIFO Data Width : 16 Bits [15:0]           */
#define DPWM_FIFO_DATAWIDTH_8BITS       				(0x2ul << DPWM_CTL_FIFOWIDTH_Pos)   	/*!< DPWM FIFO Data Width : 8 Bits [7:0]             */
#define DPWM_FIFO_DATAWIDTH_24BITS      				(0x3ul << DPWM_CTL_FIFOWIDTH_Pos)   	/*!< DPWM FIFO Data Width : 24 Bits [23:0]           */

/*---------------------------------------------------------------------------------------------------------*/
/* DPWM ZOHDIV Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DPWM_ZOHDIV_MIN									(4)													/*!< DPWM ZOHDIV minimum: zOHDIV must be >= 4	 */
#define DPWM_ZOHDIV_MAX									(DPWM_ZOHDIV_ZOHDIV_Msk>>DPWM_ZOHDIV_ZOHDIV_Pos)	/*!< DPWM ZOHDIV minimum: zOHDIV must be >= 4	 */

/*---------------------------------------------------------------------------------------------------------*/
/* DPWM Biquad Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DPWM_BIQBANDNUM_1BAND							(1 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 1 band			*/
#define DPWM_BIQBANDNUM_2BAND							(2 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 2 band			*/
#define DPWM_BIQBANDNUM_3BAND							(3 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 3 band			*/
#define DPWM_BIQBANDNUM_4BAND							(4 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 4 band			*/
#define DPWM_BIQBANDNUM_5BAND							(5 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 5 band			*/
#define DPWM_BIQBANDNUM_6BAND							(6 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 6 band			*/
#define DPWM_BIQBANDNUM_7BAND							(7 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 7 band			*/
#define DPWM_BIQBANDNUM_8BAND							(8 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 8 band			*/
#define DPWM_BIQBANDNUM_9BAND							(9 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 9 band			*/
#define DPWM_BIQBANDNUM_10BAND							(10 << DPWM_CTL_BIQBANDNUM_Pos)		/*!< DPWM Biquad bands number : choose 10 band			*/

#define DPWM_COEFF_b0									(0)										/*!< DPWM Biquad coefficient b0		*/
#define DPWM_COEFF_b1									(1)										/*!< DPWM Biquad coefficient b1		*/
#define DPWM_COEFF_b2									(2)										/*!< DPWM Biquad coefficient b2		*/
#define DPWM_COEFF_a1									(3)										/*!< DPWM Biquad coefficient a1		*/
#define DPWM_COEFF_a2									(4)										/*!< DPWM Biquad coefficient a2		*/

/*@}*/ /* end of group I94100_DPWM_EXPORTED_CONSTANTS */

/** @addtogroup I94100_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

/**
  * @brief      Set Clock Selection
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u832ClkSet: colck selection
  *             - \ref DPWM_CLKSET_500FS
  *             - \ref DPWM_CLKSET_512FS
  * @return     None
  * @details    DPWM Working Clock Selection.
  */
#define DPWM_SET_CLKSET(dpwm, u32ClkSet)   (dpwm->CTL = (dpwm->CTL&(~DPWM_CTL_CLKSET_Msk))|u32ClkSet)

/**
  * @brief      Set BIQ Band Number
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u32BandNum: number of biquad bands
  *             - \ref DPWM_BIQBANDNUM_1BAND
  *             - \ref DPWM_BIQBANDNUM_2BAND
  *             - \ref DPWM_BIQBANDNUM_3BAND
  *             - \ref DPWM_BIQBANDNUM_4BAND
  *             - \ref DPWM_BIQBANDNUM_5BAND
  *             - \ref DPWM_BIQBANDNUM_6BAND
  *             - \ref DPWM_BIQBANDNUM_7BAND
  *             - \ref DPWM_BIQBANDNUM_8BAND
  *             - \ref DPWM_BIQBANDNUM_9BAND
  *             - \ref DPWM_BIQBANDNUM_10BAND
  * @return     None
  * @details    BIQ Band Number Setting (Total 10 Bands).
  */
#define DPWM_SET_BIQBANDNUM(dpwm, u32BandNum)   (dpwm->CTL = (dpwm->CTL&(~DPWM_CTL_BIQBANDNUM_Msk))|u32BandNum)

/**
  * @brief      Enable Splitter function
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Splitter function enable.
  */
#define DPWM_ENABLE_SPLITTER(dpwm)   (dpwm->CTL |= DPWM_CTL_SPLTON_Msk)

/**
  * @brief      Disable Splitter function
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Splitter function disable.
  */
#define DPWM_DISABLE_SPLITTER(dpwm)    (dpwm->CTL &= (~DPWM_CTL_SPLTON_Msk))

/**
  * @brief      Enable Biquad function
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Biquad filter function enable.
  */
#define DPWM_ENABLE_BIQUAD(dpwm)   (dpwm->CTL |= DPWM_CTL_BIQON_Msk)

/**
  * @brief      Disable Biquad function
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Biquad filter function disable.
  */
#define DPWM_DISABLE_BIQUAD(dpwm)    (dpwm->CTL &= (~DPWM_CTL_BIQON_Msk))

/**
  * @brief      Enable Floating Point format
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Floating Point format enable.
  */
#define DPWM_ENABLE_FLOAT(dpwm)   (dpwm->CTL |= DPWM_CTL_FLTEN_Msk)

/**
  * @brief      Disable Floating Point format
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Floating Point format disable.
  */
#define DPWM_DISABLE_FLOAT(dpwm)    (dpwm->CTL &= (~DPWM_CTL_FLTEN_Msk))

/**
  * @brief      Set Floating Integer Bits
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u32FloatInteger: integer bits setting
  *             - \ref DPWM_FLTINTBIT_INTEGER0
  *             - \ref DPWM_FLTINTBIT_INTEGER1
  *             - \ref DPWM_FLTINTBIT_INTEGER2
  * @return     None
  * @details    Floating Integer Bits Setting.
  */
#define DPWM_SET_FLTINTBIT(dpwm, u32FloatInteger)   (dpwm->CTL = (dpwm->CTL&(~DPWM_CTL_FLTINTBIT_Msk))|u32FloatInteger)

/**
  * @brief      Enable DPWM FIFO threshold interrupt.
  * @param[in]  dpwm The base address of DPWM module
	* @param[in]  u8Value: FIFO buffer threshold count
  * @return     None
  * @details    DPWM FIFO threshold interrupt Enabled.
  */
#define DPWM_ENABLE_FIFOTHRESHOLDINT(dpwm,u8Value)    ((dpwm)->CTL = (((dpwm)->CTL&(~DPWM_CTL_TH_Msk))|((u8Value<<DPWM_CTL_TH_Pos)&DPWM_CTL_TH_Msk))|DPWM_CTL_THIE_Msk)

/**
  * @brief      Disable DPWM FIFO threshold interrupt.
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    DPWM FIFO threshold interrupt Disabled.			
  */
#define DPWM_DISABLE_FIFOTHRESHOLDINT(dpwm)    ((dpwm)->CTL &= (~DPWM_CTL_THIE_Msk))

/**
  * @brief     	Enable DPWM driver control.
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
  * @details   	Enable DPWM driver control.
  */
#define DPWM_ENABLE_DRIVER(dpwm)    (dpwm->CTL |= DPWM_CTL_DRVEN_Msk)

/**
  * @brief     	Disable DPWM driver control.
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
	* @details   	Disable DPWM driver control.
  */
#define DPWM_DISABLE_DRIVER(dpwm)    ((dpwm)->CTL &= (~DPWM_CTL_DRVEN_Msk))

/**
  * @brief     	Enable DPWM
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
	* @details   	SPK pins are enabled and driven, data is transferred from FIFO.
  */
#define DPWM_START_PLAY(dpwm)    (dpwm->CTL |= DPWM_CTL_DPWMEN_Msk)

/**
  * @brief     	Disable DPWM
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
	* @details   	SPK pins are tri-state, CIC filter is reset, FIFO pointers are reset.
  */
#define DPWM_STOP_PLAY(dpwm)     ((dpwm)->CTL &= (~DPWM_CTL_DPWMEN_Msk))

/**
  * @brief      Enable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Enabling dead time will insert an additional clock cycle 
  *             into the switching of PMOS and NMOS driver transistors.
  */
#define DPWM_ENABLE_DEADTIME(dpwm)    ((dpwm)->CTL |= DPWM_CTL_DEADTIME_Msk)

/**
  * @brief      Disable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  */
#define DPWM_DISABLE_DEADTIME(dpwm)    ((dpwm)->CTL &= (~DPWM_CTL_DEADTIME_Msk))

/**
  * @brief      Set DPWM FIFO data width selection from PDMA
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u32DataWidth: Data wdith settings
  *             - \ref DPWM_FIFO_DATAWIDTH_MSB24BITS
  *             - \ref DPWM_FIFO_DATAWIDTH_16BITS
  *             - \ref DPWM_FIFO_DATAWIDTH_8BITS
  *             - \ref DPWM_FIFO_DATAWIDTH_24BITS
  */
#define DPWM_SET_FIFODATAWIDTH(dpwm,u32DataWidth)    ((dpwm)->CTL = ((dpwm)->CTL&(~DPWM_CTL_FIFOWIDTH_Msk))|u32DataWidth)

/**
  * @brief     	Get pointer that the valid data within the DPWM FIFO buffer
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	FIFO Pointer.
  * @details   	The Maximum value is 15. When the using level of DPWM FIFO Buffer equals to 16,
  *            	The FULL bit is set to 1.
  */
#define DPWM_GET_FIFOPOINTER(dpwm)		((dpwm->STATUS&DPWM_STATUS_FIFOPTR_Msk) >> DPWM_STATUS_FIFOPTR_Pos)

/**
  * @brief      Check DPWM FIFO count whether less than threshold
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = The valid data count within the DPWM FIFO buffer is larger than the setting value of TH
  *             1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TH.
  */
#define DPWM_IS_FIFOLESSTHANTH(dpwm)    (dpwm->STATUS&DPWM_STATUS_THIF_Msk)

/**
  * @brief      Check DPWM FIFO empty or not
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define DPWM_IS_FIFOEMPTY(dpwm)    ((dpwm)->STATUS&DPWM_STATUS_EMPTY_Msk)

/**
  * @brief      Check DPWM FIFO full or not
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define DPWM_IS_FIFOFULL(dpwm)    ((dpwm)->STATUS&DPWM_STATUS_FULL_Msk)

/**
  * @brief     	Enable DPWM PDMA interface.
  * @param[in] 	dpwm The base address of DPWM module
  * @return   	None.
  * @details   	DPWM will request data from PDMA controller whenever there is space in FIFO.
  */
#define DPWM_ENABLE_PDMA(dpwm)    (dpwm->PDMACTL |= DPWM_PDMACTL_PDMAEN_Msk)

/**
  * @brief     	Disable DPWM PDMA interface.
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
  */
#define DPWM_DISABLE_PDMA(dpwm)    (dpwm->PDMACTL &= (~DPWM_PDMACTL_PDMAEN_Msk))

/**
  * @brief     	Write DPWM FIFO Audio Data Input.
  * @param[in] 	dpwm The base address of DPWM module
  * @param[in]  u32Data: DPWM DITHER Waite Time
  * @return    	None.
	* @details   	A write to this register pushes data onto the DPWM FIFO and increments the write pointer.
  */
#define DPWM_WRITE_INDATA(dpwm,u32Data)    (dpwm->FIFO = u32Data)

/**
  * @brief      Set DPWM Clock Divider.
  * @param[in]  dpwm: base address of DPWM module
	* @param[in]  u32Div: Clock Divisor
  * @return     None
  * @details    Divider to generate the DPWM CLOCK
	*							DPWM_CLK = (PLL or PCLK)/(1+ CLKDIV)
	*							If fs is 48K or 16K, DPWM_CLK is 24.576 MHz or 24 MHz
	*							If fs is 96K, DPWM_CLK is 49.152 MHz or 48 MHz.
  */
#define DPWM_SET_CLOCKDIV(dpwm,u32Div)		(dpwm->ZOHDIV = (dpwm->ZOHDIV&(~DPWM_ZOHDIV_CLKDIV_Msk))|((u32Div<<DPWM_ZOHDIV_CLKDIV_Pos)&DPWM_ZOHDIV_CLKDIV_Msk))

/**
  * @brief      Get DPWM Clock Divider.
  * @param[in]  dpwm: base address of DPWM module
  * @return     dpwm clock divisor
  * @details    Divider to generate the DPWM CLOCK
  *							DPWM_CLK = (PLL or PCLK)/(1+ CLKDIV)
  *							If fs is 48K or 16K, DPWM_CLK is 24.576 MHz or 24 MHz
  *							If fs is 96K, DPWM_CLK is 49.152 MHz or 48 MHz.
  */
#define DPWM_GET_CLOCKDIV(dpwm)		((dpwm->ZOHDIV&DPWM_ZOHDIV_CLKDIV_Msk)>>DPWM_ZOHDIV_CLKDIV_Pos)

/**
  * @brief      Set DPWM Zero Order Hold Divisor.
  * @param[in]  dpwm: base address of DPWM module
  * @param[in]  u32Div: Zero Order Hold Divisor
  * @return     None
  * @details    The input sample rate of the DPWM is set by DPWM_CLK frequency and the divisor set in this register by the following formula:
  *							If DPWM_CLKSET is 0, K = 128.
  *							If DPWM_CLKSET is 1, K = 125.
  *							ZOHDIV = DPWM_CLK /(Fs * K ).
  *							ZOH_DIV must be >= 4
  */
#define DPWM_SET_ZOHDIV(dpwm,u32Div)    (dpwm->ZOHDIV = (dpwm->ZOHDIV&(~DPWM_ZOHDIV_ZOHDIV_Msk))|((u32Div<<DPWM_ZOHDIV_ZOHDIV_Pos)&DPWM_ZOHDIV_ZOHDIV_Msk))

/**
  * @brief      Get DPWM Zero Order Hold Divisor.
  * @param[in]  dpwm: base address of DPWM module
  * @return     Zero Hold divisor
  * @details    The input sample rate of the DPWM is set by DPWM_CLK frequency and the divisor set in this register by the following formula:
	*							If DPWM_CLKSET is 0, K = 128.
	*							If DPWM_CLKSET is 1, K = 125.
	*							ZOHDIV = DPWM_CLK /(Fs * K ).
	*							ZOH_DIV must be >= 4
  */
#define DPWM_GET_ZOHDIV(dpwm)		((dpwm->ZOHDIV&DPWM_ZOHDIV_ZOHDIV_Msk)>>DPWM_ZOHDIV_ZOHDIV_Pos)
/**
  * @brief     	Enable Coefficient Single Floating.
  * @param[in] 	dpwm The base address of DPWM module
  * @return   	None.
  * @details   	Coefficient Single Floating Enable.
  */
#define DPWM_ENABLE_COEFFFLT(dpwm)    (dpwm->COEFF_CTL |= DPWM_COEFF_CTL_COEFFFLTEN_Msk)

/**
  * @brief     	Disable Coefficient Single Floating.
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
	* @details   	Coefficient Single Floating Disable.
  */
#define DPWM_DISABLE_COEFFFLT(dpwm)    (dpwm->COEFF_CTL &= (~DPWM_COEFF_CTL_COEFFFLTEN_Msk))

/**
  * @brief     	Enable Program Coefficient.
  * @param[in] 	dpwm The base address of DPWM module
  * @return   	None.
  * @details   	Coefficient RAM is under programming mode.
  */
#define DPWM_ENABLE_PROGCOEFF(dpwm)    (dpwm->COEFF_CTL |= DPWM_COEFF_CTL_PRGCOEFF_Msk)

/**
  * @brief     	Disable Program Coefficient.
  * @param[in] 	dpwm The base address of DPWM module
  * @return    	None.
	* @details   	Coefficient RAM is in normal mode.
  */
#define DPWM_DISABLE_PROGCOEFF(dpwm)    (dpwm->COEFF_CTL &= (~DPWM_COEFF_CTL_PRGCOEFF_Msk))

/**
  * @brief     	Write one band Coefficient.
  * @param[in] 	u8Band Which band to write coefficient.
  *	@param[in]	u8Coeff Which coefficient to write.
  *             - \ref DPWM_COEFF_b0									
  *             - \ref DPWM_COEFF_b1
  *             - \ref DPWM_COEFF_b2
  *             - \ref DPWM_COEFF_a1
  *             - \ref DPWM_COEFF_a2
  *	@param[in]	u32Value The coefficient value.
  * @return    	None.
  */
void DPWM_WriteCoeff(uint32_t u32Band, uint8_t u8Coeff, uint32_t u32Value);

void DPWM_Open(void);

void DPWM_Close(void);

void DPWM_WriteFIFO(int16_t *pi32Stream, uint32_t u32Count);

void DPWM_WriteMonotoStereo(int16_t *pi32Stream, uint32_t u32Count);

uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate);

uint32_t DPWM_GetSampleRate(void);

/*@}*/ /* end of group I94100_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_DPWM_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DPWM_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
