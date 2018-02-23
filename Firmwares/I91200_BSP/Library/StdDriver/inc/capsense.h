/**************************************************************************//**
 * @file     CapSense.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 17/08/08 11:00a $
 * @brief    I91200 Series Capture Sense Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __CAPSENSE_H__
#define __CAPSENSE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_CAPSENSE_Driver CapSense Driver
  @{
*/

/** @addtogroup I91200_CAPSENSE_EXPORTED_CONSTANTS CapSense Exported Constants
  @{
*/
	
/*---------------------------------------------------------------------------------------------------------*/
/*  CTRL constant definitions.                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define CAPSENSE_CTRL_DURATIONCOUNT_160      ( 0x0UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_320      ( 0x1UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_480      ( 0x2UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_640      ( 0x3UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_800      ( 0x4UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_960      ( 0x5UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_1120     ( 0x6UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_1280     ( 0x7UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_1440     ( 0x8UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_1600     ( 0x9UL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_1920     ( 0xAUL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_2240     ( 0xBUL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_2560     ( 0xCUL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_2880     ( 0xDUL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_3200     ( 0xEUL<<CSCAN_CTRL_DUR_CNT_Pos )
#define CAPSENSE_CTRL_DURATIONCOUNT_3840     ( 0xFUL<<CSCAN_CTRL_DUR_CNT_Pos )

#define CAPSENSE_CTRL_OSCCURRENT_300NA       ( 0x0UL<<CSCAN_CTRL_CURRENT_Pos )
#define CAPSENSE_CTRL_OSCCURRENT_450NA       ( 0x1UL<<CSCAN_CTRL_CURRENT_Pos )
#define CAPSENSE_CTRL_OSCCURRENT_600NA       ( 0x2UL<<CSCAN_CTRL_CURRENT_Pos )
#define CAPSENSE_CTRL_OSCCURRENT_1200NA      ( 0x3UL<<CSCAN_CTRL_CURRENT_Pos )

/*---------------------------------------------------------------------------------------------------------*/
/*  CYCCNT constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CAPSENSE_CYCCNT_CYCLENUMS_1          ( 0x0UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_2          ( 0x1UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_4          ( 0x2UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_8          ( 0x3UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_16         ( 0x4UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_32         ( 0x5UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_64         ( 0x6UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_128        ( 0x7UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_256        ( 0x8UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_512        ( 0x9UL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_768        ( 0xAUL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_1024       ( 0xBUL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_1536       ( 0xCUL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_2048       ( 0xDUL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_2560       ( 0xEUL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )
#define CAPSENSE_CYCCNT_CYCLENUMS_3072       ( 0xFUL<<CSCAN_CYCCNT_CYCLE_CNT_Pos )


/*@}*/ /* end of group I91200_CAPSENSE_EXPORTED_CONSTANTS */

/** @addtogroup I91200_CAPSENSE_EXPORTED_FUNCTIONS CapSense Exported Functions
  @{
*/

#define CAPSENSE_ENABLE_POWERDOWN()                 ( CSCAN->CTRL |= CSCAN_CTRL_PD_Msk )
#define CAPSENSE_DISABLE_POWERDOWN()                ( CSCAN->CTRL &= ~CSCAN_CTRL_PD_Msk )  

#define CAPSENSE_ENABLE_DURCNT(u32Count)            ( CSCAN->CTRL = (CSCAN->CTRL & ~(CSCAN_CTRL_DUR_CNT_Msk))|(CSCAN_CTRL_MODE1_Msk|u32Count) )
#define CAPSENSE_DISABLE_DURCNT()                   ( CSCAN->CTRL &= ~CSCAN_CTRL_MODE1_Msk )

#define CAPSENSE_ENABLE_SLOWCLK()                   ( CSCAN->CTRL |= CSCAN_CTRL_SLOW_CLK_Msk )
#define CAPSENSE_DISABLE_SLOWCLK()                  ( CSCAN->CTRL &= ~CSCAN_CTRL_SLOW_CLK_Msk )

#define CAPSENSE_ENABLE_INTERRUPT()                 ( CSCAN->CTRL |= CSCAN_CTRL_INT_EN_Msk )
#define CAPSENSE_DISABLE_INTERRUPT()                ( CSCAN->CTRL &= ~CSCAN_CTRL_INT_EN_Msk )

#define CAPSENSE_ENABLE()                           ( CSCAN->CTRL |= CSCAN_CTRL_EN_Msk )
#define CAPSENSE_DISABLE()                          ( CSCAN->CTRL &= ~CSCAN_CTRL_EN_Msk )

#define CAPSENSE_CLEAR_INT_FLAG()                   ( (CSCAN->INTSTS) |= CSCAN_INTSTS_INT_Msk )
#define CAPSENSE_GET_INT_FLAG()                     ( (CSCAN->INTSTS&CSCAN_INTSTS_INT_Msk)?1:0 )  

#define CAPSENSE_SET_CYCLENUMBER(u32Num)            ( CSCAN->CYCCNT = (CSCAN->CYCCNT & ~(CSCAN_CYCCNT_CYCLE_CNT_Msk))|u32Num )

#define CAPSENSE_SET_OSCCURRENT(u32Value)           ( CSCAN->CTRL = (CSCAN->CTRL & ~(CSCAN_CTRL_CURRENT_Msk))|u32Value )

void      CapSense_SetScanPinMap(uint32_t u32PinMap);
void      CapSense_SetScanOnePin(uint8_t u8Pin);
void	  CapSense_ClearScanPin(uint32_t u32PinMap);
uint8_t   CapSense_GetScanMode(void);
void      CapSense_GetPinMapValue(uint16_t* pu16Value);
uint16_t  CapSense_GetOnePinValue(void);

/*@}*/ /* end of group I91200_CAPSENSE_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_CAPSENSE_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/** @addtogroup I91200_OPA_Driver OPA Driver
  @{
*/

/** @addtogroup I91200_OPA_EXPORTED_CONSTANTS OPA Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  OPACTRL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define OPA_SHIFT				(8)		/* Registers position shift for each OPA */
#define OPA0					(0)
#define OPA1					(1)

#define OPA_INPUT_MASK			(CSCAN_OPACTL_A0NS_Msk | CSCAN_OPACTL_A0PS_Msk | CSCAN_OPACTL_A0PSEL_Msk)
#define OPA_OUTPUT_MASK			(CSCAN_OPACTL_A0OEN_Msk)
#define OPA_A0O2A1_MASK			(CSCAN_OPACTL_A0O2A1P_Msk | CSCAN_OPACTL_A0O2A1N_Msk)
#define OPA_FEEDBACK_MASK		(CSCAN_OPACTL_A0O2N_Msk)

#define OPA_POSIN_PIN			(CSCAN_OPACTL_A0PS_Msk)
#define OPA_POSIN_VBIAS_HI		(0x01ul << CSCAN_OPACTL_A0PSEL_Pos)
#define OPA_POSIN_VBIAS_MID		(0x02ul << CSCAN_OPACTL_A0PSEL_Pos)
#define OPA_POSIN_VBIAS_LO		(0x03ul << CSCAN_OPACTL_A0PSEL_Pos)
#define OPA_POSIN_NOCONNECT		(0 << CSCAN_OPACTL_A0PS_Pos | 0 << CSCAN_OPACTL_A0PSEL_Pos)
#define OPA_POSIN_OPA			(CSCAN_OPACTL_A0O2A1P_Msk)		/* Only work for OPA1 */

#define OPA_NEGIN_PIN			(CSCAN_OPACTL_A0NS_Msk)
#define OPA_NEGIN_NOCONNECT		(0 << CSCAN_OPACTL_A0NS_Pos)
#define OPA_NEGIN_OPA			(CSCAN_OPACTL_A0O2A1N_Msk)		/* Only work for OPA1 */

#define OPA_FEEDBACK_ENABLE		(CSCAN_OPACTL_A0O2N_Msk)
#define OPA_FEEDBACK_DISNABLE	(0 << CSCAN_OPACTL_A0O2N_Pos)

#define OPA_GAIN_1				(0x00ul)
#define OPA_GAIN_8				(0x01ul)
#define OPA_GAIN_16				(0x02ul)
#define OPA_GAIN_24				(0x03ul)
#define OPA_GAIN_32				(0x04ul)
#define OPA_GAIN_40				(0x05ul)
#define OPA_GAIN_48				(0x06ul)
#define OPA_GAIN_56				(0x07ul)


/*@}*/ /* end of group I91200_OPA_EXPORTED_CONSTANTS */

/** @addtogroup I91200_OPA_EXPORTED_FUNCTIONS OPA Exported Functions
  @{
*/

#define OPA_ENABLE(u32Num)			  	( CSCAN->OPACTL |= (u32Num?CSCAN_OPACTL_A1EN_Msk:CSCAN_OPACTL_A0EN_Msk) )
#define OPA_DISABLE(u32Num)		 		( CSCAN->OPACTL &= ~(u32Num?CSCAN_OPACTL_A1EN_Msk:CSCAN_OPACTL_A0EN_Msk) )

#define OPA_EN_OUTPUT(u32Num)			( CSCAN->OPACTL |= (u32Num?CSCAN_OPACTL_A1OEN_Msk:CSCAN_OPACTL_A0OEN_Msk) )
#define OPA_DIS_OUTPUT(u32Num)			( CSCAN->OPACTL &= ~(u32Num?CSCAN_OPACTL_A1OEN_Msk:CSCAN_OPACTL_A0OEN_Msk) )

#define OPA_EN_POSINTPIN(u32Num)     	( CSCAN->OPACTL |= (u32Num?CSCAN_OPACTL_A1PS_Msk:CSCAN_OPACTL_A0PS_Msk) )
#define OPA_DIS_POSINTPIN(u32Num)	  	( CSCAN->OPACTL &= ~(u32Num?CSCAN_OPACTL_A1PS_Msk:CSCAN_OPACTL_A0PS_Msk) )

#define OPA_EN_NEGINTPIN(u32Num)		( CSCAN->OPACTL |= (u32Num?CSCAN_OPACTL_A1NS_Msk:CSCAN_OPACTL_A0NS_Msk) )
#define OPA_DIS_NEGINTPIN(u32Num)	  	( CSCAN->OPACTL &= ~(u32Num?CSCAN_OPACTL_A1NS_Msk:CSCAN_OPACTL_A0NS_Msk) )

#define OPA_OPA0_SET_VBIAS(u32Bias)	  	( CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_A0PSEL_Msk) | u32Bias )
#define OPA_OPA1_SET_VBIAS(u32Bias)	  	( CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_A1PSEL_Msk) | u32Bias << OPA_SHIFT )

#define OPA_EN_FEEDBACK(u32Num)	  		( CSCAN->OPACTL |= (u32Num?CSCAN_OPACTL_A1O2N_Msk:CSCAN_OPACTL_A0O2N_Msk) )
#define OPA_DI_FEEDBACK(u32Num)	  		( CSCAN->OPACTL &= ~(u32Num?CSCAN_OPACTL_A1O2N_Msk:CSCAN_OPACTL_A0O2N_Msk) )

#define OPA_GET_OUTPUT(u32Num)	   	  	( CSCAN->OPACTL & (u32Num?CSCAN_OPACTL_A1X_Msk:CSCAN_OPACTL_A0X_Msk) )

#define OPA_OPA1_SET_GAIN(u32Gain)		( CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_PGA_Msk) | u32Gain << CSCAN_OPACTL_PGA_Pos )

#define OPA_OPA1_EN_GAIN()			  	( CSCAN->OPACTL |= CSCAN_OPACTL_PGAEN_Msk )
#define OPA_OPA1_DIS_GAIN()	  			( CSCAN->OPACTL &= ~(CSCAN_OPACTL_PGAEN_Msk) )

#define OPA_EN_VREF()				  	( CSCAN->OPACTL |= CSCAN_OPACTL_VREFEN_Msk )
#define OPA_DIS_VREF()	  		  	  	( CSCAN->OPACTL &= ~(CSCAN_OPACTL_VREFEN_Msk) )

/**/
#define OPA_OPA0_EN_OUT2OPA(u32PN)		( CSCAN->OPACTL |= (u32PN?CSCAN_OPACTL_A0O2A1P_Msk:CSCAN_OPACTL_A0O2A1N_Msk) )
#define OPA_OPA0_DIS_OUT2OPA(u32PN)	  	( CSCAN->OPACTL &= ~(u32PN?CSCAN_OPACTL_A0O2A1P_Msk:CSCAN_OPACTL_A0O2A1N_Msk) )

#define OPA_EN_LPWREN()			  		( CSCAN->OPACTL |= CSCAN_OPACTL_LPWREN_Msk )
#define OPA_DIS_LPWREN() 		  	  	( CSCAN->OPACTL &= ~(CSCAN_OPACTL_LPWREN_Msk) )

/**/
#define OPA_EN_OUT2CMP(u32Num)		  	( CSCAN->OPACTL |= (u32Num?CSCAN_OPACTL_A1O2CIN_Msk:CSCAN_OPACTL_A0O2CIN_Msk) )
#define OPA_DIS_OUT2CMP(u32Num)	  		( CSCAN->OPACTL &= ~(u32Num?CSCAN_OPACTL_A1O2CIN_Msk:CSCAN_OPACTL_A0O2CIN_Msk) )

uint8_t   OPA_Enable(uint8_t u8Num, uint32_t u32PosInput, uint32_t u32NegInput, uint32_t u32OuputEnable, uint32_t u32FeedbackEnable);
void 	  OPA_Disable(uint8_t u8Num);
void   	  OPA_GainEnable(uint32_t u32Gain);
void 	  OPA_GainDisable(void);

/*@}*/ /* end of group I91200_OPA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_OPA_Driver */

/*@}*/ /* end of group I91200_Device_Driver */


/** @addtogroup I91200_CMP_Driver CMP Driver
  @{
*/

/** @addtogroup I91200_CMP_EXPORTED_CONSTANTS CMP Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  CMPCTRL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CMP_SHIFT				(8)		/* Registers position shift for each CMP */
#define CMP1					(1)
#define CMP2					(2)

#define CMP1_POSIN_CNP			(0 << CSCAN_CMPCTL_CNPSEL_Pos)
#define CMP1_POSIN_OPA0			(CSCAN_CMPCTL_CNPSEL_Msk | CSCAN_OPACTL_A0O2CIN_Msk)
#define CMP1_POSIN_OPA1			(CSCAN_CMPCTL_CNPSEL_Msk | CSCAN_OPACTL_A1O2CIN_Msk)
#define CMP2_POSIN_C2P			(CSCAN_CMPCTL_C2PSEL_Msk)
#define CMP2_POSIN_VL0			(0 << CSCAN_CMPCTL_C2PSEL_Pos)

#define CMP1_NEGIN_C1N			(CSCAN_CMPCTL_C1NSEL_Msk)
#define CMP1_NEGIN_VH0			(0 << CSCAN_CMPCTL_C1NSEL_Pos)
#define CMP2_NEGIN_CNP			(0 << CSCAN_CMPCTL_CNPSEL_Pos)
#define CMP2_NEGIN_OPA0			(CSCAN_CMPCTL_CNPSEL_Msk | CSCAN_OPACTL_A0O2CIN_Msk)
#define CMP2_NEGIN_OPA1			(CSCAN_CMPCTL_CNPSEL_Msk | CSCAN_OPACTL_A1O2CIN_Msk)

#define CMP_RISING_EDGE			(0x01ul)
#define CMP_FALLING_EDGE		(0x02ul)
#define CMP_DUAL_EDGE			(0x03ul)


/*@}*/ /* end of group I91200_CMP_EXPORTED_CONSTANTS */

/** @addtogroup I91200_CMP_EXPORTED_FUNCTIONS CMP Exported Functions
  @{
*/

#define CMP_GET_INT()					( CSCAN->CMPCTL & CSCAN_CMPCTL_CMP_INT_Msk)
#define CMP_CLEAR_INT()			  		( CSCAN->CMPCTL |= CSCAN_CMPCTL_CMP_INT_Msk)

#define CMP_CMP1_GET_OUTPUT()	   	  	( CSCAN->CMPCTL & CSCAN_CMPCTL_C1OUT_Msk)
#define CMP_CMP2_GET_OUTPUT()	   	  	( CSCAN->CMPCTL & CSCAN_CMPCTL_C2OUT_Msk)

#define CMP_EN_LPWREN()			  		( CSCAN->CMPCTL |= CSCAN_CMPCTL_LPWREN_Msk )
#define CMP_DIS_LPWREN() 		  		( CSCAN->CMPCTL &= ~(CSCAN_CMPCTL_LPWREN_Msk) )


uint8_t   CMP_Enable(uint8_t u8Num, uint32_t u32PInput, uint32_t u32NInput, uint32_t u32Ouput);
void 	  CMP_Disable(uint8_t u8Num);
void	  CMP_IntEnable(uint8_t u8Num, uint32_t u32Mode);
void	  CMP_IntDisable(uint8_t u8Num);

/*@}*/ /* end of group I91200_CMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_CMP_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
