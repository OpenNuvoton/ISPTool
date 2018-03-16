/**************************************************************************//**
 * @file     OSC.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/08 10:00a $
 * @brief    ISD9100 Series OSC Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __OSC_H__
#define __OSC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_OSC_Driver OSC Driver
  @{
*/

/** @addtogroup ISD9100_OSC_EXPORTED_CONSTANTS OSC Exported Constants
  @{
*/
	
/*---------------------------------------------------------------------------------------------------------*/
/*  Hardware bit field define(Temp)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_IRCT10K_TRIM_Pos             (0)
#define SYS_IRCT10K_TRIM_Msk             (0x1fful<<SYS_IRCT10K_TRIM_Pos)
#define SYS_IRCT10K_TMREG_Pos            (16)  
#define SYS_IRCT10K_TMREG_Msk            (0x1ful<<SYS_IRCT10K_TMREG_Pos)  	
#define SYS_IRCT10K_IBGEN_Pos            (22)
#define SYS_IRCT10K_IBGEN_Msk            (0x3ul<<SYS_IRCT10K_IBGEN_Pos)  
#define SYS_IRCT10K_TRIMCLK_Pos          (31)
#define SYS_IRCT10K_TRIMCLK_Msk          (0x1ul<<SYS_IRCT10K_TRIMCLK_Pos)  

#define SYS_IRCTTRIM_TRIM_Pos            (0)
#define SYS_IRCTTRIM_TRIM_Msk            (0xfful<<SYS_IRCTTRIM_TRIM_Pos)
#define SYS_IRCTTRIM_RANGE_Pos           (8)  
#define SYS_IRCTTRIM_RANGE_Msk           (0x1ul<<SYS_IRCTTRIM_RANGE_Pos)  	
#define SYS_IRCTTRIM_FINE_Pos            (9)
#define SYS_IRCTTRIM_FINE_Msk            (0xful<<SYS_IRCTTRIM_FINE_Pos)  
#define SYS_IRCTTRIM_TC_Pos              (16)
#define SYS_IRCTTRIM_TC_Msk              (0xfful<<SYS_IRCTTRIM_TC_Pos)  
	
/*---------------------------------------------------------------------------------------------------------*/
/*  OSC trim onfiguration constant definitions.                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define OSC_LOWEST_FRQ_THAN_TARGET_FRQ   (1)
#define OSC_MAX_COARSE                   (7)
#define OSC_MAX_FINE                     (31)
#define OSC_NUM_SFINE                    (8)
#define OSC_RESERVE_RANGE	             (3)
#define OSC_MEASURE_HCL_DIV              (2)
#define OSC_NUM_CYCLES                   (36)

/*---------------------------------------------------------------------------------------------------------*/
/*  OSC trim fine constant definitions.                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define OSC_TRIM_FINE_100R               (0x00)
#define OSC_TRIM_FINE_75R                (0x01)
#define OSC_TRIM_FINE_50R                (0x02)
#define OSC_TRIM_FINE_42R                (0x03)
#define OSC_TRIM_FINE_33R                (0x06)
#define OSC_TRIM_FINE_30R                (0x07)
#define OSC_TRIM_FINE_25R                (0x0E)
#define OSC_TRIM_FINE_23R                (0x0F)

int32_t OSC_Trim(int32_t i32Target, uint8_t u8TrimIdx);

/*@}*/ /* end of group ISD9100_OSC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_OSC_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__OSC_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

