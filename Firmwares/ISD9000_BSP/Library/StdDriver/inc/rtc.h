/**************************************************************************//**
 * @file     RTC.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/06/25 5:06p $
 * @brief    ISD9000 RTC driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __RTC_H
#define __RTC_H

#ifdef  __cplusplus
extern "C"
{
#endif


/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_RTC_Driver RTC Driver
  @{
*/


/** @addtogroup ISD9000_RTC_EXPORTED_CONSTANTS RTC Exported Constants
  @{
*/

#define RTC_TICK_4_SEC       ((uint32_t) 0x00000000<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 4 second \hideinitializer */
#define RTC_TICK_2_SEC       ((uint32_t) 0x00000001<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 2 second \hideinitializer */
#define RTC_TICK_1_SEC       ((uint32_t) 0x00000002<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 1 second \hideinitializer */
#define RTC_TICK_1_2_SEC     ((uint32_t) 0x00000003<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 1/2 second \hideinitializer */
#define RTC_TICK_1_4_SEC     ((uint32_t) 0x00000004<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 1/4 second \hideinitializer */
#define RTC_TICK_1_8_SEC     ((uint32_t) 0x00000005<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 1/8 second \hideinitializer */
#define RTC_TICK_1_16_SEC    ((uint32_t) 0x00000006<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 1/16 second \hideinitializer */
#define RTC_TICK_1_32_SEC    ((uint32_t) 0x00000007<<RTC_CTL_RTIS_Pos)   /*!< Time tick is 1/32 second \hideinitializer */

/*@}*/ /* end of group ISD9000_RTC_EXPORTED_CONSTANTS */

/** @addtogroup ISD9000_RTC_EXPORTED_FUNCTIONS RTC Exported Functions
  @{
*/

#define RTC_CLEAR_INT_FLAG() (RTC->CTL |= RTC_CTL_RTIF_Msk)			 	/*!< Clear RTC interrupt flag \hideinitializer */
#define RTC_ENABLE() (RTC->CTL |= RTC_CTL_RTCE_Msk)								/*!< Enable RTC function \hideinitializer */	
#define RTC_DISABLE() (RTC->CTL &= (~RTC_CTL_RTCE_Msk))						/*!< Disable RTC function \hideinitializer */	
#define RTC_GetIntFlag() ((RTC->CTL & RTC_CTL_RTIF_Msk)>>RTC_CTL_RTIF_Pos)	/*!< Get RTC interrupt flag \hideinitializer */	

void RTC_Open(uint32_t u32TimerInterval);
void RTC_Close(void);
void RTC_EnableInt(void);
void RTC_DisableInt(void);
void RTC_SetTimerInterval(uint32_t u32TimerInterval);


/*@}*/ /* end of group ISD9000_RTC_EXPORTED_FUNCTIONS */


/*@}*/ /* end of group ISD9000_RTC_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */


#ifdef  __cplusplus
}
#endif

#endif /* __RTC_H */


/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
