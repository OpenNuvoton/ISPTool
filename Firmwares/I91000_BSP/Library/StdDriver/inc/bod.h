/**************************************************************************//**
 * @file     BOD.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/06/25 7:06p $
 * @brief    ISD9000 BOD driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __BOD_H
#define __BOD_H

#ifdef  __cplusplus
extern "C"
{
#endif


/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_BOD_Driver BOD Driver
  @{
*/


/** @addtogroup ISD9000_BOD_EXPORTED_CONSTANTS BOD Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  BODSEL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

#define BOD_BODVL_18V        (0x0)      /*!< BOD Voltage Level 18V */
#define BOD_BODVL_19V        (0x1)      /*!< BOD Voltage Level 19V */
#define BOD_BODVL_20V        (0x2)      /*!< BOD Voltage Level 20V */
#define BOD_BODVL_21V        (0x3)      /*!< BOD Voltage Level 21V */
#define BOD_BODVL_22V        (0x4)      /*!< BOD Voltage Level 22V */
#define BOD_BODVL_24V        (0x5)      /*!< BOD Voltage Level 24V */
#define BOD_BODVL_26V        (0x6)      /*!< BOD Voltage Level 26V */
#define BOD_BODVL_28V        (0x7)      /*!< BOD Voltage Level 28V */
#define BOD_BODVL_30V        (0x8)      /*!< BOD Voltage Level 30V */
#define BOD_BODVL_31V        (0x9)      /*!< BOD Voltage Level 31V */
#define BOD_BODVL_34V        (0xa)      /*!< BOD Voltage Level 34V */
#define BOD_BODVL_36V        (0xb)      /*!< BOD Voltage Level 36V */
#define BOD_BODVL_37V        (0xc)      /*!< BOD Voltage Level 37V */
#define BOD_BODVL_39V        (0xd)      /*!< BOD Voltage Level 39V */
#define BOD_BODVL_42V        (0xe)      /*!< BOD Voltage Level 42V */
#define BOD_BODVL_46V        (0xf)      /*!< BOD Voltage Level 46V */


/*---------------------------------------------------------------------------------------------------------*/
/*  BODCTL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define BOD_INTERRUPT_MODE     (0x0)       /*!< Enable time multiplexed BOD detection */
#define BOD_RESET_MODE         (0x1)       /*!< Enable continuous BOD detection */


/*@}*/ /* end of group ISD9000_BOD_EXPORTED_CONSTANTS */


/** @addtogroup ISD9000_BOD_EXPORTED_FUNCTIONS BOD Exported Functions
  @{
*/

/**
  * @brief     This function latched BOD interrupt occurred or not.
  * @param[in] BOD The base address of BODTALM module
  * @return    BOD interrupt occurred or not
  * @retval    0 BOD interrupt did not occur
  * @retval    1 BOD interrupt occurred
  */
__STATIC_INLINE uint32_t BOD_GetIntFlag(void)
{
    return ((SYS->BODCTL & SYS_BODCTL_BOD_INT_Msk) ? 1 : 0 );
}

/**
  * @brief     This function will monitored to determine the current state of the BOD comparator.
  * @param[in] BOD The base address of BODTALM module
  * @return    implies that VCC is less than BODVL or not.
  * @retval    0 VCC is more than BODVL 
  * @retval    1 VCC is less than BODVL 
  */
__STATIC_INLINE uint32_t BOD_GetOutput(void)
{
     return ((SYS->BODCTL & SYS_BODCTL_BOD_OUT_Msk) ? 1 : 0);    
}

void BOD_Open(uint8_t u8Mode, uint8_t u8BODLevel);
void BOD_Close(void);;
void BOD_LVR_Enable(void);
void BOD_LVR_Disable(void);
void BOD_HYS_Enable(void);
void BOD_HYS_Disable(void);
void BOD_ClearIntFlag(void);



/*@}*/ /* end of group ISD9000_BOD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_BOD_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__BOD_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/



