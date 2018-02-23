/**************************************************************************//**
 * @file     BOD.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/08/02 7:06p $
 * @brief    I91200 BOD driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __BOD_H
#define __BOD_H

#ifdef  __cplusplus
extern "C"
{
#endif


/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_BOD_Driver BOD Driver
  @{
*/


/** @addtogroup I91200_BOD_EXPORTED_CONSTANTS BOD Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  BODSEL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define BOD_BODHYS_DISABLE      (0x0ul << BOD_BODSEL_BODHYS_Pos)           /*!< Hysteresis Disabled */
#define BOD_BODHYS_ENABLE       (0x1ul << BOD_BODSEL_BODHYS_Pos)           /*!< Enable Hysteresis of BOD detection */

#define BOD_BODVL_21V        (0x0)      /*!< BOD Voltage Level 2.1V */
#define BOD_BODVL_22V        (0x1)      /*!< BOD Voltage Level 2.2V */
#define BOD_BODVL_24V        (0x2)      /*!< BOD Voltage Level 2.4V */
#define BOD_BODVL_25V        (0x3)      /*!< BOD Voltage Level 2.5V */
#define BOD_BODVL_265V       (0x4)      /*!< BOD Voltage Level 2.65V */
#define BOD_BODVL_28V        (0x5)      /*!< BOD Voltage Level 2.8V */
#define BOD_BODVL_30V        (0x6)      /*!< BOD Voltage Level 3.0V */
#define BOD_BODVL_46V        (0x7)      /*!< BOD Voltage Level 4.6V */


/*---------------------------------------------------------------------------------------------------------*/
/*  BODCTL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define BOD_BODEN_DISABLE              (0x0)       /*!< Disable BOD Detection */
#define BOD_BODEN_CONTINUOUS           (0x2)       /*!< Enable continuous BOD detection */
#define BOD_BODEN_TIME_MULTIPLEXED     (0x1)       /*!< Enable time multiplexed BOD detection */


/*@}*/ /* end of group I91200_BOD_EXPORTED_CONSTANTS */


/** @addtogroup I91200_BOD_EXPORTED_FUNCTIONS BOD Exported Functions
  @{
*/

/**
  * @brief     This function will enable BOD Interrupt.
  * @param[in] bod The base address of BOD module
  * @return    None
  */
__STATIC_INLINE void BOD_EnableInt(BOD_T *bod)
{
     bod->BODCTL |= BOD_BODCTL_BODINTEN_Msk;    
}

/**
  * @brief     This function will disable BOD Interrupt.
  * @param[in] bod The base address of BOD module
  * @return    None
  */
__STATIC_INLINE void BOD_DisableInt(BOD_T *bod)
{
     bod->BODCTL &= (~BOD_BODCTL_BODINTEN_Msk);    
}

/**
  * @brief     This function clears the BOD interrupt flag.
  * @param[in] bod The base address of BOD module
  * @return    None
  */
__STATIC_INLINE void BOD_ClearIntFlag(BOD_T *bod)
{
    bod->BODCTL |= BOD_BODCTL_BODIF_Msk;
}


/**
  * @brief     This function latched BOD interrupt occurred or not.
  * @param[in] bod The base address of BOD module
  * @return    BOD interrupt occurred or not
  * @retval    0 BOD interrupt did not occur
  * @retval    1 BOD interrupt occurred
  */
__STATIC_INLINE uint32_t BOD_GetIntFlag(BOD_T *bod)
{
    return ((bod->BODCTL&BOD_BODCTL_BODIF_Msk) ? 1 : 0 );
}

/**
  * @brief     This function will enable Hysteresis of BOD detect.
  * @param[in] bod The base address of BOD module
  * @return    None
  */
__STATIC_INLINE void BOD_EnableHyst(BOD_T *bod)
{
     bod->BODSEL |= BOD_BODSEL_BODHYS_Msk;    
}

/**
  * @brief     This function will disable Hysteresis of BOD detect.
  * @param[in] bod The base address of BOD module
  * @return    None
  */
__STATIC_INLINE void BOD_DisableHyst(BOD_T *bod)
{
     bod->BODSEL &= (~BOD_BODSEL_BODHYS_Msk);    
}

/**
  * @brief     This function will monitored to determine the current state of the BOD comparator.
  * @param[in] bod The base address of BOD module
  * @return    implies that VCC is less than BODVL or not.
  * @retval    0 VCC is more than BODVL 
  * @retval    1 VCC is less than BODVL 
  */
__STATIC_INLINE uint32_t BOD_GetOutput(BOD_T *bod)
{
     return ((bod->BODCTL&BOD_BODCTL_BODOUT_Msk) ? 1 : 0);    
}

void BOD_Open(uint8_t u8Mode, uint8_t u8BODLevel);
void BOD_Close(void);
void BOD_SetDetectionTime(uint8_t u8OnDUR, uint16_t u16OffDUR);



/*@}*/ /* end of group I91200_BOD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_BOD_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__BOD_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/



