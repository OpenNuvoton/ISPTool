/**************************************************************************//**
 * @file     TALARM.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/07/03 15:06p $
 * @brief    ISD9100 TALARM driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __TALARM_H
#define __TALARM_H

#ifdef  __cplusplus
extern "C"
{
#endif


/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_TALARM_Driver TALARM Driver
  @{
*/


/** @addtogroup ISD9100_TALARM_EXPORTED_CONSTANTS TALARM Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  TALMSEL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define TALARM_TALMVL_105C     (0x0)      /*!< Temperature Alarm Sense Level: 105C */
#define TALARM_TALMVL_115C     (0x1)      /*!< Temperature Alarm Sense Level: 115C */
#define TALARM_TALMVL_125C     (0x2)      /*!< Temperature Alarm Sense Level: 125C */
#define TALARM_TALMVL_135C     (0x4)      /*!< Temperature Alarm Sense Level: 135C */ 
#define TALARM_TALMVL_145C     (0x8)      /*!< Temperature Alarm Sense Level: 145C */

/*---------------------------------------------------------------------------------------------------------*/
/*  TALMCTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define TALARM_TALMIEN_DISABLE  (0x0ul << BODTALM_TALMCTL_TALMIEN_Pos)          /*!< Disable TALARM Interrupt */ 
#define TALARM_TALMIEN_ENABLE   (0x1ul << BODTALM_TALMCTL_TALMIEN_Pos)          /*!< Enable TALARM Interrupt */

#define TALARM_TALMEN_DISABLE   (0x0ul << BODTALM_TALMCTL_TALMEN_Pos)           /*!< Disable TALARM Detection */
#define TALARM_TALMEN_ENABLE    (0x1ul << BODTALM_TALMCTL_TALMEN_Pos)           /*!< Enable TALARM Detection */

/*@}*/ /* end of group ISD9100_TALARM_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_TALARM_EXPORTED_FUNCTIONS TALARM Exported Functions
  @{
*/

/**
  * @brief     This function will enable TALARM Interrupt.
  * @param[in] TALM The base address of BODTALM module
  * @return    None
  */
__STATIC_INLINE void TALARM_EnableInt(BODTALM_T *TALM)
{
     TALM->TALMCTL |= BODTALM_TALMCTL_TALMIEN_Msk;    
}

/**
  * @brief     This function will disable TALARM Interrupt.
  * @param[in] TALM The base address of BODTALM module
  * @return    None
  */
__STATIC_INLINE void TALARM_DisableInt(BODTALM_T *TALM)
{
     TALM->TALMCTL &= (~BODTALM_TALMCTL_TALMIEN_Msk);    
}

/**
  * @brief     This function clears the TALARM interrupt flag.
  * @param[in] TALM The base address of BODTALM module
  * @return    None
  */
__STATIC_INLINE void TALARM_ClearIntFlag(BODTALM_T *TALM)
{
    TALM->TALMCTL |= BODTALM_TALMCTL_TALMIF_Msk;
}

/**
  * @brief     This function latched TALM interrupt occurred or not.
  * @param[in] TALM The base address of BODTALM module
  * @return    TALM interrupt occurred or not
  * @retval    0 TALM interrupt did not occur
  * @retval    1 TALM interrupt occurred
  */
__STATIC_INLINE uint32_t TALARM_GetIntFlag(BODTALM_T *TALM)
{
    return ((TALM->TALMCTL&BODTALM_TALMCTL_TALMIF_Msk) ? 1 : 0 );
}

/**
  * @brief     This function will monitored current state of the temperature alarm.
  * @param[in] TALM The base address of BODTALM module
  * @return    whether TALARM active or not.
  * @retval    0 inactive
  * @retval    1 active
  */
__STATIC_INLINE uint32_t TALARM_GetOutput(BODTALM_T *TALM)
{
     return ((TALM->TALMCTL&BODTALM_TALMCTL_TALMOUT_Msk) ? 1 : 0);         	
}

void TALARM_Open(uint8_t u8TALMVL);
void TALARM_Close(void);

/*@}*/ /* end of group ISD9100_TALARM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_TALARM_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__TALARM_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/




