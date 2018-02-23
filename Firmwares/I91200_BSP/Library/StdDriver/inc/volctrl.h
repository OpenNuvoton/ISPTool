/**************************************************************************//**
 * @file     volctrl.h
 * @version  V1.00
 * $Revision: 6$
 * $Date: 17/08/01 2:35p $
 * @brief    I91200 series CLK driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __VOLCTRL_H__
#define __VOLCTRL_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_VOLCTRL_Driver VOLCTRL Driver
  @{
*/

/** @addtogroup I91200_VOLCTRL_EXPORTED_CONSTANTS VOLCTRL Exported Constants
  @{
*/
#define VOLCTRL_FIXEDPOINT_SHIFT		(18)		 /*!< Ratio 6.18 fixed point format \hideinitializer */
#define VOLCTRL_MAXDB_RATIO     (0x00CCCCCC)         /*!< Max Volume ratio for 34.1dB  \hideinitializer */
#define VOLCTRL_MINDB_RATIO     (0x00000001)         /*!< Min Volume ratio for -108.3dB  \hideinitializer */
#define VOLCTRL_0DB_RATIO    	(1 << VOLCTRL_FIXEDPOINT_SHIFT)         /*!< 0dB Volume ratio   \hideinitializer */

#define VOLCTRL_SDADC_SELECTION	(0) /*!< Volume Control Select SDADC  \hideinitializer */
#define VOLCTRL_DPWM_SELECTION	(1) /*!< Volume Control Select DPWM  \hideinitializer */


typedef enum{
	eVOLCTRL_MIN_DB 	= VOLCTRL_MINDB_RATIO,
	eVOLCTRL_NEG100_DB  = 3,
	eVOLCTRL_NEG90_DB	= 8,
	eVOLCTRL_NEG80_DB 	= 26,
	eVOLCTRL_NEG70_DB	= 83,
	eVOLCTRL_NEG60_DB	= 262,
	eVOLCTRL_NEG50_DB 	= 829,
	eVOLCTRL_NEG40_DB 	= 2621,
	eVOLCTRL_NEG30_DB	= 8290,
	eVOLCTRL_NEG20_DB	= 26214,
	eVOLCTRL_NEG10_DB	= 82897,
	eVOLCTRL_NEG6_DB	= 131383,
	eVOLCTRL_NEG3_DB	= 185584,
	eVOLCTRL_0_DB		= VOLCTRL_0DB_RATIO,
	eVOLCTRL_3_DB		= 370288,
	eVOLCTRL_6_DB		= 523046,
	eVOLCTRL_10_DB		= 828972,
	eVOLCTRL_15_DB		= 1474144,
	eVOLCTRL_20_DB		= 2621440,
	eVOLCTRL_25_DB		= 4661653,
	eVOLCTRL_30_DB		= 8289721,
	eVOLCTRL_MAX_DB 	= VOLCTRL_MAXDB_RATIO,
} E_VOLCTRL_DB;										/*!< Define ratio for volume dB; volume db = 20*log10(ratio>>VOLCTRL_FIXEDPOINT_SHIFT) \hideinitializer */

/*@}*/ /* end of group I91200_VOLCTRL_EXPORTED_CONSTANTS */


/** @addtogroup I91200_VOLCTRL_EXPORTED_FUNCTIONS VOLCTRL Exported Functions
  @{
*/

/**
  * @brief     Enable Zero crossing for gain updated.
  * @param[in] volctrl Base address of VOLCTRL module.
  * @param[in] u8whichIP Select IP to control volume.
  *            - \ref VOLCTRL_SDADC_SELECTION
  *            - \ref VOLCTRL_DPWM_SELECTION
  * @return    None.
  */
#define VOLCTRL_ENABLE_ZEROCROSS(volctrl, u8whichIP)		(volctrl->EN |= (VOLCTRL_EN_SDADCZCEN_Msk << u8whichIP))

/**
  * @brief     Disable Zero crossing for gain updated.
  * @param[in] volctrl Base address of VOLCTRL module.
  * @param[in] u8whichIP Select IP to control volume.
  *            - \ref VOLCTRL_SDADC_SELECTION
  *            - \ref VOLCTRL_DPWM_SELECTION
  * @return    None.
  */
#define VOLCTRL_DISABLE_ZEROCROSS(volctrl, u8whichIP)		(volctrl->EN &= ~(VOLCTRL_EN_SDADCZCEN_Msk << u8whichIP))

/**
  * @brief     Enable the volume control function.
  * @param[in] volctrl Base address of VOLCTRL module.
  * @param[in] u8whichIP Select IP to control volume.
  *            - \ref VOLCTRL_SDADC_SELECTION
  *            - \ref VOLCTRL_DPWM_SELECTION
  * @return    None.
  * @note	   Clock source of VOLCTRL module is shared with BIQ filter, user needs to enable BIQ engine clock.
  */
#define VOLCTRL_ENABLE_FUNCTION(volctrl, u8whichIP)		(volctrl->EN |= (VOLCTRL_EN_SDADCVOLEN_Msk << u8whichIP))

/**
  * @brief     Disable the volume control function.
  * @param[in] volctrl Base address of VOLCTRL module.
  * @param[in] u8whichIP Select IP to control volume.
  *            - \ref VOLCTRL_SDADC_SELECTION
  *            - \ref VOLCTRL_DPWM_SELECTION
  * @return    None.
  */
#define VOLCTRL_DISABLE_FUNCTION(volctrl, u8whichIP)		(volctrl->EN &= ~(VOLCTRL_EN_SDADCVOLEN_Msk << u8whichIP))

/**
  * @brief     Set ratio of fixed point.
  * @param[in] volctrl Base address of VOLCTRL module.
  * @param[in] u8whichIP Select IP to control volume.
  * @param[in] u32Ratio volume ratio after fixed point shift, range is 0x00000001~0x00CCCCCC.
  * @return    None.
  */
#define VOLCTRL_SET_VOLUMERATION(volctrl, u8whichIP, u32Ratio) 			(*((__IO uint32_t *)(((uint32_t)&(volctrl->ADCVAL)) + (u8whichIP*4))) = u32Ratio&VOLCTRL_ADCVAL_VALUE_Msk)

/**
  * @brief     Get ratio of fixed point.
  * @param[in] volctrl Base address of VOLCTRL module.
  * @param[in] u8whichIP Select IP to control volume.
  * @return    Volume ratio after fixed point shift, range is 0x00000001~0x00CCCCCC.
  */
#define VOLCTRL_GET_VOLUMERATION(volctrl, u8whichIP) 					(*((__IO uint32_t *)(((uint32_t)&(volctrl->ADCVAL)) + (u8whichIP*4)))&VOLCTRL_ADCVAL_VALUE_Msk)

/**
  * @brief 	   This function set SDADC volume
  * @param[in] eVoldB The volume dB value is defined E_VOLCTRL_DB table
  *            - \ref E_VOLCTRL_DB
  * @return    None
  * @details   The volume control shares a clock source with the BIQ filter, so user needs to
  *			   call CLK_EnableModuleClock and BIQ_LoadDefaultCoeff to enable volume control IP.
  */
__STATIC_INLINE void VOLCTRL_Set_SDADCVol(E_VOLCTRL_DB eVoldB)
{
    VOLCTRL_ENABLE_FUNCTION(VOLCTRL, VOLCTRL_SDADC_SELECTION);
	VOLCTRL_SET_VOLUMERATION(VOLCTRL, VOLCTRL_SDADC_SELECTION, eVoldB);
}

/**
  * @brief 	   This function set DPWM volume
  * @param[in] eVoldB The volume dB value is defined E_VOLCTRL_DB table
  *            - \ref E_VOLCTRL_DB
  * @return    None
  * @details   The volume control shares a clock source with the BIQ filter, so user needs to
  *			   call CLK_EnableModuleClock and BIQ_LoadDefaultCoeff to enable volume control IP.
  */
__STATIC_INLINE void VOLCTRL_Set_DPWMVol(E_VOLCTRL_DB eVoldB)
{
    VOLCTRL_ENABLE_FUNCTION(VOLCTRL, VOLCTRL_DPWM_SELECTION);
	VOLCTRL_SET_VOLUMERATION(VOLCTRL, VOLCTRL_DPWM_SELECTION, eVoldB);
}

/*@}*/ /* end of group I91200_VOLCTRL_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_VOLCTRL_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__VOLCTRL_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
