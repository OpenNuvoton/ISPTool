/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#include "ML51.h"


/**
  * @brief  Configure disable BOD for low apower application
  * @param[in]  none
  * @return     none
  * @example    BOD_Disable();
  */	
void BOD_Disable(void)
{
	clr_BODCON0_BODEN;
}

/**
  * @brief  Configure the BOD level and enable
  * @param[in]  u8BODLEVEL Comparator number.
  *                  - \ref VBOD18 (BOD level is 1.8V)
  *                  - \ref VBOD20 (BOD level is 2.0V)
  *                  - \ref VBOD24 (BOD level is 2.4V)
  *                  - \ref VBOD27 (BOD level is 2.7V)
  *                  - \ref VBOD30 (BOD level is 3.0V)
  *                  - \ref VBOD37 (BOD level is 3.7V)
  *                  - \ref VBOD44 (BOD level is 4.4V)
  * @param[in]  u8RSTENABLE defein BOD reset status
  *                  - \ref BOD_RESET_DISABLE
  *                  - \ref BOD_RESET_ENABLE
  * @return     None
  * @example    BOD_Enable(VBOD24,BOD_RESET_ENABLE);
  */		
void BOD_Enable(unsigned char u8BODLEVEL,unsigned char u8RSTENABLE)
{
		SFRS=0;
		BIT_TMP=EA;
		EA=0;
		TA_REG_TMP = (u8BODLEVEL<<4)|(u8RSTENABLE<<2)|SET_BIT7;
		TA=0xAA;
		TA=0x55;
		BODCON0=TA_REG_TMP;
		EA = BIT_TMP;
}
/**
  * @brief  Configure the BOD low power mode 
  * @param[in]  u8BODLEVEL define low power BOD mode
  *                  -u8BODLPMODE\ref LPBOD_MODE0 (BOD always enable)
  *                  - \ref LPBOD_MODE1 (turning on BOD circuit every 1.6 ms)
  *                  - \ref LPBOD_MODE2 (turning on BOD circuit every 6.4 ms)
  *                  - \ref LPBOD_MODE3 (turning on BOD circuit every 25.6 ms)
  * @param[in]  u8BODFTEN defein BOD filter status
  *                  - \ref BOD_FT_DISABLE
  *                  - \ref BOD_FT_ENABLE
  * @return     None
  * @example    BOD_Enable(VBOD24,BOD_RESET_ENABLE);
  */		
void BOD_LowPower_Enable(unsigned char u8BODLPMODE,unsigned char u8BODFTEN)
{
		SFRS=0;
		BIT_TMP=EA;
		EA=0;
		TA_REG_TMP = (u8BODLPMODE<<1)|(u8BODFTEN);
		TA=0xAA;
		TA=0x55;
		BODCON1=TA_REG_TMP;
		EA = BIT_TMP;
}
/**
  * @brief  Configure disable POR for low apower application
  * @param[in]  none
  * @return     none
  * @example    POR_Disable();
  */
void POR_Disable(void)
{
		SFRS = 1;
		BIT_TMP=EA;
		EA=0;
		TA = 0xAA;
		TA = 0x55;
		PORDIS = 0x5A;
		TA = 0xAA;
		TA = 0x55;
		PORDIS = 0xA5;
		EA = BIT_TMP;
}

/**
  * @brief  Configure Enable POR 
  * @param[in]  none
  * @return     none
  * @example    POR_Enable();
  */
void POR_Enable(void)
{
		SFRS = 1;
		BIT_TMP=EA;
		EA=0;
		TA = 0xAA;
		TA = 0x55;
		PORDIS = 0x00;
		EA = BIT_TMP;
}

/**
  * @brief  Configure disable LVR for low apower application
  * @param[in]  none
  * @return     none
  * @example    LVR_Disable();
  */
void LVR_Disable(void)
{
		SFRS = 1;
		BIT_TMP=EA;
		EA=0;
		TA = 0xAA;
		TA = 0x55;
		LVRDIS = 0x5A;
		TA = 0xAA;
		TA = 0x55;
		LVRDIS = 0xA5;
		EA = BIT_TMP;
}

/**
  * @brief  Configure Enable Low power LVR function for low ultra low power application
  * @param[in]  none
  * @return     none
  * @example    LVR_Disable();
  */
void LowPowerLVR_Enable(void)
{
		SFRS = 1;
		BIT_TMP=EA;
		EA=0;
		TA = 0xAA;
		TA = 0x55;
		LVRDIS = 0x55;
		TA = 0xAA;
		TA = 0x55;
		LVRDIS = 0xAA;
		EA = BIT_TMP;
}
/**
  * @brief  Configure Enable LVR 
  * @param[in]  none
  * @return     none
  * @example    LVR_Enable();
  */
void LVR_Enable(void)
{
		SFRS = 1;
		BIT_TMP=EA;
		EA=0;
		TA = 0xAA;
		TA = 0x55;
		LVRDIS = 0x00;
		EA = BIT_TMP;
}

void BIAS_ALL_DIGITAL(void)
{
		SFRS = 1;
		BIT_TMP=EA;
		EA=0;
		TA = 0xAA;
		TA = 0x55;
		BCKCON &= 0xCF;
		TA = 0xAA;
		TA = 0x55;
		BCKCON |= 0x10;
		EA = BIT_TMP;
}