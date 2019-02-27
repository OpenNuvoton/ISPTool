#include "ML51.h"

/**
  * @brief  Configure the specified ACMP module
  * @param[in]  u8VREFValue from 0~5
  *                  - \ref LEVEL0 = 1.538V , when VDD > 1.8V
  *                  - \ref LEVEL1 = 2.048V , when VDD > 2.4V
  *                  - \ref LEVEL2 = 2.560V , when VDD > 2.9V
  *                  - \ref LEVEL3 = 3.072V , when VDD > 3.4V
  *                  - \ref LEVEL4 = 4.096V , when VDD > 4.5V
*/

void VREF_Open(unsigned char u8VREFValue)
{
		SFRS = 0x01;
		TA_REG_TMP|=(u8VREFValue<<4)|0x03;
		TA=0xAA;TA=0x55;
		VRFCON=TA_REG_TMP;
		_delay_();
		_delay_();
		TA=0xAA;TA=0x55;
		VRFCON&=0xFD;
}


/**
  * @brief  Reload VREF trim value 
  * @param[in]  u8VREFtrimValue only 2 level as following 
  *                  - \ref LEVEL1 = 2.048V , when VDD > 2.4V
  *                  - \ref LEVEL3 = 3.072V , when VDD > 3.4V

*/
void VREF_RELOAD(unsigned char u8VREFtrimValue)
{
	unsigned char u8Count;
	switch(u8VREFtrimValue)
	{
		case LEVEL1:
        set_CHPCON_IAPEN;
        IAPAL = 0x43;
        IAPAH = 0x00;
        IAPCN = READ_UID;
				set_IAPTRG_IAPGO;
				u8Count = IAPFD;
				SFRS=1;
				TA=0XAA;TA=0X55;
				VRFTRIM = u8Count;
				clr_CHPCON_IAPEN;
		break;
		case LEVEL3:
			  set_CHPCON_IAPEN;
        IAPAL = 0x46;
        IAPAH = 0x00;
        IAPCN = READ_UID;
				set_IAPTRG_IAPGO;
				u8Count = IAPFD;
				SFRS=1;
				TA=0XAA;TA=0X55;
				VRFTRIM = u8Count;
				clr_CHPCON_IAPEN;
		break;
	}
	
}