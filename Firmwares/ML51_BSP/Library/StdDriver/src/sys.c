/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#include "ML51.h"

bit BIT_TMP;
unsigned char data  TA_REG_TMP,BYTE_TMP;

  /**
  * @brief When systen switch fail always use HIRC as system clock.
  * @param[in] None
  * @return  None
  */
void FsysSwitchError(void)
{
		set_CKEN_HIRCEN;									//step1: Enable extnal clock source.
		while((CKSWT&SET_BIT5)==0);				//step2: check clock source status and wait for ready
		clr_CKSWT_OSC2;										//step3: switching system clock source to HIRC
		clr_CKSWT_OSC1;
		BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN=0x34;EA=BIT_TMP;	//Set clock enable status as default value.
		_nop_();
}
 

  /**
  * @brief This API configures ADC module to be ready for convert the input from selected channel
  * @param[in] u8OpMode Decides the ADC operation mode. Valid values are:
  *                       - \ref ADC_SINGLE               :Single mode.
  *                       - \ref ADC_CONTINUOUS           :Continuous scan mode.
  * @param[in] u8ChMask Channel enable bit. Each bit corresponds to a input channel. 0 is channel 0, 1 is channel 1..., 7 is channel 7.
	*							VBG means band-gap voltage, VTEMP means temperature sensor, VLDO means LDO voltage.
  * @return  None
  * @note ML51 series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * @note This API does not turn on ADC power nor does trigger ADC conversion
	* @exmaple :  ADC_Open(ADC_SINGLE,0);
  */
void FsysSelect(unsigned char u8FsysMode)
{
	switch (u8FsysMode)
	{
		//***** HXT enable part*****
		case FSYS_HXT:
				ClockEnable(FSYS_HIRC);                 //step1: switching system clock to HIRC
				ClockSwitch(FSYS_HIRC);
				ClockEnable(FSYS_HXT);                  //step2: switching system clock to HXT
				ClockSwitch(FSYS_HXT);
				if((CKEN&SET_BIT0)==SET_BIT0)						//step3: check clock switching flag to confirm switch OK. 
					FsysSwitchError();
				clr_CKEN_HIRCEN;                      //step4: disable HIRC if needed 
//				clr_CKEN_ELXTEN;
		break;		
				//***** LXT enable part*****
		case FSYS_LXT:
				ClockEnable(FSYS_HIRC);                 //step1: switching system clock HIRC
				ClockSwitch(FSYS_HIRC);
				ClockEnable(FSYS_LXT);									//step2: switching system clock to LXT
				ClockSwitch(FSYS_LXT);
				if((CKEN&SET_BIT0)==SET_BIT0)						//step3: check clock switching flag to confirm switch OK. 
					FsysSwitchError();
//				clr_CKEN_EHXTEN;                      //step4: disable HXT if needed
//				clr_CKEN_HIRCEN;                      //step5: disable HIRC if needed 
		break;	
				
				//***** HIRC enable part *****  
		case FSYS_HIRC:
        ClockEnable(FSYS_HIRC);                 //step1: switching system clock HIRC
				ClockSwitch(FSYS_HIRC);
				clr_CKEN_EHXTEN;                        //step2: disable HXT if needed 
		break;
		
		//***** LIRC enable part*****
		case FSYS_LIRC:
//				ClockEnable(FSYS_HIRC);                 //step1: switching system clock to HIRC
//				ClockSwitch(FSYS_HIRC);
				ClockEnable(FSYS_LIRC);                 //step2: switching system clock LIRC
				ClockSwitch(FSYS_LIRC);
				if((CKEN&SET_BIT0)==SET_BIT0)						//step3: check clock switching flag to confirm switch OK. 
					FsysSwitchError();
				clr_CKEN_HIRCEN;                        //step4: disable HIRC if needed 
//				if((CKSWT&SET_BIT5)==SET_BIT5)				/* check clock disable status*/
//					ClockDisableError();
				clr_CKEN_EHXTEN;                        //step5: disable HXT if needed 
//				if((CKSWT&SET_BIT7)==SET_BIT7)				 /*step6: check clock disable status*/
//					ClockDisableError();
		break;
				
		//***** ECLK enable part *****      
		case FSYS_ECLK:
				ClockEnable(FSYS_HIRC);                 //step1: switching system clock to HIRC
				ClockSwitch(FSYS_HIRC);
				ClockEnable(FSYS_ECLK);                 //step1: switching system clock to HIRC
				ClockSwitch(FSYS_ECLK);
				if((CKEN&SET_BIT0)==SET_BIT0)						//step4: check clock switching flag to confirm switch OK. 
					FsysSwitchError();
				clr_CKEN_HIRCEN;                        //step5: disable HIRC if needed 
				clr_CKEN_EHXTEN;                        //step6: disable HXT if needed 
		break;
	}
}

void ClockEnable(unsigned char u8FsysMode)
{
	switch (u8FsysMode)
	{
		/***** HXT enable part*****/
		case FSYS_HXT:
				set_CKEN_EHXTEN;									      //step1: Enable HXT.
				while((CKSWT|CLR_BIT7)==CLR_BIT7);			//step2: check clock source status and wait for ready
		break;		
				//***** LXT enable part******/
		case FSYS_LXT:
				set_CKEN_ELXTEN;									      //step3: Enable LXT.
				while((CKSWT|CLR_BIT6)==CLR_BIT6);			//step4: check clock source status and wait for ready
		break;	
				//***** HIRC enable part ******/
		case FSYS_HIRC:
				set_CKEN_HIRCEN;									      //step1: Enable extnal clock source.
				while((CKSWT|CLR_BIT5)==CLR_BIT5);			//step2: check clock source status and wait for ready
		break;
		//***** LIRC enable part******/
		case FSYS_LIRC:
				set_CKEN_LIRCEN;									      //step1: Enable extnal clock source.
				while((CKSWT|CLR_BIT4)==CLR_BIT4);			//step2: check clock source status and wait for ready
		break;
		//***** ECLK enable part ******/
		case FSYS_ECLK:
				set_CKEN_ECLKEN;									      //step1: Enable extnal clock source.
				while((CKSWT|CLR_BIT3)==CLR_BIT3);			//step2: check clock source status and wait for ready
		break;
	}
}

void ClockDisable(unsigned char u8FsysMode)
{
	switch (u8FsysMode)
	{
		/***** HXT Disable part*****/
		case FSYS_HXT:
				clr_CKEN_EHXTEN;									     
		break;		
				//***** LXT Disable part******/
		case FSYS_LXT:
				clr_CKEN_ELXTEN;									      
		break;	
				//***** HIRC Disable part ******/
		case FSYS_HIRC:
				clr_CKEN_HIRCEN;									      
		break;
		//***** LIRC Disable part******/
		case FSYS_LIRC:
				clr_CKEN_LIRCEN;									      
		break;
		//***** ECLK Disable part ******/
		case FSYS_ECLK:
				clr_CKEN_ECLKEN;									      
		break;
	}
}

void ClockSwitch(unsigned char u8FsysMode)
{
	BIT_TMP=EA;EA=0;
	switch (u8FsysMode)
	{
		/***** HXT Disable part*****/
		case FSYS_HXT:
				TA=0xAA;TA=0x55;CKSWT|=0x06;
		break;		
		/***** LXT Disable part******/
		case FSYS_LXT:
				TA=0xAA;TA=0x55;CKSWT|=0x07;
		break;	
		/***** HIRC Disable part ******/
		case FSYS_HIRC:
				TA=0xAA;TA=0x55;CKSWT&=0xF8;
		break;
		/***** LIRC Disable part******/
		case FSYS_LIRC:
				TA=0xAA;TA=0x55;CKSWT|=0x04;
		break;
		/***** ECLK Disable part ******/
		case FSYS_ECLK:
				TA=0xAA;TA=0x55;CKSWT|=0x02;
		break;
	}
	EA=BIT_TMP;
}

/****************************************************************************/
/* Before call software reset function, please first confirm PCON define 
/* Boot from APROM or LDROM
/****************************************************************************/
void SW_Reset(void)
{
    set_CHPCON_SWRST;
}
/*==========================================================================*/


