/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2018
//***********************************************************************************************************

#include "ML51.h"

  
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
	* @exmaple :  ADC_Open(ADC_SINGLE,0);
  */
void ADC_Open(unsigned char u8ADCOpMode, unsigned char u8ADCChMask)
{
	set_ADCCON1_ADCEN;                              /* enable ADC circuit*/
	switch (u8ADCOpMode)														//ADC signle mode or continus mode select
	{
		case ADC_SINGLE: SFRS=0x00;ADCCON1&=0xEF; break;
		case ADC_CONTINUOUS: SFRS=0x00;ADCCON1|=0x10; break;
		default: break;
	}
	
	switch (u8ADCChMask)														//ADC input channel digtial function disable
	{
		case 0: SFRS=0x00;ADCCON0&=0xF0;AINDIDS=0xFE;MFP_P25_ADC_CH0;P25_INPUT_MODE; break;
		case 1: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x01;SFRS=0x01;AINDIDS=0xFD;MFP_P24_ADC_CH1;P24_INPUT_MODE; break;
		case 2: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x02;SFRS=0x01;AINDIDS=0xFB;MFP_P23_ADC_CH2;P23_INPUT_MODE; break;
		case 3: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x03;SFRS=0x01;AINDIDS=0xF7;MFP_P22_ADC_CH3;P22_INPUT_MODE; break;
		case 4: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x04;SFRS=0x01;AINDIDS=0xEF;MFP_P21_ADC_CH4;P21_INPUT_MODE; break;
		case 5: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x05;SFRS=0x01;AINDIDS=0xDF;MFP_P20_ADC_CH5;P20_INPUT_MODE; break;
		case 6: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x06;SFRS=0x01;AINDIDS=0xBF;MFP_P31_ADC_CH6;P31_INPUT_MODE; break;
		case 7: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x07;SFRS=0x01;AINDIDS=0x7F;MFP_P32_ADC_CH7;P32_INPUT_MODE; break;
		case VBG: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x08; break;
		case VTEMP: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x09; break;
		case VLDO: SFRS=0x00;ADCCON0&=0xF0;ADCCON0|=0x0A; break;
	}
}

/**
  * @brief Disable ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_Close(void)
{
    clr_ADCCON1_ADCEN;
}


 /**
  * @brief This configures ADC module to be ready for convert the input from selected channel
  * @param[in] u16ADCRBase Decides the ADC RAM Base Address High byte + Low byte total
  * @param[in] u8ADCRLength Decides the ADC continui The total sampling numbers for ADC continue sampling select. 
  * @param[in] u8ADCSpeed is the ADCSR.7 Setting for select ADC low speed. 
  * @return  None
  * @note ML51 series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
	* @example ADC_InitialContinous(0x0300,128,ADCSlowSpeed);
  */
void ADC_InitialContinous(unsigned int u16ADCRBase, unsigned char u8ADCRLength, unsigned char u8ADCSpeed)
{
	SFRS=0x00;
	ADCBAL=u16ADCRBase;
	ADCBAH = u16ADCRBase>>8;
	ADCSN  = u8ADCRLength - 1; 	//Offset value, since the actually sampling number= ADCSN[7:0] + 1
	switch (u8ADCSpeed)
	{
		case ADCSlowSpeed: set_ADCSR_SLOW; break;
		case ADCHighSpeed: clr_ADCSR_SLOW; break;
		default: break;
	}
}


 /**
  * @brief This configures ADC Sampling time 
  * @param[in] u8ADCDIV Decides the ADC clock devider value. Value from 0 ~ 7, devider is from 1 ~ 128, default value is 0 means Fadc = Fsys/1 
  * @param[in] u8ADCAQT Decides the ADC acquisition time base to add sampling time for ADC input, value is from 0 ~ 7, time = (4*u8ADCAQT+10)/Fadc, default value is 10/Fsys = 417ns.
  * @return  None
  * @note 
	* @example ADC_ConvertTime(2,7);
  */
void ADC_ConvertTime(unsigned char u8ADCDIV, unsigned char u8ADCAQT)
{
	SFRS=0x00;
	ADCSR &= 0x8F;
	ADCSR |= (u8ADCDIV&0x07)<<4;
	ADCCON2&=0xF1;
	ADCCON2|=(u8ADCAQT&0x07)<<1;
}

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] adc Base address of ADC module
  * @param[in] u8Source Decides the hardware trigger source. Valid values are:
  *                       - \ref ADC_HWT_PWM0CH0            :A/D conversion is started by PWM0CH0 status.
	* 											- \ref ADC_HWT_PWM0CH2            :A/D conversion is started by PWM0CH2 status.
	* 											- \ref ADC_HWT_PWM0CH4            :A/D conversion is started by PWM0CH4 status.
	* 											- \ref ADC_HWT_STADC              :A/D conversion is started by STADC pin status.
  * @param[in] u8Param While ADC trigger by PWM, this parameter is used to set the delay between PWM
  *                     trigger and ADC conversion. Valid values are from 0 ~ 0xFF, and actual delay
  *                     time is (4 * u32Param * HCLK). While ADC trigger by external pin, this parameter
  *                     is used to set trigger condition. Valid values are:
  *                      - \ref ADC_HWT_FALLINGEDGE     :STADC pin or PWM channel falling edge active
  *                      - \ref ADC_HWT_RISINGEDGE    	:STADC pin or PWM channel rising edge active
  *                      - \ref ADC_HWT_CENTRAL  				:PWM channel period central active
  *                      - \ref ADC_HWT_END   					:PWM channel period end active
	* @param[in] u8ADCHWTDelay Define External trigger delay time between PWM
  *                     External trigger delay time = FADC / ADCDLY
  * @return None
  * @note ADC hardware trigger source does not support PWM trigger (M05xxBN only).
	* @example ADC_EnableHWTrigger(ADC_HWT_STADC, ADC_HWT_FALLINGEDGE,0);
  */
void ADC_EnableHWTrigger(unsigned char u8ADCSHWTource, unsigned char u8ADCHWTParam, unsigned char u8ADCHWTDelay)
{
	SFRS=0x00;
	switch(u8ADCSHWTource)
	{
		case ADC_HWT_PWM0CH0: ADCCON0&=0xCF; break;
		case ADC_HWT_PWM0CH2: ADCCON0&=0xCF;ADCCON0|=0x10;break;
		case ADC_HWT_PWM0CH4: ADCCON0&=0xCF;ADCCON0|=0x20;break;
		case ADC_HWT_STADC:   ADCCON0&=0xCF;ADCCON0|=0x30;break;
		default: break;
	}
	switch(u8ADCHWTParam)
	{
		case ADC_HWT_FALLINGEDGE: ADCCON1&=0xF3;break;
		case ADC_HWT_RISINGEDGE:  ADCCON1&=0xF3;ADCCON1|=0x04;break;
		case ADC_HWT_CENTRAL:     ADCCON1&=0xF3;ADCCON1|=0x08;break;
		case ADC_HWT_END:         ADCCON1&=0xF3;ADCCON1|=0x0C;break;
		default: break;
	}
	ADCDLY = u8ADCHWTDelay;
	clr_ADCCON0_ADCS;
	ADCCON1|=0x02;							//ADC external conversion trigger enable bit
}

void ADC_DisableHWTrigger(void)
{
	clr_ADCCON1_ADCEX;
}

/**
  * @brief Enable the interrupt(s) selected by u8IntSource parameter.
  * @param[in] adc Base address of ADC module
	* @param[in] u8ADCINT Select Enable or disable 
  *                     - \ref Enable :then difine ADC interrupt source.
  *                     - \ref Disable
  * @param[in] u8IntSource The combination of interrupt status bits listed below. Each bit
  *                    corresponds to a interrupt status. This parameter decides which
  *                    interrupts will be enabled.
  *                     - \ref ADC_INT_HALFDONE :ADC convert continuse mode half done interrupt mode
  *                     - \ref ADC_INT_CONTDONE :ADC convert continuse mode full complete interrupt mode
  *                     - \ref ADC_INT_SINGLE   :ADC convert signle mode complete
  * @return None
	* @example ADC_EnableInt(ADC_INT_SINGLE);
  */
void ADC_Interrupt(unsigned char u8ADCINT, unsigned char u8ADCIntSource)
{
		SFRS=0x00;
		switch (u8ADCINT)
		{
			case Disable: 
				clr_IE_EADC; 
			break;
			
			case Enable: 
				switch (u8ADCIntSource)
					{
						case ADC_INT_HALFDONE:	ADCCON1|=0x30; break;
						case ADC_INT_CONTDONE:  ADCCON1|=0x01; ADCCON1&=0xDF; break;
						case ADC_INT_SINGLE:    ADCCON1&=0xCF; break;
						default: break;
					}
				set_IE_EADC;
			break;
			default: break;
		}
}

/**
  * @brief Read the bandgap value base on Vref = 3.072V storage address after UID area.
  * @param[in] none
  * @return 12bit bandgap value
	* @example temp = READ_BANDGAP();
  */
unsigned int READ_BANDGAP()
{
		unsigned char BandgapHigh,BandgapLow;
	  unsigned int u16bgvalue;
		set_CHPCON_IAPEN;
		IAPCN = READ_UID;
		IAPAL = 0x0d;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
		BandgapLow = IAPFD&0x0F;
		IAPAL = 0x0C;
		IAPAH = 0x00;
		set_IAPTRG_IAPGO;
		BandgapHigh = IAPFD;
		u16bgvalue = (BandgapHigh<<4)+BandgapLow;
		clr_CHPCON_IAPEN;
		return (u16bgvalue);
}

void ADC_Calibration(void)
{
	unsigned char calibration;
	SFRS=2;
	calibration =ADCCALI&0x8F;
	TA=0XAA;
	TA=0X55;
	ADCCALI=calibration;
	SFRS =0;
	ADCCON1|=SET_BIT7;
}