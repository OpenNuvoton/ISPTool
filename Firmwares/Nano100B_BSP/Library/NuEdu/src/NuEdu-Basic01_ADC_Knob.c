#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_Volume_Knob.h"

#define _ADC_Source_Clock   12000000            // from 12MHz XTAL
uint32_t ADC_Clock = 300000;                    // 300 kHz, ADC_F_Max = 42M
uint32_t Open_Volume_Knob_Fail = FALSE;

void Open_Volume_Knob(void)
{
    uint32_t ADC_Source_Clock_DIV;

    //Initial ADC Function Pin
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA0_MFP_Msk) | SYS_PA_L_MFP_PA0_MFP_ADC_CH0;
    GPIO_DISABLE_DIGITAL_PATH(PA, BIT0);

    //Initial ADC Clock Source
    while(ADC_IS_BUSY(ADC));                    //Safe for other unknown ADC device
    SYS_UnlockReg();
    ADC_Source_Clock_DIV = _ADC_Source_Clock / ADC_Clock;
    if(ADC_Source_Clock_DIV>256) {
        Open_Volume_Knob_Fail = TRUE;
        ADC_Source_Clock_DIV = 256;
        ADC_Clock = _ADC_Source_Clock / 256;
        printf("\nADC Clock Initial Fail!\n");
    } else {
        Open_Volume_Knob_Fail = FALSE;
        //  printf("\nADC Clock Initial Pass!\n");
    }

//  SYS_ResetModule(ADC_RST);                   //Donot Reset ADC, Safe for other unknown ADC device
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HXT, CLK_ADC_CLK_DIVIDER(ADC_Source_Clock_DIV));
    CLK_EnableModuleClock(ADC_MODULE);
    SYS_LockReg();

    //Initial ADC Peripheral
    ADC_Open(ADC, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_SINGLE_CYCLE, ADC_CH_0_MASK);
    ADC_SET_RESOLUTION(ADC, ADC_RESSEL_12_BIT);
    ADC_SET_REF_VOLTAGE(ADC, ADC_REFSEL_POWER);
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_POWER_ON(ADC);
}

void Close_Volume_Knob(void)
{
    //Close ADC Function Pin
    SYS->PA_L_MFP &= ~SYS_PA_L_MFP_PA0_MFP_Msk;
    GPIO_ENABLE_DIGITAL_PATH(PA, BIT0);
//  _ADC_DISABLE_CHANNEL(7);

    if(!(ADC->CHEN&ADC_CHEN_Msk)) {     //Donot Cloce ADC, Safe for other unknown ADC device
        //Close ADC Clock Source
        SYS_UnlockReg();
        SYS_ResetModule(ADC_RST);
        CLK_DisableModuleClock(ADC_MODULE);
        SYS_LockReg();
    }
}

uint32_t Get_Volume_Knob(void)
{
    uint32_t ADC_Raw_Data;

    //Start ADC Coversion
    ADC_START_CONV(ADC);
    while(!(ADC_GET_INT_FLAG(ADC, ADC_ADF_INT)));
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_Raw_Data = ADC_GET_CONVERSION_DATA(ADC, 0);

    return ADC_Raw_Data;
}
