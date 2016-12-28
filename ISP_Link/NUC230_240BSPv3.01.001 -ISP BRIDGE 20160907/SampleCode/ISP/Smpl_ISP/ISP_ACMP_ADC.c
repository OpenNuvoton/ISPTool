#include "NUC230_240.h"
#include "ISP_BSP.h"

//ADC
volatile double TVCC_voltage_old;
volatile double TVCC_voltage_new;
void Open_ADC0(void)
{
    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);
    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

    /* Disable the GPA7 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PA, 0x01);

    /* Configure the GPA7 ADC analog input pins */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA0_Msk) ;
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_ADC0 ;

    SYS->ALT_MFP1 = 0;

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 7 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x1 << 0);

    /* Power on ADC module */
    ADC_POWER_ON(ADC);
	
            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the ADC interrupt */
            ADC_EnableInt(ADC, ADC_ADF_INT);
            NVIC_EnableIRQ(ADC_IRQn);
						/* Reset the ADC interrupt indicator and Start A/D conversion */
            
}

/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{    
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* clear the A/D conversion flag */
	  TVCC_voltage_new = ADC_GET_CONVERSION_DATA(ADC, 0);
}

uint32_t Get_ADC0_Value(void)
{
    uint32_t ADC_Raw_Data;
    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    //Start ADC Conversion
    ADC_START_CONV(ADC);

    while(ADC_GET_INT_FLAG(ADC, ADC_ADF_INT) != ADC_ADF_INT);
    ADC_Raw_Data = ADC_GET_CONVERSION_DATA(ADC, 0);

    return ADC_Raw_Data;
}


void Poll_ADC0(void)
{
    
    if((ADC->ADCR&ADC_ADCR_ADST_Msk)!=ADC_ADCR_ADST_Msk) //trigger to start
    //Start ADC Conversion
    ADC_START_CONV(ADC);

}


//ACMP
void Open_ACMP1(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.6 multi-function pin for ACMP0 positive input pin */
    SYS->GPC_MFP |= SYS_GPC_MFP_PC14_ACMP1_P | SYS_GPC_MFP_PC15_ACMP1_N;
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PC14_ACMP1_P | SYS_ALT_MFP1_PC15_ACMP1_N;


    /* Disable digital input path of analog pin ACMP0_P and ACMP0_N to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PC, (1ul << 14));
    GPIO_DISABLE_DIGITAL_PATH(PC, (1ul << 15));

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP_MODULE);

    /* Configure ACMP0. Enable ACMP0 and select internal reference voltage as negative input. */
    ACMP_Open(ACMP, 1, ACMP_CR_VNEG_PIN, ACMP_CR_HYSTERESIS_ENABLE);
}

uint32_t Get_ACMP1(void)
{
    uint32_t ACMP1_Output_Level;

    if(ACMP->CMPSR & ACMP_CMPSR_CO1_Msk)
        ACMP1_Output_Level = 1;
    else
        ACMP1_Output_Level = 0;

    ACMP->CMPSR |= ACMP_CMPSR_CMPF1_Msk;        //Clear ACMP0 Flags

    return ACMP1_Output_Level;
}
