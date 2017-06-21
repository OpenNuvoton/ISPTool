#include <stdio.h>
#include "NUC200Series.h"
#include "NuEdu-Basic01_PWM_Capture.h"

uint32_t PWM23_Clock            = 1000000;  //1 MHz
uint32_t PWM67_Clock            = 1000000;  //1 MHz
uint32_t Open_PWM6_OUT_Fail     = FALSE;
uint32_t Open_PWM3_Capture_Fail = FALSE;
uint32_t Open_PWM7_Capture_Fail = FALSE;

PWM_Capture_T PWM3 = {0,0,0,0,0,0,0};
PWM_Capture_T PWM7 = {0,0,0,0,0,0,0};

void Open_PWM6_OUT(uint32_t PWM_Frequency, uint32_t PWM_Duty)
{
    uint32_t TempPrescale;
    uint32_t TempCounter;
    uint32_t TempComparator;
    uint32_t PWM_Freq_Max;
    uint32_t PWM_Freq_Min;

    Open_PWM6_OUT_Fail = FALSE;

    //Initial PWM6 Function Pin
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE0_Msk) | SYS_GPE_MFP_PE0_PWM6;

    //Initial PWM67 Clock Source from XTAL's 12 MHz
    SYS_UnlockReg();
    if(!(SYSCLK->PWRCON&SYSCLK_PWRCON_XTL12M_EN_Msk)) {
        SYSCLK->PWRCON = SYSCLK_PWRCON_XTL12M_EN_Msk;                   //Enable XTAL's 12 MHz
        while(!(SYSCLK->CLKSTATUS&SYSCLK_CLKSTATUS_XTL12M_STB_Msk));
        SystemCoreClockUpdate();
    }
//  SYS->IPRSTC2 |= SYS_IPRSTC2_PWM47_RST_Msk;                          //Donot Reset PWM, Safe for other unknown PWM device
//  SYS->IPRSTC2 &= ~SYS_IPRSTC2_PWM47_RST_Msk;
    SYSCLK->CLKSEL2 = (SYSCLK->CLKSEL2 & ~SYSCLK_CLKSEL2_PWM67_S_EXT_Msk) | SYSCLK_CLKSEL2_PWM67_EXT_XTAL;
    SYSCLK->CLKSEL2 = (SYSCLK->CLKSEL2 & ~SYSCLK_CLKSEL2_PWM67_S_Msk) | SYSCLK_CLKSEL2_PWM67_XTAL;
    SYSCLK->APBCLK |= SYSCLK_APBCLK_PWM67_EN_Msk;
    SYS_LockReg();

    //Initial PWM67 Peripheral Clock Prescale
    TempPrescale = _PWM_Source_Clock / PWM67_Clock;                     //Prescale = XTAL / PWM67_Clock
    if(TempPrescale>_PWM_Prescale_Max) {
        Open_PWM6_OUT_Fail = TRUE;
        TempPrescale = _PWM_Prescale_Max;
        PWM67_Clock = _PWM_Source_Clock / _PWM_Prescale_Max;            //PWM67_Clock = _PWM_Source_Clock / Prescale
        printf("\nPWM67 Clock Prescale Initial Fail!\n");
    }
    _PWM_SET_TIMER_PRESCALE(PWMB, PWM_CH2, (TempPrescale-1));           //Setting PWM67 Clock Prescale
    while(PWMB->SYNCBUSY2);

    //Check Local PWM6 Channel Frequency and Duty-Scale
    PWM_Freq_Max = PWM67_Clock / _PWM_Duty_Scale;
    PWM_Freq_Min = PWM67_Clock / _PWM_Resolution;
    if(PWM_Frequency>PWM_Freq_Max) {
        Open_PWM6_OUT_Fail = TRUE;
        PWM_Frequency = PWM_Freq_Max;
        printf("\nPWM6 Frequency too High! Max are %d Hz\n", PWM_Freq_Max);
    }

    if(PWM_Frequency<PWM_Freq_Min) {
        Open_PWM6_OUT_Fail = TRUE;
        PWM_Frequency = PWM_Freq_Min;
        printf("\nPWM6 Frequency too Low! Min are %d Hz\n", PWM_Freq_Min);
    }

    if(PWM_Duty>_PWM_Duty_Scale) {
        Open_PWM6_OUT_Fail = TRUE;
        PWM_Duty = _PWM_Duty_Scale;
        printf("\nPWM6 Duty too High! Max are %d %\n", _PWM_Duty_Scale);
    }

    //Initial Local PWM6 Channel
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMB, PWM_CH2);
    while(PWMB->SYNCBUSY2);
    PWMB->CSR = (PWMB->CSR & ~PWM_CSR_CSR2_Msk) | PWM_CSR_CSR2(PWM_CSR_DIV1);   //Set PWM6 Clock Source Divider = 1
    while(PWMB->SYNCBUSY2);

    TempCounter = PWM67_Clock / PWM_Frequency;
    TempComparator = (PWM_Duty * TempCounter) / _PWM_Duty_Scale;
    PWMB->CNR2 = TempCounter - 1;
    while(PWMB->SYNCBUSY2);
    if(TempComparator==0)   PWMB->CMR2 = TempComparator;
    else                    PWMB->CMR2 = TempComparator - 1;
    while(PWMB->SYNCBUSY2);

    //Start PWM6_OUT
    _PWM_ENABLE_PWM_OUT(PWMB, PWM_CH2);
    while(PWMB->SYNCBUSY2);
    _PWM_ENABLE_TIMER(PWMB, PWM_CH2);
    while(PWMB->SYNCBUSY2);

//  if(Open_PWM6_OUT_Fail==FALSE)
//      printf("\nOpen PWM_OUT Success!\n");
}

void Close_PWM6_OUT(void)
{
    //Close PWM6 Function Pin
    SYS->GPE_MFP &= ~SYS_GPE_MFP_PE0_Msk;

    //Disable PWM6 Function
    _PWM_DISABLE_TIMER(PWMB, PWM_CH2);
    _PWM_DISABLE_PWM_OUT(PWMB, PWM_CH2);

    //Donot Cloce PWM67, Safe for other unknown PWM device
    /*  //Close PWM Clock Source
        SYS_UnlockReg();
    //  SYS->IPRSTC2 |= SYS_IPRSTC2_PWM47_RST_Msk;
    //  SYS->IPRSTC2 &= ~SYS_IPRSTC2_PWM47_RST_Msk;
        SYSCLK->APBCLK &= ~SYSCLK_APBCLK_PWM67_EN_Msk;
        SYS_LockReg();
    */
}

void Open_PWM7_Capture(void)
{
    uint32_t TempPrescale;

    Open_PWM7_Capture_Fail = FALSE;

    //Initial PWM7 Function Pin
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE1_Msk) | SYS_GPE_MFP_PE1_PWM7;

    //Initial PWM67 Clock Source from XTAL's 12 MHz
    SYS_UnlockReg();
    if(!(SYSCLK->PWRCON&SYSCLK_PWRCON_XTL12M_EN_Msk)) {
        SYSCLK->PWRCON = SYSCLK_PWRCON_XTL12M_EN_Msk;                           //Enable XTAL's 12 MHz
        while(!(SYSCLK->CLKSTATUS&SYSCLK_CLKSTATUS_XTL12M_STB_Msk));
        SystemCoreClockUpdate();
    }
//  SYS->IPRSTC2 |= SYS_IPRSTC2_PWM47_RST_Msk;                                  //Donot Reset PWM, Safe for other unknown PWM device
//  SYS->IPRSTC2 &= ~SYS_IPRSTC2_PWM47_RST_Msk;
    SYSCLK->CLKSEL2 = (SYSCLK->CLKSEL2 & ~SYSCLK_CLKSEL2_PWM67_S_EXT_Msk) | SYSCLK_CLKSEL2_PWM67_EXT_XTAL;
    SYSCLK->CLKSEL2 = (SYSCLK->CLKSEL2 & ~SYSCLK_CLKSEL2_PWM67_S_Msk) | SYSCLK_CLKSEL2_PWM67_XTAL;
    SYSCLK->APBCLK |= SYSCLK_APBCLK_PWM67_EN_Msk;
    SYS_LockReg();

    //Initial PWM67 Peripheral Clock Prescale
    TempPrescale = _PWM_Source_Clock / PWM67_Clock;                             //Prescale = XTAL / PWM67_Clock
    if(TempPrescale>_PWM_Prescale_Max) {
        Open_PWM7_Capture_Fail = TRUE;
        TempPrescale = _PWM_Prescale_Max;
        PWM67_Clock = _PWM_Source_Clock / _PWM_Prescale_Max;                    //PWM67_Clock = _PWM_Source_Clock / Prescale
        printf("\nPWM67 Clock Prescale Initial Fail!\n");
    }
    _PWM_SET_TIMER_PRESCALE(PWMB, PWM_CH3, (TempPrescale-1));
    while(PWMB->SYNCBUSY3);   //Setting PWM67 Clock Prescale

    //Initial Local PWM7 Channel
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);
    _PWM_SET_TIMER_CLOCK_DIV(PWMB, PWM_CH3, PWM_CSR_DIV1);
    while(PWMB->SYNCBUSY3);     //Set PWM7 Clock Source Divider = 1
    PWMB->CNR3 = _PWM_Resolution - 1;
    while(PWMB->SYNCBUSY3);
    PWMB->PBCR |= PWM_PBCR_BCn_Msk;

    //Start Capture
    _PWM_ENABLE_CAP_IN(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);
    _PWM_ENABLE_CAP_FUNC(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);
    _PWM_ENABLE_TIMER(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);

//  if(Open_PWM7_OUT_Fail==FALSE)
//      printf("\nOpen PWM7_Capture Success!\n");
}

void Close_PWM7_Capture(void)
{
    //Close PWM7 Function Pin
    SYS->GPE_MFP &= ~SYS_GPE_MFP_PE1_Msk;

    //Disable PWM7 Function
    _PWM_DISABLE_TIMER(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);
    _PWM_DISABLE_CAP_FUNC(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);
    _PWM_DISABLE_CAP_IN(PWMB, PWM_CH3);
    while(PWMB->SYNCBUSY3);

    //Donot Cloce PWM67, Safe for other unknown PWM device
    /*  //Close PWM Clock Source
        SYS_UnlockReg();
    //  SYS->IPRSTC2 |= SYS_IPRSTC2_PWM47_RST_Msk;
    //  SYS->IPRSTC2 &= ~SYS_IPRSTC2_PWM47_RST_Msk;
        SYSCLK->APBCLK &= ~SYSCLK_APBCLK_PWM67_EN_Msk;
        SYS_LockReg();
    */
}

void Get_PWM7_Capture_Data(void)
{
    uint32_t i;

    _PWM_CLR_CAP_RISING_INDICATOR(PWMB, PWM_CH3);
    _PWM_CLR_CAP_FALLING_INDICATOR(PWMB, PWM_CH3);

    for(i=0; i<4; ) {
        if(_PWM_GET_CAP_RISING_INDICATOR(PWMB, PWM_CH3)) {
            PWM7.Capture_Rising[i>>1] = PWMB->CRLR3;
            PWM7.Last_Edge = Rising;
            _PWM_CLR_CAP_RISING_INDICATOR(PWMB, PWM_CH3);
            i++;
        } else if(_PWM_GET_CAP_FALLING_INDICATOR(PWMB, PWM_CH3)) {
            PWM7.Capture_Falling[i>>1] = PWMB->CFLR3;
            PWM7.Last_Edge = Falling;
            _PWM_CLR_CAP_FALLING_INDICATOR(PWMB, PWM_CH3);
            i++;
        }
    }

    if(PWM7.Last_Edge == Falling) {
        //Calculate Case 1:
        if(PWM7.Capture_Rising[0]>PWM7.Capture_Rising[1]) {
            PWM7.Signal_Period = PWM7.Capture_Rising[0] - PWM7.Capture_Rising[1];                       //(us)
            PWM7.High_Period = PWM7.Capture_Rising[0] - PWM7.Capture_Falling[0];                        //(us)
        }
        //Calculate Case 2:
        else if(PWM7.Capture_Rising[0]<PWM7.Capture_Rising[1]) {
            PWM7.Signal_Period = _PWM_Resolution + PWM7.Capture_Rising[0] - PWM7.Capture_Rising[1];     //(us)
            PWM7.High_Period = PWM7.Capture_Rising[1] - PWM7.Capture_Falling[1];                        //(us)
        }
        PWM7.Low_Period = PWM7.Signal_Period - PWM7.High_Period;                                        //(us)
        PWM7.Signal_Frequency = (float) PWM67_Clock / (float) PWM7.Signal_Period;                       //(Hz)
    } else if(PWM7.Last_Edge == Rising) {
        //Calculate Case 3:
        if(PWM7.Capture_Falling[0]>PWM7.Capture_Falling[1]) {
            PWM7.Signal_Period = PWM7.Capture_Falling[0] - PWM7.Capture_Falling[1];                     //(us)
            PWM7.Low_Period = PWM7.Capture_Falling[0] - PWM7.Capture_Rising[0];                         //(us)
        }
        //Calculate Case 4:
        else if(PWM7.Capture_Rising[0]<PWM7.Capture_Rising[1]) {
            PWM7.Signal_Period = _PWM_Resolution + PWM7.Capture_Falling[0] - PWM7.Capture_Falling[1];   //(us)
            PWM7.Low_Period = PWM7.Capture_Falling[1] - PWM7.Capture_Rising[1];                         //(us)
        }
        PWM7.High_Period = PWM7.Signal_Period - PWM7.Low_Period;                                        //(us)
        PWM7.Signal_Frequency = (float) PWM67_Clock / (float) PWM7.Signal_Period;                       //(Hz)
    }
}

void Open_PWM3_Capture(void)
{
    uint32_t TempPrescale;

    Open_PWM3_Capture_Fail = FALSE;

    //Initial PWM3 Function Pin
    SYS->GPA_MFP = (SYS->GPA_MFP & ~SYS_GPA_MFP_PA15_Msk) | SYS_GPA_MFP_PA15_PWM3;
    SYS->ALT_MFP = (SYS->ALT_MFP & ~SYS_ALT_MFP_PA15_Msk) | SYS_ALT_MFP_PA15_PWM3;
    SYS->ALT_MFP1 = (SYS->ALT_MFP1 & ~SYS_ALT_MFP1_PA15_Msk) | SYS_ALT_MFP1_PA15_PWM3;

    //Initial PWM23 Clock Source from XTAL's 12 MHz
    SYS_UnlockReg();
    if(!(SYSCLK->PWRCON&SYSCLK_PWRCON_XTL12M_EN_Msk)) {
        SYSCLK->PWRCON = SYSCLK_PWRCON_XTL12M_EN_Msk;                           //Enable XTAL's 12 MHz
        while(!(SYSCLK->CLKSTATUS&SYSCLK_CLKSTATUS_XTL12M_STB_Msk));
        SystemCoreClockUpdate();
    }
//  SYS->IPRSTC2 |= SYS_IPRSTC2_PWM03_RST_Msk;                                  //Donot Reset PWM, Safe for other unknown PWM device
//  SYS->IPRSTC2 &= ~SYS_IPRSTC2_PWM03_RST_Msk;
    SYSCLK->CLKSEL1 = (SYSCLK->CLKSEL1 & ~SYSCLK_CLKSEL1_PWM23_S_Msk) | SYSCLK_CLKSEL1_PWM23_XTAL;
    SYSCLK->CLKSEL2 = (SYSCLK->CLKSEL2 & ~SYSCLK_CLKSEL2_PWM23_S_EXT_Msk) | SYSCLK_CLKSEL2_PWM23_EXT_XTAL;
    SYSCLK->APBCLK |= SYSCLK_APBCLK_PWM23_EN_Msk;
    SYS_LockReg();

    //Initial PWM23 Peripheral Clock Prescale
    TempPrescale = _PWM_Source_Clock / PWM23_Clock;                             //Prescale = XTAL / PWM23_Clock
    if(TempPrescale>_PWM_Prescale_Max) {
        Open_PWM3_Capture_Fail = TRUE;
        TempPrescale = _PWM_Prescale_Max;
        PWM23_Clock = _PWM_Source_Clock / _PWM_Prescale_Max;                    //PWM23_Clock = _PWM_Source_Clock / Prescale
        printf("\nPWM23 Clock Prescale Initial Fail!\n");
    }
    _PWM_SET_TIMER_PRESCALE(PWMA, PWM_CH3, (TempPrescale-1));
    while(PWMA->SYNCBUSY3);   //Setting PWM23 Clock Prescale

    //Initial Local PWM3 Channel
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);
    _PWM_SET_TIMER_CLOCK_DIV(PWMA, PWM_CH3, PWM_CSR_DIV1);
    while(PWMA->SYNCBUSY3);     //Set PWM3 Clock Source Divider = 1
    PWMA->CNR3 = _PWM_Resolution - 1;
    while(PWMA->SYNCBUSY3);
    PWMA->PBCR |= PWM_PBCR_BCn_Msk;

    //Start Capture
    _PWM_ENABLE_CAP_IN(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);
    _PWM_ENABLE_CAP_FUNC(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);
    _PWM_ENABLE_TIMER(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);

//  if(Open_PWM3_OUT_Fail==FALSE)
//      printf("\nOpen PWM3_Capture Success!\n");
}

void Close_PWM3_Capture(void)
{
    //Close PWM3 Function Pin
    SYS->GPA_MFP &= ~SYS_GPA_MFP_PA15_Msk;
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PA15_Msk;
    SYS->ALT_MFP1 &= ~SYS_ALT_MFP1_PA15_Msk;

    //Disable PWM3 Function
    _PWM_DISABLE_TIMER(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);
    _PWM_DISABLE_CAP_FUNC(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);
    _PWM_DISABLE_CAP_IN(PWMA, PWM_CH3);
    while(PWMA->SYNCBUSY3);

    //Donot Cloce PWM23, Safe for other unknown PWM device
    /*  //Close PWM Clock Source
        SYS_UnlockReg();
    //  SYS->IPRSTC2 |= SYS_IPRSTC2_PWM03_RST_Msk;
    //  SYS->IPRSTC2 &= ~SYS_IPRSTC2_PWM03_RST_Msk;
        SYSCLK->APBCLK &= ~SYSCLK_APBCLK_PWM23_EN_Msk;
        SYS_LockReg();
    */
}

void Get_PWM3_Capture_Data(void)
{
    uint32_t i;

    _PWM_CLR_CAP_RISING_INDICATOR(PWMA, PWM_CH3);
    _PWM_CLR_CAP_FALLING_INDICATOR(PWMA, PWM_CH3);

    for(i=0; i<4; ) {
        if(_PWM_GET_CAP_RISING_INDICATOR(PWMA, PWM_CH3)) {
            PWM3.Capture_Rising[i>>1] = PWMA->CRLR3;
            PWM3.Last_Edge = Rising;
            _PWM_CLR_CAP_RISING_INDICATOR(PWMA, PWM_CH3);
            i++;
        } else if(_PWM_GET_CAP_FALLING_INDICATOR(PWMA, PWM_CH3)) {
            PWM3.Capture_Falling[i>>1] = PWMA->CFLR3;
            PWM3.Last_Edge = Falling;
            _PWM_CLR_CAP_FALLING_INDICATOR(PWMA, PWM_CH3);
            i++;
        }
    }

    if(PWM3.Last_Edge == Falling) {
        //Calculate Case 1:
        if(PWM3.Capture_Rising[0]>PWM3.Capture_Rising[1]) {
            PWM3.Signal_Period = PWM3.Capture_Rising[0] - PWM3.Capture_Rising[1];                       //(us)
            PWM3.High_Period = PWM3.Capture_Rising[0] - PWM3.Capture_Falling[0];                        //(us)
        }
        //Calculate Case 2:
        else if(PWM3.Capture_Rising[0]<PWM3.Capture_Rising[1]) {
            PWM3.Signal_Period = _PWM_Resolution + PWM3.Capture_Rising[0] - PWM3.Capture_Rising[1];     //(us)
            PWM3.High_Period = PWM3.Capture_Rising[1] - PWM3.Capture_Falling[1];                        //(us)
        }
        PWM3.Low_Period = PWM3.Signal_Period - PWM3.High_Period;                                        //(us)
        PWM3.Signal_Frequency = (float) PWM23_Clock / (float) PWM3.Signal_Period;                       //(Hz)
    } else if(PWM3.Last_Edge == Rising) {
        //Calculate Case 3:
        if(PWM3.Capture_Falling[0]>PWM3.Capture_Falling[1]) {
            PWM3.Signal_Period = PWM3.Capture_Falling[0] - PWM3.Capture_Falling[1];                     //(us)
            PWM3.Low_Period = PWM3.Capture_Falling[0] - PWM3.Capture_Rising[0];                         //(us)
        }
        //Calculate Case 4:
        else if(PWM3.Capture_Rising[0]<PWM3.Capture_Rising[1]) {
            PWM3.Signal_Period = _PWM_Resolution + PWM3.Capture_Falling[0] - PWM3.Capture_Falling[1];   //(us)
            PWM3.Low_Period = PWM3.Capture_Falling[1] - PWM3.Capture_Rising[1];                         //(us)
        }
        PWM3.High_Period = PWM3.Signal_Period - PWM3.Low_Period;                                        //(us)
        PWM3.Signal_Frequency = (float) PWM23_Clock / (float) PWM3.Signal_Period;                       //(Hz)
    }
}
