//EIPH1                     
#define set_PWKTH   EIPH1   |= SET_BIT2
#define set_PT3H    EIPH1   |= SET_BIT1
#define set_PSH_1   EIPH1   |= SET_BIT0
                            
#define clr_PWKTH   EIPH1   &= ~SET_BIT2
#define clr_PT3H    EIPH1   &= ~SET_BIT1
#define clr_PSH_1   EIPH1   &= ~SET_BIT0
                            
//EIP1                      
#define set_PWKT    EIP1    |= SET_BIT2
#define set_PT3     EIP1    |= SET_BIT1
#define set_PS_1    EIP1    |= SET_BIT0
                            
#define clr_PWKT    EIP1    &= ~SET_BIT2
#define clr_PT3     EIP1    &= ~SET_BIT1
#define clr_PS_1    EIP1    &= ~SET_BIT0
                            
//PMD                       
#define set_PMD7    PMD     |= SET_BIT7
#define set_PMD6    PMD     |= SET_BIT6
#define set_PMD5    PMD     |= SET_BIT5
#define set_PMD4    PMD     |= SET_BIT4
#define set_PMD3    PMD     |= SET_BIT3
#define set_PMD2    PMD     |= SET_BIT2
#define set_PMD1    PMD     |= SET_BIT1
#define set_PMD0    PMD     |= SET_BIT0
                            
#define clr_PMD7    PMD     &= ~SET_BIT7
#define clr_PMD6    PMD     &= ~SET_BIT6
#define clr_PMD5    PMD     &= ~SET_BIT5
#define clr_PMD4    PMD     &= ~SET_BIT4
#define clr_PMD3    PMD     &= ~SET_BIT3
#define clr_PMD2    PMD     &= ~SET_BIT2
#define clr_PMD1    PMD     &= ~SET_BIT1
#define clr_PMD0    PMD     &= ~SET_BIT0
                            
//PMEN                      
#define set_PMEN7   PMEN    |= SET_BIT7
#define set_PMEN6   PMEN    |= SET_BIT6
#define set_PMEN5   PMEN    |= SET_BIT5
#define set_PMEN4   PMEN    |= SET_BIT4
#define set_PMEN3   PMEN    |= SET_BIT3
#define set_PMEN2   PMEN    |= SET_BIT2
#define set_PMEN1   PMEN    |= SET_BIT1
#define set_PMEN0   PMEN    |= SET_BIT0
                            
#define clr_PMEN7   PMEN    &= ~SET_BIT7
#define clr_PMEN6   PMEN    &= ~SET_BIT6
#define clr_PMEN5   PMEN    &= ~SET_BIT5
#define clr_PMEN4   PMEN    &= ~SET_BIT4
#define clr_PMEN3   PMEN    &= ~SET_BIT3
#define clr_PMEN2   PMEN    &= ~SET_BIT2
#define clr_PMEN1   PMEN    &= ~SET_BIT1
#define clr_PMEN0   PMEN    &= ~SET_BIT0

//PDTCNT

//PDTEN
#define set_PDT67EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN |= SET_BIT3  ;EA=BIT_TMP;
#define set_PDT45EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN |= SET_BIT2  ;EA=BIT_TMP;
#define set_PDT23EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN |= SET_BIT1  ;EA=BIT_TMP;
#define set_PDT01EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN |= SET_BIT0  ;EA=BIT_TMP;

#define clr_PDT67EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN &= ~SET_BIT3 ;EA=BIT_TMP;
#define clr_PDT45EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN &= ~SET_BIT2 ;EA=BIT_TMP;
#define clr_PDT23EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN &= ~SET_BIT1 ;EA=BIT_TMP;
#define clr_PDT01EN BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55 PDTEN &= ~SET_BIT0 ;EA=BIT_TMP;

//SCON_1
#define set_FE_1    FE_1  = 1
#define set_SM1_1   SM1_1 = 1
#define set_SM2_1   SM2_1 = 1
#define set_REN_1   REN_1 = 1
#define set_TB8_1   TB8_1 = 1
#define set_RB8_1   RB8_1 = 1
#define set_TI_1    TI_1  = 1
#define set_RI_1    RI_1  = 1

#define clr_FE_1    FE_1  = 0
#define clr_SM1_1   SM1_1 = 0
#define clr_SM2_1   SM2_1 = 0
#define clr_REN_1   REN_1 = 0
#define clr_TB8_1   TB8_1 = 0
#define clr_RB8_1   RB8_1 = 0
#define clr_TI_1    TI_1  = 0
#define clr_RI_1    RI_1  = 0
                            
//EIPH                      
#define set_PT2H    EIPH    |= SET_BIT7
#define set_PSPIH   EIPH    |= SET_BIT6
#define set_PFBH    EIPH    |= SET_BIT5
#define set_PWDTH   EIPH    |= SET_BIT4
#define set_PPWMH   EIPH    |= SET_BIT3
#define set_PCAPH   EIPH    |= SET_BIT2
#define set_PPIH    EIPH    |= SET_BIT1
#define set_PI2CH   EIPH    |= SET_BIT0
                            
#define clr_PT2H    EIPH    &= ~SET_BIT7
#define clr_PSPIH   EIPH    &= ~SET_BIT6
#define clr_PFBH    EIPH    &= ~SET_BIT5
#define clr_PWDTH   EIPH    &= ~SET_BIT4
#define clr_PPWMH   EIPH    &= ~SET_BIT3
#define clr_PCAPH   EIPH    &= ~SET_BIT2
#define clr_PPIH    EIPH    &= ~SET_BIT1
#define clr_PI2CH   EIPH    &= ~SET_BIT0
                            
//P0DIDS                    
#define set_P07DIDS P0DIDS  |= SET_BIT7
#define set_P06DIDS P0DIDS  |= SET_BIT6
#define set_P05DIDS P0DIDS  |= SET_BIT5
#define set_P04DIDS P0DIDS  |= SET_BIT4
#define set_P03DIDS P0DIDS  |= SET_BIT3
#define set_P02DIDS P0DIDS  |= SET_BIT2
#define set_P01DIDS P0DIDS  |= SET_BIT1
#define set_P00DIDS P0DIDS  |= SET_BIT0
                            
#define clr_P07DIDS P0DIDS  &= ~SET_BIT7
#define clr_P06DIDS P0DIDS  &= ~SET_BIT6
#define clr_P05DIDS P0DIDS  &= ~SET_BIT5
#define clr_P04DIDS P0DIDS  &= ~SET_BIT4
#define clr_P03DIDS P0DIDS  &= ~SET_BIT3
#define clr_P02DIDS P0DIDS  &= ~SET_BIT2
#define clr_P01DIDS P0DIDS  &= ~SET_BIT1
#define clr_P00DIDS P0DIDS  &= ~SET_BIT0

//SPDR

//SPSR                      
#define set_SPIF    SPSR    |= SET_BIT7
#define set_WCOL    SPSR    |= SET_BIT6
#define set_SPIOVF  SPSR    |= SET_BIT5
#define set_MODF    SPSR    |= SET_BIT4
#define set_DISMODF SPSR    |= SET_BIT3
                            
#define clr_SPIF    SPSR    &= ~SET_BIT7
#define clr_WCOL    SPSR    &= ~SET_BIT6
#define clr_SPIOVF  SPSR    &= ~SET_BIT5
#define clr_MODF    SPSR    &= ~SET_BIT4
#define clr_DISMODF SPSR    &= ~SET_BIT3
                            
//SPCR                      
#define set_SSOE    SPCR    |= SET_BIT7
#define set_SPIEN   SPCR    |= SET_BIT6
#define set_LSBFE   SPCR    |= SET_BIT5
#define set_MSTR    SPCR    |= SET_BIT4
#define set_CPOL    SPCR    |= SET_BIT3
#define set_CPHA    SPCR    |= SET_BIT2
#define set_SPR1    SPCR    |= SET_BIT1
#define set_SPR0    SPCR    |= SET_BIT0
                            
#define clr_SSOE    SPCR    &= ~SET_BIT7
#define clr_SPIEN   SPCR    &= ~SET_BIT6
#define clr_LSBFE   SPCR    &= ~SET_BIT5
#define clr_MSTR    SPCR    &= ~SET_BIT4
#define clr_CPOL    SPCR    &= ~SET_BIT3
#define clr_CPHA    SPCR    &= ~SET_BIT2
#define clr_SPR1    SPCR    &= ~SET_BIT1
#define clr_SPR0    SPCR    &= ~SET_BIT0

//ADCAQT

//B
                            
//EIP                       
#define set_PT2     EIP     |= SET_BIT7
#define set_PSPI    EIP     |= SET_BIT6
#define set_PFB     EIP     |= SET_BIT5
#define set_PWDT    EIP     |= SET_BIT4
#define set_PPWM    EIP     |= SET_BIT3
#define set_PCAP    EIP     |= SET_BIT2
#define set_PPI     EIP     |= SET_BIT1
#define set_PI2C    EIP     |= SET_BIT0
                            
#define clr_PT2     EIP     &= ~SET_BIT7
#define clr_PSPI    EIP     &= ~SET_BIT6
#define clr_PFB     EIP     &= ~SET_BIT5
#define clr_PWDT    EIP     &= ~SET_BIT4
#define clr_PPWM    EIP     &= ~SET_BIT3
#define clr_PCAP    EIP     &= ~SET_BIT2
#define clr_PPI     EIP     &= ~SET_BIT1
#define clr_PI2C    EIP     &= ~SET_BIT0

//C2H

//C2L

//PIF
#define set_PIF7    PIF     |= SET_BIT7
#define set_PIF6    PIF     |= SET_BIT6
#define set_PIF5    PIF     |= SET_BIT5
#define set_PIF4    PIF     |= SET_BIT4
#define set_PIF3    PIF     |= SET_BIT3
#define set_PIF2    PIF     |= SET_BIT2
#define set_PIF1    PIF     |= SET_BIT1
#define set_PIF0    PIF     |= SET_BIT0

#define clr_PIF7    PIF     &= ~SET_BIT7
#define clr_PIF6    PIF     &= ~SET_BIT6
#define clr_PIF5    PIF     &= ~SET_BIT5
#define clr_PIF4    PIF     &= ~SET_BIT4
#define clr_PIF3    PIF     &= ~SET_BIT3
#define clr_PIF2    PIF     &= ~SET_BIT2
#define clr_PIF1    PIF     &= ~SET_BIT1
#define clr_PIF0    PIF     &= ~SET_BIT0

//PIPEN                     
#define set_PIPEN7  PIPEN   |= SET_BIT7
#define set_PIPEN6  PIPEN   |= SET_BIT6
#define set_PIPEN5  PIPEN   |= SET_BIT5
#define set_PIPEN4  PIPEN   |= SET_BIT4
#define set_PIPEN3  PIPEN   |= SET_BIT3
#define set_PIPEN2  PIPEN   |= SET_BIT2
#define set_PIPEN1  PIPEN   |= SET_BIT1
#define set_PIPEN0  PIPEN   |= SET_BIT0
                            
#define clr_PIPEN7  PIPEN   &= ~SET_BIT7
#define clr_PIPEN6  PIPEN   &= ~SET_BIT6
#define clr_PIPEN5  PIPEN   &= ~SET_BIT5
#define clr_PIPEN4  PIPEN   &= ~SET_BIT4
#define clr_PIPEN3  PIPEN   &= ~SET_BIT3
#define clr_PIPEN2  PIPEN   &= ~SET_BIT2
#define clr_PIPEN1  PIPEN   &= ~SET_BIT1
#define clr_PIPEN0  PIPEN   &= ~SET_BIT0
                            
//PINEN                     
#define set_PINEN7  PINEN   |= SET_BIT7
#define set_PINEN6  PINEN   |= SET_BIT6
#define set_PINEN5  PINEN   |= SET_BIT5
#define set_PINEN4  PINEN   |= SET_BIT4
#define set_PINEN3  PINEN   |= SET_BIT3
#define set_PINEN2  PINEN   |= SET_BIT2
#define set_PINEN1  PINEN   |= SET_BIT1
#define set_PINEN0  PINEN   |= SET_BIT0
                            
#define clr_PINEN7  PINEN   &= ~SET_BIT7
#define clr_PINEN6  PINEN   &= ~SET_BIT6
#define clr_PINEN5  PINEN   &= ~SET_BIT5
#define clr_PINEN4  PINEN   &= ~SET_BIT4
#define clr_PINEN3  PINEN   &= ~SET_BIT3
#define clr_PINEN2  PINEN   &= ~SET_BIT2
#define clr_PINEN1  PINEN   &= ~SET_BIT1
#define clr_PINEN0  PINEN   &= ~SET_BIT0
                            
//PICON                     
#define set_PIT67   PICON   |= SET_BIT7
#define set_PIT45   PICON   |= SET_BIT6
#define set_PIT3    PICON   |= SET_BIT5
#define set_PIT2    PICON   |= SET_BIT4
#define set_PIT1    PICON   |= SET_BIT3
#define set_PIT0    PICON   |= SET_BIT2
#define set_PIPS1   PICON   |= SET_BIT1
#define set_PIPS0   PICON   |= SET_BIT0
                            
#define clr_PIT67   PICON   &= ~SET_BIT7
#define clr_PIT45   PICON   &= ~SET_BIT6
#define clr_PIT3    PICON   &= ~SET_BIT5
#define clr_PIT2    PICON   &= ~SET_BIT4
#define clr_PIT1    PICON   &= ~SET_BIT3
#define clr_PIT0    PICON   &= ~SET_BIT2
#define clr_PIPS1   PICON   &= ~SET_BIT1
#define clr_PIPS0   PICON   &= ~SET_BIT0

//ADCCON0
#define set_ADCF    ADCF     = 1
#define set_ADCS    ADCS     = 1
#define set_ETGSEL1 ETGSEL1  = 1
#define set_ETGSEL0 ETGSEL0  = 1
#define set_ADCHS3  ADCHS3   = 1
#define set_ADCHS2  ADCHS2   = 1
#define set_ADCHS1  ADCHS1   = 1
#define set_ADCHS0  ADCHS0   = 1

#define clr_ADCF    ADCF     = 0
#define clr_ADCS    ADCS     = 0
#define clr_ETGSEL1 ETGSEL1  = 0
#define clr_ETGSEL0 ETGSEL0  = 0
#define clr_ADCHS3  ADCHS3   = 0
#define clr_ADCHS2  ADCHS2   = 0
#define clr_ADCHS1  ADCHS1   = 0
#define clr_ADCHS0  ADCHS0   = 0

//C1H

//C1L

//C0H

//C0L

//ADCDLY
                            
//ADCON2                    
#define set_ADFBEN  ADCCON2  |= SET_BIT7
#define set_ADCMPOP ADCCON2  |= SET_BIT6
#define set_ADCMPEN ADCCON2  |= SET_BIT5
#define set_ADCMPO  ADCCON2  |= SET_BIT4 
#define set_P26DIDS ADCCON2  |= SET_BIT3
#define set_P20DIDS ADCCON2  |= SET_BIT2
                            
#define clr_ADFBEN  ADCCON2  &= ~SET_BIT7
#define clr_ADCMPOP ADCCON2  &= ~SET_BIT6
#define clr_ADCMPEN ADCCON2  &= ~SET_BIT5
#define clr_ADCMPO  ADCCON2  &= ~SET_BIT4 
#define clr_P26DIDS ADCCON2  &= ~SET_BIT3
#define clr_P20DIDS ADCCON2  &= ~SET_BIT2
                            
//ADCON1                    
#define set_VREFSEL ADCCON1  |= SET_BIT7
#define set_ADCDIV2 ADCCON1  |= SET_BIT6
#define set_ADCDIV1 ADCCON1  |= SET_BIT5
#define set_ADCDIV0 ADCCON1  |= SET_BIT4
#define set_ETGTYP1 ADCCON1  |= SET_BIT3 
#define set_ETGTYP0 ADCCON1  |= SET_BIT2
#define set_ADCEX   ADCCON1  |= SET_BIT1
#define set_ADCEN   ADCCON1  |= SET_BIT0
                            
#define clr_VREFSEL ADCCON1  &= ~SET_BIT7
#define clr_ADCDIV2 ADCCON1  &= ~SET_BIT6
#define clr_ADCDIV1 ADCCON1  &= ~SET_BIT5
#define clr_ADCDIV0 ADCCON1  &= ~SET_BIT4
#define clr_ETGTYP1 ADCCON1  &= ~SET_BIT3 
#define clr_ETGTYP0 ADCCON1  &= ~SET_BIT2
#define clr_ADCEX   ADCCON1  &= ~SET_BIT1
#define clr_ADCEN   ADCCON1  &= ~SET_BIT0

//ACC

//PWMCON1                   
#define set_PWMMOD1 PWMCON1  |= SET_BIT7
#define set_PWMMOD0 PWMCON1  |= SET_BIT6
#define set_GP      PWMCON1  |= SET_BIT5
#define set_PWMTYP  PWMCON1  |= SET_BIT4
#define set_FBINEN  PWMCON1  |= SET_BIT3
#define set_PWMDIV2 PWMCON1  |= SET_BIT2 
#define set_PWMDIV1 PWMCON1  |= SET_BIT1
#define set_PWMDIV0 PWMCON1  |= SET_BIT0
                            
#define clr_PWMMOD1 PWMCON1  &= ~SET_BIT7
#define clr_PWMMOD0 PWMCON1  &= ~SET_BIT6
#define clr_GP      PWMCON1  &= ~SET_BIT5
#define clr_PWMTYP  PWMCON1  &= ~SET_BIT4
#define clr_FBINEN  PWMCON1  &= ~SET_BIT3
#define clr_PWMDIV2 PWMCON1  &= ~SET_BIT2 
#define clr_PWMDIV1 PWMCON1  &= ~SET_BIT1
#define clr_PWMDIV0 PWMCON1  &= ~SET_BIT0
                            
//PIO                       
#define set_PIO7    PIO     |= SET_BIT7
#define set_PIO6    PIO     |= SET_BIT6
#define set_PIO5    PIO     |= SET_BIT5
#define set_PIO4    PIO     |= SET_BIT4
#define set_PIO3    PIO     |= SET_BIT3
#define set_PIO2    PIO     |= SET_BIT2
#define set_PIO1    PIO     |= SET_BIT1
#define set_PIO0    PIO     |= SET_BIT0
                            
#define clr_PIO7    PIO     &= ~SET_BIT7
#define clr_PIO6    PIO     &= ~SET_BIT6
#define clr_PIO5    PIO     &= ~SET_BIT5
#define clr_PIO4    PIO     &= ~SET_BIT4
#define clr_PIO3    PIO     &= ~SET_BIT3
#define clr_PIO2    PIO     &= ~SET_BIT2
#define clr_PIO1    PIO     &= ~SET_BIT1
#define clr_PIO0    PIO     &= ~SET_BIT0

//PWM45L

//PWM67L

//PWM23L

//PWM01L

//PWMPL

//PWMCON0
#define set_PWMRUN  PWMRUN   = 1
#define set_LOAD    LOAD     = 1
#define set_PWMF    PWMF     = 1
#define set_CLRPWM  CLRPWM   = 1
#define set_INTTYP1 INTTYP1  = 1
#define set_INTTYP0 INTTYP0  = 1
#define set_INTSEL1 INTSEL1  = 1
#define set_INTSEL0 INTSEL0  = 1

#define clr_PWMRUN  PWMRUN   = 0
#define clr_LOAD    LOAD     = 0
#define clr_PWMF    PWMF     = 0 
#define clr_CLRPWM  CLRPWM   = 0
#define clr_INTTYP1 INTTYP1  = 0
#define clr_INTTYP0 INTTYP0  = 0
#define clr_INTSEL1 INTSEL1  = 0
#define clr_INTSEL0 INTSEL0  = 0

//FBD
#define set_FBF     FBD     |= SET_BIT7
#define set_FBINLS  FBD     |= SET_BIT6
#define set_FBD5    FBD     |= SET_BIT5
#define set_FBD4    FBD     |= SET_BIT4
#define set_FBD3    FBD     |= SET_BIT3
#define set_FBD2    FBD     |= SET_BIT2
#define set_FBD1    FBD     |= SET_BIT1
#define set_FBD0    FBD     |= SET_BIT0

#define clr_FBF     FBD     &= ~SET_BIT7
#define clr_FBINLS  FBD      &= ~SET_BIT6
#define clr_FBD5    FBD     &= ~SET_BIT5
#define clr_FBD4    FBD     &= ~SET_BIT4
#define clr_FBD3    FBD     &= ~SET_BIT3
#define clr_FBD2    FBD     &= ~SET_BIT2
#define clr_FBD1    FBD     &= ~SET_BIT1
#define clr_FBD0    FBD     &= ~SET_BIT0

//PNP
#define set_PNP7    PNP     |= SET_BIT7
#define set_PNP6    PNP     |= SET_BIT6
#define set_PNP5    PNP     |= SET_BIT5
#define set_PNP4    PNP     |= SET_BIT4
#define set_PNP3    PNP     |= SET_BIT3
#define set_PNP2    PNP     |= SET_BIT2
#define set_PNP1    PNP     |= SET_BIT1
#define set_PNP0    PNP     |= SET_BIT0

#define clr_PNP7    PNP     &= ~SET_BIT7
#define clr_PNP6    PNP     &= ~SET_BIT6
#define clr_PNP5    PNP     &= ~SET_BIT5
#define clr_PNP4    PNP     &= ~SET_BIT4
#define clr_PNP3    PNP     &= ~SET_BIT3
#define clr_PNP2    PNP     &= ~SET_BIT2
#define clr_PNP1    PNP     &= ~SET_BIT1
#define clr_PNP0    PNP     &= ~SET_BIT0

//PWM45H

//PWM67H

//PWM23H

//PWM01H

//PWMPH

//PSW

//ADCMPH

//ADCMPL

//TH2

//TL2

//RCMP2H

//RCMP2L

//T2MOD                     
#define set_LDEN    T2MOD   |= SET_BIT7
#define set_T2DIV2  T2MOD   |= SET_BIT6
#define set_T2DIV1  T2MOD   |= SET_BIT5
#define set_T2DIV0  T2MOD   |= SET_BIT4
#define set_CAPCR   T2MOD   |= SET_BIT3
#define set_CMPCR   T2MOD   |= SET_BIT2
#define set_LDTS1   T2MOD   |= SET_BIT1
#define set_LDTS0   T2MOD   |= SET_BIT0
                            
#define clr_LDEN    T2MOD   &= ~SET_BIT7
#define clr_T2DIV2  T2MOD   &= ~SET_BIT6
#define clr_T2DIV1  T2MOD   &= ~SET_BIT5
#define clr_T2DIV0  T2MOD   &= ~SET_BIT4
#define clr_CAPCR   T2MOD   &= ~SET_BIT3
#define clr_CMPCR   T2MOD   &= ~SET_BIT2
#define clr_LDTS1   T2MOD   &= ~SET_BIT1
#define clr_LDTS0   T2MOD   &= ~SET_BIT0

//T2CON  
#define set_TF2     TF2      = 1
#define set_TR2     TR2      = 1
#define set_CMRL2   CMRL2    = 1

#define clr_TF2     TF2      = 0
#define clr_TR2     TR2      = 0
#define clr_CMRL2   CMRL2    = 0

//TA

//RH3

//RL3
                            
//T3CON                     
#define set_SMOD_1  T3CON   |= SET_BIT7
#define set_SMOD0_1 T3CON   |= SET_BIT6
#define set_BRCK    T3CON   |= SET_BIT5
#define set_TF3     T3CON   |= SET_BIT4
#define set_TR3     T3CON   |= SET_BIT3
#define set_T3PS2   T3CON   |= SET_BIT2
#define set_T3PS1   T3CON   |= SET_BIT1
#define set_T3PS0   T3CON   |= SET_BIT0
                            
#define clr_SMOD_1  T3CON   &= ~SET_BIT7
#define clr_SMOD0_1 T3CON   &= ~SET_BIT6
#define clr_BRCK    T3CON   &= ~SET_BIT5
#define clr_TF3     T3CON   &= ~SET_BIT4
#define clr_TR3     T3CON   &= ~SET_BIT3
#define clr_T3PS2   T3CON   &= ~SET_BIT2
#define clr_T3PS1   T3CON   &= ~SET_BIT1
#define clr_T3PS0   T3CON   &= ~SET_BIT0

//ADCRH

//ADCRL

//I2ADDR
#define set_GC      I2ADDR  |= SET_BIT0
#define clr_GC      I2ADDR  &= ~SET_BIT0

//I2CON  
#define set_I2CEN   I2CEN    = 1
#define set_STA     STA      = 1
#define set_STO     STO      = 1
#define set_SI      SI       = 1
#define set_AA      AA       = 1

#define clr_I2CEN   I2CEN    = 0
#define clr_STA     STA      = 0
#define clr_STO     STO      = 0
#define clr_SI      SI       = 0
#define clr_AA      AA       = 0

//I2TOC

//I2CLK

//I2STAT

//I2DAT

//SADDR_1

//SADEN_1

//SADEN

//IP
#define set_PADC    PADC     = 1
#define set_PBOD    PBOD     = 1
#define set_PS      PS       = 1
#define set_PT1     PT1      = 1
#define set_PX1     PX1      = 1
#define set_PT0     PT0      = 1
#define set_PX0     PX0      = 1

#define clr_PADC    PADC     = 0
#define clr_PBOD    PBOD     = 0
#define clr_PS      PS       = 0
#define clr_PT1     PT1      = 0
#define clr_PX1     PX1      = 0
#define clr_PT0     PT0      = 0
#define clr_PX0     PX0      = 0

//IPH                       
#define set_PADCH   IPH     |= SET_BIT6
#define set_PBODH   IPH     |= SET_BIT5
#define set_PSH     IPH     |= SET_BIT4
#define set_PT1H    IPH     |= SET_BIT3
#define set_PX11    IPH     |= SET_BIT2
#define set_PT0H    IPH     |= SET_BIT1
#define set_PX0H    IPH     |= SET_BIT0
                            
#define clr_PADCH   IPH     &= ~SET_BIT6
#define clr_PBODH   IPH     &= ~SET_BIT5
#define clr_PSH     IPH     &= ~SET_BIT4
#define clr_PT1H    IPH     &= ~SET_BIT3
#define clr_PX11    IPH     &= ~SET_BIT2
#define clr_PT0H    IPH     &= ~SET_BIT1
#define clr_PX0H    IPH     &= ~SET_BIT0

//P2SR
#define set_P2SR_6  P2SR    |= SET_BIT6
#define set_P2SR_5  P2SR    |= SET_BIT5
#define set_P2SR_4  P2SR    |= SET_BIT4
#define set_P2SR_3  P2SR    |= SET_BIT3
#define set_P2SR_2  P2SR    |= SET_BIT2
#define set_P2SR_1  P2SR    |= SET_BIT1
#define set_P2SR_0  P2SR    |= SET_BIT0

#define clr_P2SR_6  P2SR    &= ~SET_BIT6
#define clr_P2SR_5  P2SR    &= ~SET_BIT5
#define clr_P2SR_4  P2SR    &= ~SET_BIT4
#define clr_P2SR_3  P2SR    &= ~SET_BIT3
#define clr_P2SR_2  P2SR    &= ~SET_BIT2
#define clr_P2SR_1  P2SR    &= ~SET_BIT1
#define clr_P2SR_0  P2SR    &= ~SET_BIT0

//P2M2
#define set_P2M2_6  P2M2    |= SET_BIT6
#define set_P2M2_5  P2M2    |= SET_BIT5 
#define set_P2M2_4  P2M2    |= SET_BIT4
#define set_P2M2_3  P2M2    |= SET_BIT3
#define set_P2M2_2  P2M2    |= SET_BIT2
#define set_P2M2_1  P2M2    |= SET_BIT1
#define set_P2M2_0  P2M2    |= SET_BIT0

#define clr_P2M2_6  P2M2    &= ~SET_BIT6
#define clr_P2M2_5  P2M2    &= ~SET_BIT5
#define clr_P2M2_4  P2M2    &= ~SET_BIT4
#define clr_P2M2_3  P2M2    &= ~SET_BIT3
#define clr_P2M2_2  P2M2    &= ~SET_BIT2
#define clr_P2M2_1  P2M2    &= ~SET_BIT1
#define clr_P2M2_0  P2M2    &= ~SET_BIT0

//P2S
#define set_P2S_6   P2S     |= SET_BIT6
#define set_P2S_5   P2S     |= SET_BIT5
#define set_P2S_4   P2S     |= SET_BIT4
#define set_P2S_3   P2S     |= SET_BIT3
#define set_P2S_2   P2S     |= SET_BIT2
#define set_P2S_1   P2S     |= SET_BIT1
#define set_P2S_0   P2S     |= SET_BIT0

#define clr_P2S_6   P2S     &= ~SET_BIT6
#define clr_P2S_5   P2S     &= ~SET_BIT5
#define clr_P2S_4   P2S     &= ~SET_BIT4
#define clr_P2S_3   P2S     &= ~SET_BIT3
#define clr_P2S_2   P2S     &= ~SET_BIT2
#define clr_P2S_1   P2S     &= ~SET_BIT1
#define clr_P2S_0   P2S     &= ~SET_BIT0

//P2M1
#define set_P2M1_6  P2M1    |= SET_BIT6
#define set_P2M1_5  P2M1    |= SET_BIT5 
#define set_P2M1_4  P2M1    |= SET_BIT4
#define set_P2M1_3  P2M1    |= SET_BIT3
#define set_P2M1_2  P2M1    |= SET_BIT2
#define set_P2M1_1  P2M1    |= SET_BIT1
#define set_P2M1_0  P2M1    |= SET_BIT0

#define clr_P2M1_6  P2M1    &= ~SET_BIT6
#define clr_P2M1_5  P2M1    &= ~SET_BIT5
#define clr_P2M1_4  P2M1    &= ~SET_BIT4
#define clr_P2M1_3  P2M1    &= ~SET_BIT3
#define clr_P2M1_2  P2M1    &= ~SET_BIT2
#define clr_P2M1_1  P2M1    &= ~SET_BIT1
#define clr_P2M1_0  P2M1    &= ~SET_BIT0

//P1SR
#define set_P1SR_1  P1SR    |= SET_BIT1
#define set_P1SR_0  P1SR    |= SET_BIT0

#define clr_P1SR_1  P1SR    &= ~SET_BIT1
#define clr_P1SR_0  P1SR    &= ~SET_BIT0

//P1M2                      
#define set_CLOEN   P1M2    |= SET_BIT3
#define set_P12UP   P1M2    |= SET_BIT2
#define set_P1M2_1  P1M2    |= SET_BIT1
#define set_P1M2_0  P1M2    |= SET_BIT0
                            
#define clr_CLOEN   P1M2    &= ~SET_BIT3
#define clr_P12UP   P1M2    &= ~SET_BIT2
#define clr_P1M2_1  P1M2    &= ~SET_BIT1
#define clr_P1M2_0  P1M2    &= ~SET_BIT0

//P1S
#define set_P21SNK  P1S     |= SET_BIT7
#define set_P20SNK  P1S     |= SET_BIT6
#define set_P03SNK  P1S     |= SET_BIT5
#define set_P02SNK  P1S     |= SET_BIT4
#define set_P01SNK  P1S     |= SET_BIT3
#define set_P1S_2   P1S     |= SET_BIT2
#define set_P1S_1   P1S     |= SET_BIT1
#define set_P1S_0   P1S     |= SET_BIT0

#define clr_P21SNK  P1S     &= ~SET_BIT7
#define clr_P20SNK  P1S     &= ~SET_BIT6
#define clr_P03SNK  P1S     &= ~SET_BIT5
#define clr_P02SNK  P1S     &= ~SET_BIT4
#define clr_P01SNK  P1S     &= ~SET_BIT3
#define clr_P1S_2   P1S     &= ~SET_BIT2
#define clr_P1S_1   P1S     &= ~SET_BIT1
#define clr_P1S_0   P1S     &= ~SET_BIT0

//P1M1
#define set_T1OE    P1M1    |= SET_BIT3
#define set_T0OE    P1M1    |= SET_BIT2
#define set_P1M1_1  P1M1    |= SET_BIT1
#define set_P1M1_0  P1M1    |= SET_BIT0
                            
#define clr_T1OE    P1M1    &= ~SET_BIT3
#define clr_T0OE    P1M1    &= ~SET_BIT2
#define clr_P1M1_1  P1M1    &= ~SET_BIT1
#define clr_P1M1_0  P1M1    &= ~SET_BIT0

//P0SR
#define set_P0SR_7  P0SR    |= SET_BIT7
#define set_P0SR_6  P0SR    |= SET_BIT6
#define set_P0SR_5  P0SR    |= SET_BIT5
#define set_P0SR_4  P0SR    |= SET_BIT4
#define set_P0SR_3  P0SR    |= SET_BIT3
#define set_P0SR_2  P0SR    |= SET_BIT2
#define set_P0SR_1  P0SR    |= SET_BIT1
#define set_P0SR_0  P0SR    |= SET_BIT0

#define clr_P0SR_7  P0SR    &= ~SET_BIT7
#define clr_P0SR_6  P0SR    &= ~SET_BIT6
#define clr_P0SR_5  P0SR    &= ~SET_BIT5
#define clr_P0SR_4  P0SR    &= ~SET_BIT4
#define clr_P0SR_3  P0SR    &= ~SET_BIT3
#define clr_P0SR_2  P0SR    &= ~SET_BIT2
#define clr_P0SR_1  P0SR    &= ~SET_BIT1
#define clr_P0SR_0  P0SR    &= ~SET_BIT0

//P0M2
#define set_P0M2_7  P0M2    |= SET_BIT7
#define set_P0M2_6  P0M2    |= SET_BIT6
#define set_P0M2_5  P0M2    |= SET_BIT5 
#define set_P0M2_4  P0M2    |= SET_BIT4
#define set_P0M2_3  P0M2    |= SET_BIT3
#define set_P0M2_2  P0M2    |= SET_BIT2
#define set_P0M2_1  P0M2    |= SET_BIT1
#define set_P0M2_0  P0M2    |= SET_BIT0

#define clr_P0M2_7  P0M2    &= ~SET_BIT7
#define clr_P0M2_6  P0M2    &= ~SET_BIT6
#define clr_P0M2_5  P0M2    &= ~SET_BIT5
#define clr_P0M2_4  P0M2    &= ~SET_BIT4
#define clr_P0M2_3  P0M2    &= ~SET_BIT3
#define clr_P0M2_2  P0M2    &= ~SET_BIT2
#define clr_P0M2_1  P0M2    &= ~SET_BIT1
#define clr_P0M2_0  P0M2    &= ~SET_BIT0

//P0S
#define set_P0S_7   P0S     |= SET_BIT7
#define set_P0S_6   P0S     |= SET_BIT6
#define set_P0S_5   P0S     |= SET_BIT5
#define set_P0S_4   P0S     |= SET_BIT4
#define set_P0S_3   P0S     |= SET_BIT3
#define set_P0S_2   P0S     |= SET_BIT2
#define set_P0S_1   P0S     |= SET_BIT1
#define set_P0S_0   P0S     |= SET_BIT0

#define clr_P0S_7   P0S     &= ~SET_BIT7
#define clr_P0S_6   P0S     &= ~SET_BIT6
#define clr_P0S_5   P0S     &= ~SET_BIT5
#define clr_P0S_4   P0S     &= ~SET_BIT4
#define clr_P0S_3   P0S     &= ~SET_BIT3
#define clr_P0S_2   P0S     &= ~SET_BIT2
#define clr_P0S_1   P0S     &= ~SET_BIT1
#define clr_P0S_0   P0S     &= ~SET_BIT0

//P0M1
#define set_P0M1_7  P0M1    |= SET_BIT7
#define set_P0M1_6  P0M1    |= SET_BIT6
#define set_P0M1_5  P0M1    |= SET_BIT5 
#define set_P0M1_4  P0M1    |= SET_BIT4
#define set_P0M1_3  P0M1    |= SET_BIT3
#define set_P0M1_2  P0M1    |= SET_BIT2
#define set_P0M1_1  P0M1    |= SET_BIT1
#define set_P0M1_0  P0M1    |= SET_BIT0

#define clr_P0M1_7  P0M1    &= ~SET_BIT7
#define clr_P0M1_6  P0M1    &= ~SET_BIT6
#define clr_P0M1_5  P0M1    &= ~SET_BIT5
#define clr_P0M1_4  P0M1    &= ~SET_BIT4
#define clr_P0M1_3  P0M1    &= ~SET_BIT3
#define clr_P0M1_2  P0M1    &= ~SET_BIT2
#define clr_P0M1_1  P0M1    &= ~SET_BIT1
#define clr_P0M1_0  P0M1    &= ~SET_BIT0

//P3

//IAPCN
#define set_FOEN    IAPN    |= SET_BIT5
#define set_FCEN    IAPN    |= SET_BIT4
#define set_FCTRL3  IAPN    |= SET_BIT3
#define set_FCTRL2  IAPN    |= SET_BIT2
#define set_FCTRL1  IAPN    |= SET_BIT1
#define set_FCTRL0  IAPN    |= SET_BIT0
                            
#define clr_FOEN    IAPN    &= ~SET_BIT5
#define clr_FCEN    IAPN    &= ~SET_BIT4
#define clr_FCTRL3  IAPN    &= ~SET_BIT3
#define clr_FCTRL2  IAPN    &= ~SET_BIT2
#define clr_FCTRL1  IAPN    &= ~SET_BIT1
#define clr_FCTRL0  IAPN    &= ~SET_BIT0

//IAPFD

//P3SR
#define set_P3SR_7  P3SR    |= SET_BIT7
#define set_P3SR_6  P3SR    |= SET_BIT6
#define set_P3SR_5  P3SR    |= SET_BIT5
#define set_P3SR_4  P3SR    |= SET_BIT4
#define set_P3SR_3  P3SR    |= SET_BIT3
#define set_P3SR_2  P3SR    |= SET_BIT2
#define set_P3SR_1  P3SR    |= SET_BIT1
#define set_P3SR_0  P3SR    |= SET_BIT0

#define clr_P3SR_7  P3SR    &= ~SET_BIT7
#define clr_P3SR_6  P3SR    &= ~SET_BIT6
#define clr_P3SR_5  P3SR    &= ~SET_BIT5
#define clr_P3SR_4  P3SR    &= ~SET_BIT4
#define clr_P3SR_3  P3SR    &= ~SET_BIT3
#define clr_P3SR_2  P3SR    &= ~SET_BIT2
#define clr_P3SR_1  P3SR    &= ~SET_BIT1
#define clr_P3SR_0  P3SR    &= ~SET_BIT0

//P3M2
#define set_P3M2_7  P3M2    |= SET_BIT7
#define set_P3M2_6  P3M2    |= SET_BIT6
#define set_P3M2_5  P3M2    |= SET_BIT5 
#define set_P3M2_4  P3M2    |= SET_BIT4
#define set_P3M2_3  P3M2    |= SET_BIT3
#define set_P3M2_2  P3M2    |= SET_BIT2
#define set_P3M2_1  P3M2    |= SET_BIT1
#define set_P3M2_0  P3M2    |= SET_BIT0

#define clr_P3M2_7  P3M2    &= ~SET_BIT7
#define clr_P3M2_6  P3M2    &= ~SET_BIT6
#define clr_P3M2_5  P3M2    &= ~SET_BIT5
#define clr_P3M2_4  P3M2    &= ~SET_BIT4
#define clr_P3M2_3  P3M2    &= ~SET_BIT3
#define clr_P3M2_2  P3M2    &= ~SET_BIT2
#define clr_P3M2_1  P3M2    &= ~SET_BIT1
#define clr_P3M2_0  P3M2    &= ~SET_BIT0

//P3S
#define set_P3S_7   P3S     |= SET_BIT7
#define set_P3S_6   P3S     |= SET_BIT6
#define set_P3S_5   P3S     |= SET_BIT5
#define set_P3S_4   P3S     |= SET_BIT4
#define set_P3S_3   P3S     |= SET_BIT3
#define set_P3S_2   P3S     |= SET_BIT2
#define set_P3S_1   P3S     |= SET_BIT1
#define set_P3S_0   P3S     |= SET_BIT0

#define clr_P3S_7   P3S     &= ~SET_BIT7
#define clr_P3S_6   P3S     &= ~SET_BIT6
#define clr_P3S_5   P3S     &= ~SET_BIT5
#define clr_P3S_4   P3S     &= ~SET_BIT4
#define clr_P3S_3   P3S     &= ~SET_BIT3
#define clr_P3S_2   P3S     &= ~SET_BIT2
#define clr_P3S_1   P3S     &= ~SET_BIT1
#define clr_P3S_0   P3S     &= ~SET_BIT0

//P3M1
#define set_P3M1_7  P3M1    |= SET_BIT7
#define set_P3M1_6  P3M1    |= SET_BIT6
#define set_P3M1_5  P3M1    |= SET_BIT5 
#define set_P3M1_4  P3M1    |= SET_BIT4
#define set_P3M1_3  P3M1    |= SET_BIT3
#define set_P3M1_2  P3M1    |= SET_BIT2
#define set_P3M1_1  P3M1    |= SET_BIT1
#define set_P3M1_0  P3M1    |= SET_BIT0

#define clr_P3M1_7  P3M1    &= ~SET_BIT7
#define clr_P3M1_6  P3M1    &= ~SET_BIT6
#define clr_P3M1_5  P3M1    &= ~SET_BIT5
#define clr_P3M1_4  P3M1    &= ~SET_BIT4
#define clr_P3M1_3  P3M1    &= ~SET_BIT3
#define clr_P3M1_2  P3M1    &= ~SET_BIT2
#define clr_P3M1_1  P3M1    &= ~SET_BIT1
#define clr_P3M1_0  P3M1    &= ~SET_BIT0

//BODCON1
#define set_LPBOD1  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1 |= SET_BIT2 ;EA=BIT_TMP;
#define set_LPBOD0  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1 |= SET_BIT1 ;EA=BIT_TMP;
#define set_BODFLT  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1 |= SET_BIT0 ;EA=BIT_TMP;
        
#define clr_LPBOD1  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1 &= ~SET_BIT2;EA=BIT_TMP;
#define clr_LPBOD0  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1 &= ~SET_BIT1;EA=BIT_TMP;
#define clr_BODFLT  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1 &= ~SET_BIT0;EA=BIT_TMP;

//WDCON
#define set_WDTEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT7 ;EA=BIT_TMP;
#define set_WDCLR   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT6 ;EA=BIT_TMP;
#define set_WDTF    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT5 ;EA=BIT_TMP;
#define set_WDTRF   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT3 ;EA=BIT_TMP;
#define set_WPS2    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT2 ;EA=BIT_TMP;
#define set_WPS1    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT1 ;EA=BIT_TMP;
#define set_WPS0    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON |= SET_BIT0 ;EA=BIT_TMP;
        
#define clr_WDTEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT7;EA=BIT_TMP;
#define clr_WDCLR   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT6;EA=BIT_TMP;
#define clr_WDTF    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT5;EA=BIT_TMP;
#define clr_WDTRF   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT3;EA=BIT_TMP;
#define clr_WPS2    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT2;EA=BIT_TMP;
#define clr_WPS1    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT1;EA=BIT_TMP;
#define clr_WPS0    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON &= ~SET_BIT0;EA=BIT_TMP;

//SADDR

//IE
#define set_EA      EA       = 1
#define set_EADC    EADC     = 1
#define set_EBOD    EBOD     = 1
#define set_ES      ES       = 1
#define set_ET1     ET1      = 1
#define set_EX1     EX1      = 1
#define set_ET0     ET0      = 1
#define set_EX0     EX0      = 1

#define clr_EA      EA       = 0
#define clr_EADC    EADC     = 0
#define clr_EBOD    EBOD     = 0
#define clr_ES      ES       = 0
#define clr_ET1     ET1      = 0
#define clr_EX1     EX1      = 0
#define clr_ET0     ET0      = 0
#define clr_EX0     EX0      = 0

//IAPAH

//IAPAL

//IAPUEN
#define set_CFUEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN |= SET_BIT2 ;EA=BIT_TMP;
#define set_LDUEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN |= SET_BIT1 ;EA=BIT_TMP;
#define set_APUEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN |= SET_BIT0 ;EA=BIT_TMP;

#define clr_CFUEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN &= ~SET_BIT2;EA=BIT_TMP;
#define clr_LDUEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN &= ~SET_BIT1;EA=BIT_TMP;
#define clr_APUEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN &= ~SET_BIT0;EA=BIT_TMP;

//IAPTRG
#define set_IAPGO   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPTRG |= SET_BIT0 ;EA=BIT_TMP
#define clr_IAPGO   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPTRG &= ~SET_BIT0;EA=BIT_TMP

//BODCON0
#define set_BODEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT7;EA=BIT_TMP;
#define set_BOV2    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT6;EA=BIT_TMP;
#define set_BOV1    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT5;EA=BIT_TMP;
#define set_BOV0    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT4;EA=BIT_TMP;
#define set_BOF     BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT3;EA=BIT_TMP;
#define set_BORST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT2;EA=BIT_TMP;
#define set_BORF    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT1;EA=BIT_TMP;
#define set_BOS     BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 |= SET_BIT0;EA=BIT_TMP;

#define clr_BODEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT7;EA=BIT_TMP;
#define clr_BOV2    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT6;EA=BIT_TMP;
#define clr_BOV1    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT5;EA=BIT_TMP;
#define clr_BOV0    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT4;EA=BIT_TMP;
#define clr_BOF     BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT3;EA=BIT_TMP;
#define clr_BORST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT2;EA=BIT_TMP;
#define clr_BORF    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT1;EA=BIT_TMP;
#define clr_BOS     BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0 &= ~SET_BIT0;EA=BIT_TMP;

//AUXR1
#define set_SWRF    AUXR1   |= SET_BIT7
#define set_RSTPINF AUXR1   |= SET_BIT6
#define set_T1EXIN  AUXR1   |= SET_BIT5
#define set_T0EXIN  AUXR1   |= SET_BIT4
#define set_GF2     AUXR1   |= SET_BIT3
#define set_UART0PX AUXR1   |= SET_BIT2
#define set_DPS     AUXR1   |= SET_BIT0
                            
#define clr_SWRF    AUXR1   &= ~SET_BIT7
#define clr_RSTPINF AUXR1   &= ~SET_BIT6
#define clr_T1EXIN  AUXR1   &= ~SET_BIT5
#define clr_T0EXIN  AUXR1   &= ~SET_BIT4
#define clr_GF2     AUXR1   &= ~SET_BIT3
#define clr_UART0PX AUXR1   &= ~SET_BIT2
#define clr_DPS     AUXR1   &= ~SET_BIT0

//AUXR
#define set_IOTEST  AUXR    |= SET_BIT0
#define clr_IOTEST  AUXR    &= ~SET_BIT0

//P2

//CHPCON
#define set_SWRST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON |= SET_BIT7 ;EA=BIT_TMP;
#define set_IAPFF   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON |= SET_BIT6 ;EA=BIT_TMP;
#define set_BS      BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON |= SET_BIT1 ;EA=BIT_TMP;
#define set_IAPEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON |= SET_BIT0 ;EA=BIT_TMP;
       
#define clr_SWRST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON &= ~SET_BIT7;EA=BIT_TMP;
#define clr_IAPFF   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON &= ~SET_BIT6;EA=BIT_TMP;
#define clr_BS      BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON &= ~SET_BIT1;EA=BIT_TMP;
#define clr_IAPEN   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON &= ~SET_BIT0;EA=BIT_TMP;

//RSR

//EIE1                      
#define set_EWKT    EIE1    |= SET_BIT2
#define set_ET3     EIE1    |= SET_BIT1
#define set_ES_1    EIE1    |= SET_BIT0
                            
#define clr_EWKT    EIE1    &= ~SET_BIT2
#define clr_ET3     EIE1    &= ~SET_BIT1
#define clr_ES_1    EIE1    &= ~SET_BIT0
                            
//EIE                       
#define set_ET2     EIE     |= SET_BIT7
#define set_ESPI    EIE     |= SET_BIT6
#define set_EFB     EIE     |= SET_BIT5
#define set_EWDT    EIE     |= SET_BIT4
#define set_EPWM    EIE     |= SET_BIT3
#define set_ECAP    EIE     |= SET_BIT2
#define set_EPI     EIE     |= SET_BIT1
#define set_EI2C    EIE     |= SET_BIT0
                            
#define clr_ET2     EIE     &= ~SET_BIT7
#define clr_ESPI    EIE     &= ~SET_BIT6
#define clr_EFB     EIE     &= ~SET_BIT5
#define clr_EWDT    EIE     &= ~SET_BIT4
#define clr_EPWM    EIE     &= ~SET_BIT3
#define clr_ECAP    EIE     &= ~SET_BIT2
#define clr_EPI     EIE     &= ~SET_BIT1
#define clr_EI2C    EIE     &= ~SET_BIT0

//SBUF_1

//SBUF

//SCON
#define set_FE      FE    = 1
#define set_SM1     SM1   = 1
#define set_SM2     SM2   = 1
#define set_REN     REN   = 1
#define set_TB8     TB8   = 1
#define set_RB8     RB8   = 1
#define set_TI      TI    = 1
#define set_RI      RI    = 1

#define clr_FE      FE    = 0
#define clr_SM1     SM1   = 0
#define clr_SM2     SM2   = 0
#define clr_REN     REN   = 0
#define clr_TB8     TB8   = 0
#define clr_RB8     RB8   = 0
#define clr_TI      TI    = 0
#define clr_RI      RI    = 0

//CKEN 
#define set_EXTEN1  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  |= SET_BIT7  ;EA=BIT_TMP;
#define set_EXTEN0  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  |= SET_BIT6  ;EA=BIT_TMP;
#define set_HIRCEN  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  |= SET_BIT5  ;EA=BIT_TMP;
#define set_LIRCEN  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  |= SET_BIT4  ;EA=BIT_TMP;
#define set_CKSWTF  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  |= SET_BIT0  ;EA=BIT_TMP;
       
#define clr_EXTEN1  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  &= ~SET_BIT7 ;EA=BIT_TMP;
#define clr_EXTEN0  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  &= ~SET_BIT6 ;EA=BIT_TMP;
#define clr_HIRCEN  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  &= ~SET_BIT5 ;EA=BIT_TMP;
#define clr_LIRCEN  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  &= ~SET_BIT4 ;EA=BIT_TMP;
#define clr_CKSWTF  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN  &= ~SET_BIT0 ;EA=BIT_TMP;

//CKSWT
#define set_HXTST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT7  ;EA=BIT_TMP;
#define set_LXTST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT6  ;EA=BIT_TMP;
#define set_HIRCST  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT5  ;EA=BIT_TMP;
#define set_LIRCST  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT4  ;EA=BIT_TMP;
#define set_ECLKST  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT3  ;EA=BIT_TMP;
#define set_OSC1    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT2  ;EA=BIT_TMP;
#define set_OSC0    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT |= SET_BIT1  ;EA=BIT_TMP;

#define clr_HXTST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT7  ;EA=BIT_TMP;
#define clr_LXTST   BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT6  ;EA=BIT_TMP;
#define clr_HIRCST  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT5  ;EA=BIT_TMP;
#define clr_LIRCST  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT4  ;EA=BIT_TMP;
#define clr_ECLKST  BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT3  ;EA=BIT_TMP;
#define clr_OSC1    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT2  ;EA=BIT_TMP;
#define clr_OSC0    BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT &= ~SET_BIT1  ;EA=BIT_TMP;

//CKDIV

//CAPCON2
#define set_ENF2  CAPCON2   |= SET_BIT6
#define set_ENF1  CAPCON2   |= SET_BIT5
#define set_ENF0  CAPCON2   |= SET_BIT4
                            
#define clr_ENF2  CAPCON2   &= ~SET_BIT6
#define clr_ENF1  CAPCON2   &= ~SET_BIT5
#define clr_ENF0  CAPCON2   &= ~SET_BIT4

//CAPCON1
#define set_CAP2LS1 CAPCON1 |= SET_BIT5
#define set_CAP2LS0 CAPCON1 |= SET_BIT4
#define set_CAP1LS1 CAPCON1 |= SET_BIT3
#define set_CAP1LS0 CAPCON1 |= SET_BIT2
#define set_CAP0LS1 CAPCON1 |= SET_BIT1
#define set_CAP0LS0 CAPCON1 |= SET_BIT0

#define clr_CAP2LS1 CAPCON1 &= ~SET_BIT5
#define clr_CAP2LS0 CAPCON1 &= ~SET_BIT4
#define clr_CAP1LS1 CAPCON1 &= ~SET_BIT3
#define clr_CAP1LS0 CAPCON1 &= ~SET_BIT2
#define clr_CAP0LS1 CAPCON1 &= ~SET_BIT1
#define clr_CAP0LS0 CAPCON1 &= ~SET_BIT0

//CAPCON0
#define set_CAPEN2  CAPCON0 |= SET_BIT6
#define set_CAPEN1  CAPCON0 |= SET_BIT5
#define set_CAPEN0  CAPCON0 |= SET_BIT4
#define set_CAPF2   CAPCON0 |= SET_BIT2
#define set_CAPF1   CAPCON0 |= SET_BIT1
#define set_CAPF0   CAPCON0 |= SET_BIT0

#define clr_CAPEN2  CAPCON0 &= ~SET_BIT6
#define clr_CAPEN1  CAPCON0 &= ~SET_BIT5
#define clr_CAPEN0  CAPCON0 &= ~SET_BIT4
#define clr_CAPF2   CAPCON0 &= ~SET_BIT2
#define clr_CAPF1   CAPCON0 &= ~SET_BIT1
#define clr_CAPF0   CAPCON0 &= ~SET_BIT0

//SFRS
#define set_SFRSEL  SFRS    |= SET_BIT0
#define clr_SFRSEL  SFRS    &= ~SET_BIT0

//P1

//WKCON
#define set_WKTCK   WKCON   |= SET_BIT5
#define set_WKTF    WKCON   |= SET_BIT4
#define set_WKTR    WKCON   |= SET_BIT3
#define set_WKPS2   WKCON   |= SET_BIT2
#define set_WKPS1   WKCON   |= SET_BIT1
#define set_WKPS0   WKCON   |= SET_BIT0
                            
#define clr_WKTCK   WKCON   &= ~SET_BIT5
#define clr_WKTF    WKCON   &= ~SET_BIT4
#define clr_WKTR    WKCON   &= ~SET_BIT3
#define clr_WKPS2   WKCON   &= ~SET_BIT2
#define clr_WKPS1   WKCON   &= ~SET_BIT1
#define clr_WKPS0   WKCON   &= ~SET_BIT0

//CKCON
#define set_PWMCKS  CKCON   |= SET_BIT6
#define set_T1M     CKCON   |= SET_BIT4
#define set_T0M     CKCON   |= SET_BIT3
                            
#define clr_PWMCKS  CKCON   &= ~SET_BIT6
#define clr_T1M     CKCON   &= ~SET_BIT4
#define clr_T0M     CKCON   &= ~SET_BIT3

//TH1

//TH0

//TL1

//TL0

//TMOD
#define set_GATE_T1 TMOD |= SET_BIT7
#define set_CT_T1 TMOD   |= SET_BIT6
#define set_M1_T1 TMOD   |= SET_BIT5
#define set_M0_T1 TMOD   |= SET_BIT4
#define set_GATE_T0 TMOD |= SET_BIT3
#define set_CT_T0 TMOD   |= SET_BIT2
#define set_M1_T0 TMOD   |= SET_BIT1
#define set_M0_T0 TMOD   |= SET_BIT0
                            
#define clr_GATE_T1 TMOD &= ~SET_BIT7
#define clr_CT_T1 TMOD   &= ~SET_BIT6
#define clr_M1_T1 TMOD   &= ~SET_BIT5
#define clr_M0_T1 TMOD   &= ~SET_BIT4
#define clr_GATE_T0 TMOD &= ~SET_BIT3
#define clr_CT_T0 TMOD   &= ~SET_BIT2
#define clr_M1_T0 TMOD   &= ~SET_BIT1
#define clr_M0_T0 TMOD   &= ~SET_BIT0

//TCON
#define set_TF1     TF1      = 1
#define set_TR1     TR1      = 1
#define set_TF0     TF0      = 1
#define set_TR0     TR0      = 1
#define set_IE1     IE1      = 1
#define set_IT1     IT1      = 1
#define set_IE0     IE0      = 1
#define set_IT0     IT0      = 1

#define clr_TF1     TF1      = 0
#define clr_TR1     TR1      = 0
#define clr_TF0     TF0      = 0
#define clr_TR0     TR0      = 0
#define clr_IE1     IE1      = 0
#define clr_IT1     IT1      = 0
#define clr_IE0     IE0      = 0
#define clr_IT0     IT0      = 0

//PCON
#define set_SMOD    PCON    |= SET_BIT7
#define set_SMOD0   PCON    |= SET_BIT6
#define set_POF     PCON    |= SET_BIT4
#define set_GF1     PCON    |= SET_BIT3
#define set_GF0     PCON    |= SET_BIT2 
#define set_PD      PCON    |= SET_BIT1
#define set_IDLE    PCON    |= SET_BIT0
                            
#define clr_SMOD    PCON    &= ~SET_BIT7
#define clr_SMOD0   PCON    &= ~SET_BIT6
#define clr_POF     PCON    &= ~SET_BIT4
#define clr_GF1     PCON    &= ~SET_BIT3
#define clr_GF0     PCON    &= ~SET_BIT2 
#define clr_PD      PCON    &= ~SET_BIT1
#define clr_IDLE    PCON    &= ~SET_BIT0

//RWK

//DPH

//DPL

//SP

//P0
#define set_P07     P07      = 1
#define set_P06     P06      = 1
#define set_P05     P05      = 1
#define set_P04     P04      = 1
#define set_P03     P03      = 1
#define set_P02     P02      = 1
#define set_P01     P01      = 1
#define set_P00     P00      = 1

#define clr_P07     P07      = 0
#define clr_P06     P06      = 0
#define clr_P05     P05      = 0
#define clr_P04     P04      = 0
#define clr_P03     P03      = 0
#define clr_P02     P02      = 0
#define clr_P01     P01      = 0
#define clr_P00     P00      = 0

//P1
#define set_P12     P12      = 1
#define set_P11     P11      = 1
#define set_P10     P10      = 1

#define clr_P12     P12      = 0
#define clr_P11     P11      = 0
#define clr_P10     P10      = 0

//P2
#define set_P26     P26      = 1
#define set_P25     P25      = 1
#define set_P24     P24      = 1
#define set_P23     P23      = 1
#define set_P22     P22      = 1
#define set_P21     P21      = 1
#define set_P20     P20      = 1

#define clr_P26     P26      = 0
#define clr_P25     P25      = 0
#define clr_P24     P24      = 0
#define clr_P23     P23      = 0
#define clr_P22     P22      = 0
#define clr_P21     P21      = 0
#define clr_P20     P20      = 0

//P3
#define set_P37     P37      = 1
#define set_P36     P36      = 1
#define set_P35     P35      = 1
#define set_P34     P34      = 1
#define set_P33     P33      = 1
#define set_P32     P32      = 1
#define set_P31     P31      = 1
#define set_P30     P30      = 1

#define clr_P37     P37      = 0
#define clr_P36     P36      = 0
#define clr_P35     P35      = 0
#define clr_P34     P34      = 0
#define clr_P33     P33      = 0
#define clr_P32     P32      = 0
#define clr_P31     P31      = 0
#define clr_P30     P30      = 0