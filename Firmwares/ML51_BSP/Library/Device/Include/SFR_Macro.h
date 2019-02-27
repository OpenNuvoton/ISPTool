typedef bit                   BIT;
typedef unsigned char         UINT8;
typedef unsigned int          UINT16;
typedef unsigned long         UINT32;

typedef unsigned char         uint8_t;
typedef unsigned int          uint16_t;
typedef unsigned long         uint32_t;

#define Disable    0
#define Enable     1

#define CHK_BIT0              0x01
#define CHK_BIT1              0x02
#define CHK_BIT2              0x04
#define CHK_BIT3              0x08
#define CHK_BIT4              0x10
#define CHK_BIT5              0x20
#define CHK_BIT6              0x40
#define CHK_BIT7              0x80
#define CHK_BIT8              0x0100
#define CHK_BIT9              0x0200
#define CHK_BIT10             0x0400
#define CHK_BIT11             0x0800
#define CHK_BIT12             0x1000
#define CHK_BIT13             0x2000
#define CHK_BIT14             0x4000
#define CHK_BIT15             0x8000

#define SET_BIT0              0x01
#define SET_BIT1              0x02
#define SET_BIT2              0x04
#define SET_BIT3              0x08
#define SET_BIT4              0x10
#define SET_BIT5              0x20
#define SET_BIT6              0x40
#define SET_BIT7              0x80
#define SET_BIT8              0x0100
#define SET_BIT9              0x0200
#define SET_BIT10             0x0400
#define SET_BIT11             0x0800
#define SET_BIT12             0x1000
#define SET_BIT13             0x2000
#define SET_BIT14             0x4000
#define SET_BIT15             0x8000

#define CLR_BIT0              0xFE
#define CLR_BIT1              0xFD
#define CLR_BIT2              0xFB
#define CLR_BIT3              0xF7
#define CLR_BIT4              0xEF
#define CLR_BIT5              0xDF
#define CLR_BIT6              0xBF
#define CLR_BIT7              0x7F
                              
#define CLR_BIT8              0xFEFF
#define CLR_BIT9              0xFDFF
#define CLR_BIT10             0xFBFF
#define CLR_BIT11             0xF7FF
#define CLR_BIT12             0xEFFF
#define CLR_BIT13             0xDFFF
#define CLR_BIT14             0xBFFF
#define CLR_BIT15             0x7FFF

//16 --> 8 x 2
#define HIBYTE(v1)              ((UINT8)((v1)>>8))                      //v1 is UINT16
#define LOBYTE(v1)              ((UINT8)((v1)&0xFF))
//8 x 2 --> 16
#define MAKEWORD(v1,v2)         ((((UINT16)(v1))<<8)+(UINT16)(v2))      //v1,v2 is UINT8
//8 x 4 --> 32
#define MAKELONG(v1,v2,v3,v4)   (UINT32)((v1<<32)+(v2<<16)+(v3<<8)+v4)  //v1,v2,v3,v4 is UINT8
//32 --> 16 x 2
#define YBYTE1(v1)              ((UINT16)((v1)>>16))                    //v1 is UINT32
#define YBYTE0(v1)              ((UINT16)((v1)&0xFFFF))
//32 --> 8 x 4
#define TBYTE3(v1)              ((UINT8)((v1)>>24))                     //v1 is UINT32
#define TBYTE2(v1)              ((UINT8)((v1)>>16))
#define TBYTE1(v1)              ((UINT8)((v1)>>8)) 
#define TBYTE0(v1)              ((UINT8)((v1)&0xFF))



//Order of SFR
//FFH --> 80H, page0----------------------------------------------------------------------
//      0/8      1/9       2/A       3/B       4/C       5/D       6/E       7/F
//0F8H  S1CON    SPI1CR0   SPI1CR1   SPI1SR    SPI1DR    DMA1BAH   EIP1      EIPH1
//0F0H  B        DMA1TSR   MTM1DA    SPI0CR0   SPI0SR    SPI0DR    DMA0BAH   EIPH0
//0E8H  I2C1CON  DMA0TSR   MTM0DA    DMA1CR    DMA1MA    DMA1CNT   DMA1CCNT  EIP0
//0E0H  ACC      ADCCON1   ADCCON2   ADCDLY    ADCBAH    ADCSN     ADCCN     ADCSR
//0D8H  P4       SC0DR     SC0EGT    SC0ETURD0 SC0ETURD1 SC0IE     SC0IS     SC0TSR
//0D0H  PSW      PWM0CON0  ACMPCR0   ACMPCR1   ACMPSR    ACMPVREF  SC0CR0    SC0CR1
//0C8H  T2CON    T2MOD     PIF       ADCBAL    TL2       TH2       ADCMPL    ADCMPH
//0C0H  I2C0CON  I2C0ADDR  ADCRL     ADCRH     T3CON     RL3       RH3 TA
//0B8H  IP       SADEN     SADEN1    SADDR1    I2C0DAT   I2C0STAT  I2C0CLK   I2C0TOC
//0B0H  P3       P5        I2C1ADDR  I2C1DAT   I2C1STAT  I2C1CLK   I2C1TOC   IPH
//0A8H  IE       SADDR     WDCON     BODCON1   EIP2      EIPH2     IAPFD     IAPCN
//0A0H  P2       ADCCON0   AUXR0     BODCON0   IAPTRG    IAPUEN    IAPAL     IAPAH
//098H  SCON     SBUF      SBUF1     EIE0      EIE1      RSR       IAPTC     CHPCON
//090H  P1       SFRS      DMA0CR    DMA0MA    DMA0CNT   DMA0CCNT  CKSWT     CKEN
//088H  TCON     TMOD      TL0       TL1       TH0       TH1       CKCON     WKCON
//080H  P0       SP        DPL       DPH       RCTRIM0   RCTRIM1   RWK       PCON

//FFH --> 80H, page1----------------------------------------------------------------------
//      0/8      1/9       2/A       3/B       4/C       5/D       6/E       7/F
//1F8H  S1CON    PWM0DTEN  PWM0DTCNT PWM0MEN   PWM0MD   LVRFLTEN   RCNT      LVRDIS
//1F0H  B        CAPCON3   CAPCON4   SPI0CR1   AUXR2    LDOOEN     ROMMAP11  -
//1E8H  I2C1CON   PICON    PINEN     PIPEN     BCKCON   C2L        C2H       LDOTRIM
//1E0H  ACC      CAPCON0   CAPCON1   CAPCON2   C0L      C0H        C1L       C1H
//1D8H  P4       PWM0PL    PWM0C0L   PWM0C1L   PWM0C2L  PWM0C3L    PWM0IOCON PWM0CON1
//1D0H  PSW      PWM0PH    PWM0C0H   PWM0C1H   PWM0C2H  PWM0C3H    PWM0NP    PWM0FBD
//1C8H  T2CON    AUXR1     RCMP2L    RCMP2H    PWM0C4L  PWM0C5L    AINDIDS   BIASTEST
//1C0H  I2C0CON  CKDIV     P3M1      P3M2      PWM0C4H  PWM0C5H    PORDIS    TA
//1B8H  IP       P4M1      P4M2      P4S       P4SR     P5M1       P5M2      P5S
//1B0H  P3       P0M1      P0M2      P1M1      P1M2     P2M1       P2M2      PWM0INTC
//1A8H  IE       VRFCON    VRFTRIM   ACMPCR2   P3S      P3SR       P5SR      PIPS7
//1A0H  P2       PIPS0     PIPS1     PIPS2     PIPS3    PIPS4      PIPS5     PIPS6
//198H  SCON     P0S       P0SR      P1S       P1SR     P2S        P2SR      CHPCON
//190H  P1       SFRS      P0UP      P1UP      P2UP     P3UP       P4UP      P5UP
//188H  TCON     P0DW      P1DW      P2DW      P3DW     P4DW       P5DW
//180H  P0       SP        DPL       DPH       LRCTRIM  XLTCON     CWK       PCON

//FFH --> 80H, page2----------------------------------------------------------------------
//      0/8      1/9       2/A       3/B       4/C       5/D       6/E       7/F
//2F8H  S1CON    P0MF01   P0MF23    P0MF45    P0MF67    P1MF01   P1MF23     P1MF45
//2F0H  B        P1MF67   P2MF01    P2MF23    P2MF45    P2MF67   P3MF01     P3MF23
//2E8H  I2C1CON  P3MF45   P3MF67    P4MF01    P4MF23    P4MF45   P4MF67     P5MF01
//2E0H  ACC      P5MF23   P5MF45    P5MF67    SC1CR0    SC1CR1
//2D8H  P4       SC1DR    SC1EGT    SC1ETURD0 SC1ETURD1 SC1IE    SC1IS      SC1TSR
//2D0H  PSW      PWM1PL   PWM1C0L   PWM1C1L   PWM1C2L   PWM1C3L  PWM1C4L    PWM1C5L
//2C8H  T2CON    PWM1PH   PWM1C0H   PWM1C1H   PWM1C2H   PWM1C3H  PWM1C4H    PWM1C5H
//2C0H  I2C0CON  PWM1DTEN PWM1DTCNT PWM1MEN   PWM1MD    PWM1CON0 PWM1CON1   TA
//2B8H  IP      PWM1IOCON PWM1NP    PWM1FBD   PWM1INTC
//2B0H  P3       DMA2TSR  DMA2BAH   DMA2CR    DMA2MA    DMA2CNT  DMA2CCNT   MTM2DA
//2A8H  IE       DMA3TSR  DMA3BAH   DMA3CR    DMA3MA    DMA3CNT  DMA3CCNT   MTM3DA
//2A0H  P2      
//298H  SCON                                                                CHPCON
//290H  P1       SFRS
//288H  TCON    

/**********************/
/*	PAGE0         */
/**********************/
// ;---------------------------------------------
// ; S1CON 
// ;---------------------------------------------                  
#define set_S1CON_FE_1                   SFRS=0;S1CON|=SET_BIT7 
#define set_S1CON_SM1_1                  SFRS=0;S1CON|=SET_BIT6 
#define set_S1CON_SM2_1                  SFRS=0;S1CON|=SET_BIT5 
#define set_S1CON_REN_1                  SFRS=0;S1CON|=SET_BIT4 
#define set_S1CON_TB8_1                  SFRS=0;S1CON|=SET_BIT3 
#define set_S1CON_RB8_1                  SFRS=0;S1CON|=SET_BIT2 
#define set_S1CON_TI_1                   SFRS=0;S1CON|=SET_BIT1 
#define set_S1CON_RI_1                   SFRS=0;S1CON|=SET_BIT0 
                                         
#define clr_S1CON_FE_1                   SFRS=0;S1CON&=CLR_BIT7
#define clr_S1CON_SM1_1                  SFRS=0;S1CON&=CLR_BIT6
#define clr_S1CON_SM2_1                  SFRS=0;S1CON&=CLR_BIT5
#define clr_S1CON_REN_1                  SFRS=0;S1CON&=CLR_BIT4
#define clr_S1CON_TB8_1                  SFRS=0;S1CON&=CLR_BIT3
#define clr_S1CON_RB8_1                  SFRS=0;S1CON&=CLR_BIT2
#define clr_S1CON_TI_1                   SFRS=0;S1CON&=CLR_BIT1
#define clr_S1CON_RI_1                   SFRS=0;S1CON&=CLR_BIT0
// ;---------------------------------------------  
//; SPI1CR0
// ;---------------------------------------------   
#define set_SPI1CR0_SSOE                 SFRS=0;SPI1CR0|=SET_BIT7
#define set_SPI1CR0_SPIEN                SFRS=0;SPI1CR0|=SET_BIT6
#define set_SPI1CR0_LSBFE                SFRS=0;SPI1CR0|=SET_BIT5
#define set_SPI1CR0_MSTR                 SFRS=0;SPI1CR0|=SET_BIT4
#define set_SPI1CR0_CPOL                 SFRS=0;SPI1CR0|=SET_BIT3
#define set_SPI1CR0_CPHA                 SFRS=0;SPI1CR0|=SET_BIT2
                                             
#define clr_SPI1CR0_SSOE                 SFRS=0;SPI1CR0&=CLR_BIT7
#define clr_SPI1CR0_SPIEN                SFRS=0;SPI1CR0&=CLR_BIT6
#define clr_SPI1CR0_LSBFE                SFRS=0;SPI1CR0&=CLR_BIT5
#define clr_SPI1CR0_MSTR                 SFRS=0;SPI1CR0&=CLR_BIT4
#define clr_SPI1CR0_CPOL                 SFRS=0;SPI1CR0&=CLR_BIT3      
#define clr_SPI1CR0_CPHA                 SFRS=0;SPI1CR0&=CLR_BIT2
// ;---------------------------------------------
// ; SPI1CR1
// ;---------------------------------------------
#define set_SPI1CR1_TXDMAEN              SFRS=0;SPI1CR1|=SET_BIT3
#define set_SPI1CR1_RXDMAEN              SFRS=0;SPI1CR1|=SET_BIT2
                                         
#define	clr_SPI1CR1_TXDMAEN              SFRS=0;SPI1CR1&=CLR_BIT3
#define	clr_SPI1CR1_RXDMAEN              SFRS=0;SPI1CR1&=CLR_BIT2
// ;---------------------------------------------
//SPI1SR
// ;---------------------------------------------
#define set_SPI1SR_SPIF                  SFRS=0;SPI1SR|=SET_BIT7
#define set_SPI1SR_WCOL                  SFRS=0;SPI1SR|=SET_BIT6
#define set_SPI1SR_SPIOVF                SFRS=0;SPI1SR|=SET_BIT5
#define set_SPI1SR_MODF                  SFRS=0;SPI1SR|=SET_BIT4
#define set_SPI1SR_DISMODF               SFRS=0;SPI1SR|=SET_BIT3
#define set_SPI1SR_DISSPIF               SFRS=0;SPI1SR|=SET_BIT2
#define set_SPI1SR_TXBF                  SFRS=0;SPI1SR|=SET_BIT1
                                
#define clr_SPI1SR_SPIF                  SFRS=0;SPI1SR&=CLR_BIT7
#define clr_SPI1SR_WCOL                  SFRS=0;SPI1SR&=CLR_BIT6
#define clr_SPI1SR_SPIOVF                SFRS=0;SPI1SR&=CLR_BIT5
#define clr_SPI1SR_MODF                  SFRS=0;SPI1SR&=CLR_BIT4
#define clr_SPI1SR_DISMODF               SFRS=0;SPI1SR&=CLR_BIT3
#define clr_SPI1SR_DISSPIF               SFRS=0;SPI1SR&=CLR_BIT2
#define clr_SPI1SR_TXBFF                 SFRS=0;SPI1SR&=CLR_BIT1
// ;---------------------------------------------
// ; SPI1DR
// ;---------------------------------------------  
// ;---------------------------------------------
// ; DMA1BAH
// ;---------------------------------------------    
// ;---------------------------------------------     
// ; EIP1
// ;---------------------------------------------
#define set_EIP1_PWKT                     SFRS=0;EIP1|=SET_BIT2
#define set_EIP1_PT3                      SFRS=0;EIP1|=SET_BIT1
#define set_EIP1_PS_1                     SFRS=0;EIP1|=SET_BIT0

#define clr_EIP1_PWKT                     SFRS=0;EIP1&=CLR_BIT2
#define clr_EIP1_PT3                      SFRS=0;EIP1&=CLR_BIT1
#define clr_EIP1_PS_1                     SFRS=0;EIP1&=CLR_BIT0
// ;---------------------------------------------
// ; EIPH1
// ;---------------------------------------------
#define set_EIPH1_PWKTH                   SFRS=0;EIPH1|=SET_BIT2
#define set_EIPH1_PT3H                    SFRS=0;EIPH1|=SET_BIT1
#define set_EIPH1_PSH_1                   SFRS=0;EIPH1|=SET_BIT0

#define clr_EIPH1_PWKTH                   SFRS=0;EIPH1&=CLR_BIT2
#define clr_EIPH1_PT3H                    SFRS=0;EIPH1&=CLR_BIT1
#define clr_EIPH1_PSH_1                   SFRS=0;EIPH1&=CLR_BIT0
// ;---------------------------------------------
// ; DMA1TSR
// ;---------------------------------------------
#define clr_DMA1TSR_HDONE                 SFRS=0;DMA1TSR&=CLR_BIT2
#define clr_DMA1TSR_FDONE                 SFRS=0;DMA1TSR&=CLR_BIT1
// ;---------------------------------------------
// ; MTM1DA
// ;---------------------------------------------
// ;---------------------------------------------  
//; SPI0CR0
// ;---------------------------------------------   
#define set_SPI0CR0_SSOE                  SFRS=0;SPI0CR0|=SET_BIT7
#define set_SPI0CR0_SPIEN                 SFRS=0;SPI0CR0|=SET_BIT6
#define set_SPI0CR0_LSBFE                 SFRS=0;SPI0CR0|=SET_BIT5
#define set_SPI0CR0_MSTR                  SFRS=0;SPI0CR0|=SET_BIT4
#define set_SPI0CR0_CPOL                  SFRS=0;SPI0CR0|=SET_BIT3
#define set_SPI0CR0_CPHA                  SFRS=0;SPI0CR0|=SET_BIT2
                                  
#define clr_SPI0CR0_SSOE                  SFRS=0;SPI0CR0&=CLR_BIT7
#define clr_SPI0CR0_SPIEN                 SFRS=0;SPI0CR0&=CLR_BIT6
#define clr_SPI0CR0_LSBFE                 SFRS=0;SPI0CR0&=CLR_BIT5
#define clr_SPI0CR0_MSTR                  SFRS=0;SPI0CR0&=CLR_BIT4
#define clr_SPI0CR0_CPOL                  SFRS=0;SPI0CR0&=CLR_BIT3      
#define clr_SPI0CR0_CPHA                  SFRS=0;SPI0CR0&=CLR_BIT2
// ;---------------------------------------------
// ; SPI0SR
// ;---------------------------------------------
#define set_SPI0SR_SPIF                   SFRS=0;SPI0SR|=SET_BIT7
#define set_SPI0SR_WCOL                   SFRS=0;SPI0SR|=SET_BIT6
#define set_SPI0SR_SPIOVF                 SFRS=0;SPI0SR|=SET_BIT5
#define set_SPI0SR_MODF                   SFRS=0;SPI0SR|=SET_BIT4
#define set_SPI0SR_DISMODF                SFRS=0;SPI0SR|=SET_BIT3
#define set_SPI0SR_DISSPIF                SFRS=0;SPI0SR|=SET_BIT2
#define set_SPI0SR_TXBFF                  SFRS=0;SPI0SR|=SET_BIT1
                                
#define clr_SPI0SR_SPIF                   SFRS=0;SPI0SR&=CLR_BIT7
#define clr_SPI0SR_WCOL                   SFRS=0;SPI0SR&=CLR_BIT6
#define clr_SPI0SR_SPIOVF                 SFRS=0;SPI0SR&=CLR_BIT5
#define clr_SPI0SR_MODF                   SFRS=0;SPI0SR&=CLR_BIT4
#define clr_SPI0SR_DISMODF                SFRS=0;SPI0SR&=CLR_BIT3
#define clr_SPI0SR_DISSPIF                SFRS=0;SPI0SR&=CLR_BIT2
#define clr_SPI0SR_TXBFF                  SFRS=0;SPI0SR&=CLR_BIT1
// ;---------------------------------------------
// ; SPI0DR
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA0BAH
// ;---------------------------------------------
// ;---------------------------------------------
// ; EIPH0
// ;---------------------------------------------
#define set_EIPH0_PT2H                    SFRS=0;EIPH0|=SET_BIT7
#define set_EIPH0_PSPI0H                  SFRS=0;EIPH0|=SET_BIT6
#define set_EIPH0_PFBH                    SFRS=0;EIPH0|=SET_BIT5
#define set_EIPH0_PWDTH                   SFRS=0;EIPH0|=SET_BIT4
#define set_EIPH0_PPWM0H                  SFRS=0;EIPH0|=SET_BIT3
#define set_EIPH0_PCAPH                   SFRS=0;EIPH0|=SET_BIT2
#define set_EIPH0_PPIH                    SFRS=0;EIPH0|=SET_BIT1
#define set_EIPH0_PI2C0H                  SFRS=0;EIPH0|=SET_BIT0
                        
#define clr_EIPH0_PT2H                    SFRS=0;EIPH0&=CLR_BIT7
#define clr_EIPH0_PSPI0H                  SFRS=0;EIPH0&=CLR_BIT6
#define clr_EIPH0_PFBH                    SFRS=0;EIPH0&=CLR_BIT5
#define clr_EIPH0_PWDTH                   SFRS=0;EIPH0&=CLR_BIT4
#define clr_EIPH0_PPWM0H                  SFRS=0;EIPH0&=CLR_BIT3
#define clr_EIPH0_PCAPH	                  SFRS=0;EIPH0&=CLR_BIT2
#define clr_EIPH0_PPIH                    SFRS=0;EIPH0&=CLR_BIT1
#define clr_EIPH0_PI2C0H                  SFRS=0;EIPH0&=CLR_BIT0
// ;--------------------------------------------- 
// ; I2C1CON
// ;--------------------------------------------- 
#define set_I2C1CON_I2CEN                 SFRS=0;I2C1CON|=SET_BIT6
#define set_I2C1CON_STA                   SFRS=0;I2C1CON|=SET_BIT5
#define set_I2C1CON_STO                   SFRS=0;I2C1CON|=SET_BIT4
#define set_I2C1CON_SI                    SFRS=0;I2C1CON|=SET_BIT3
#define set_I2C1CON_AA                    SFRS=0;I2C1CON|=SET_BIT2
                                            
#define clr_I2C1CON_I2CEN                 SFRS=0;I2C1CON&=CLR_BIT6
#define clr_I2C1CON_STA                   SFRS=0;I2C1CON&=CLR_BIT5
#define clr_I2C1CON_STO                   SFRS=0;I2C1CON&=CLR_BIT4
#define clr_I2C1CON_SI                    SFRS=0;I2C1CON&=CLR_BIT3          
#define clr_I2C1CON_AA                    SFRS=0;I2C1CON&=CLR_BIT2          
// ;--------------------------------------------- 
// ; DMA0TSR                                      
// ;--------------------------------------------- 
#define clr_DMA0TSR_HDONE                 SFRS=0;DMA0TSR&=CLR_BIT2
#define clr_DMA0TSR_FDONE                 SFRS=0;DMA0TSR&=CLR_BIT1
// ;--------------------------------------------- 
// ; MTM0DA                                      
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; DMA1CR                                      
// ;---------------------------------------------
#define set_DMA1CR_HIE                    SFRS=0;DMA1CR|=SET_BIT3
#define set_DMA1CR_FIE                    SFRS=0;DMA1CR|=SET_BIT2
#define set_DMA1CR_RUN                    SFRS=0;DMA1CR|=SET_BIT1
#define set_DMA1CR_EN                     SFRS=0;DMA1CR|=SET_BIT0
                                          
#define clr_DMA1CR_HIE                    SFRS=0;DMA1CR&=CLR_BIT3
#define clr_DMA1CR_FIE                    SFRS=0;DMA1CR&=CLR_BIT2
#define clr_DMA1CR_RUN                    SFRS=0;DMA1CR&=CLR_BIT1
#define clr_DMA1CR_EN                     SFRS=0;DMA1CR&=CLR_BIT0
// ;---------------------------------------------
// ; DMA1MA                                      
// ;---------------------------------------------  
// ;---------------------------------------------  
// ; DMA1CNT                                        
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA1CCNT                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; EIP0
// ;---------------------------------------------
#define set_EPI0_PT2                      SFRS=0;EIP0|=SET_BIT7
#define set_EPI0_PSPI0                    SFRS=0;EIP0|=SET_BIT6
#define set_EPI0_PFB                      SFRS=0;EIP0|=SET_BIT5
#define set_EPI0_PWDT                     SFRS=0;EIP0|=SET_BIT4
#define set_EPI0_PPWM0                    SFRS=0;EIP0|=SET_BIT3
#define set_EPI0_PCAP                     SFRS=0;EIP0|=SET_BIT2
#define set_EPI0_PPI                      SFRS=0;EIP0|=SET_BIT1
#define set_EPI0_PI2C0                    SFRS=0;EIP0|=SET_BIT0
                                          
#define clr_EPI0_PT2                      SFRS=0;EIP0&=CLR_BIT7
#define clr_EPI0_PSPI0                    SFRS=0;EIP0&=CLR_BIT6
#define clr_EPI0_PFB                      SFRS=0;EIP0&=CLR_BIT5
#define clr_EPI0_PWDT                     SFRS=0;EIP0&=CLR_BIT4
#define clr_EPI0_PPWM0                    SFRS=0;EIP0&=CLR_BIT3
#define clr_EPI0_PCAP                     SFRS=0;EIP0&=CLR_BIT2
#define clr_EPI0_PPI                      SFRS=0;EIP0&=CLR_BIT1
#define clr_EPI0_PI2C0                    SFRS=0;EIP0&=CLR_BIT0
// ;---------------------------------------------
// ; ADCCON1
// ;---------------------------------------------                            
#define set_ADCCON1_STADCPX               SFRS=0;ADCCON1|=SET_BIT6
#define set_ADCCON1_ETGTYP1               SFRS=0;ADCCON1|=SET_BIT3
#define set_ADCCON1_ETGTYP0               SFRS=0;ADCCON1|=SET_BIT2
#define set_ADCCON1_ADCEX                 SFRS=0;ADCCON1|=SET_BIT1
#define set_ADCCON1_ADCEN                 SFRS=0;ADCCON1|=SET_BIT0
                                                  
#define clr_ADCCON1_STADCPX               SFRS=0;ADCCON1&=CLR_BIT6
#define clr_ADCCON1_ETGTYP1               SFRS=0;ADCCON1&=CLR_BIT3
#define clr_ADCCON1_ETGTYP0               SFRS=0;ADCCON1&=CLR_BIT2
#define clr_ADCCON1_ADCEX                 SFRS=0;ADCCON1&=CLR_BIT1
#define clr_ADCCON1_ADCEN                 SFRS=0;ADCCON1&=CLR_BIT0
// ;--------------------------------------------- 
//ADCON2
// ;--------------------------------------------- 
#define set_ADCON2_ADFBEN                 SFRS=0;ADCCON2|=SET_BIT7
#define set_ADCON2_ADCMPOP                SFRS=0;ADCCON2|=SET_BIT6
#define set_ADCON2_ADCMPEN                SFRS=0;ADCCON2|=SET_BIT5
#define set_ADCON2_ADCMPO                 SFRS=0;ADCCON2|=SET_BIT4
                                          
#define clr_ADCON2_ADFBEN                 SFRS=0;ADCCON2&=CLR_BIT7
#define clr_ADCON2_ADCMPOP                SFRS=0;ADCCON2&=CLR_BIT6
#define clr_ADCON2_ADCMPEN                SFRS=0;ADCCON2&=CLR_BIT5
#define clr_ADCON2_ADCMPO                 SFRS=0;ADCCON2&=CLR_BIT4
// ;--------------------------------------------- 
// ; ADCDLY
// ;--------------------------------------------- 
// ;--------------------------------------------- 
// ; ADCBAH
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; ADCSN
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; ADCCN
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; ADCSR
// ;---------------------------------------------
#define set_ADCSR_SLOW                    SFRS=0;ADCSR|=SET_BIT7
                                          
#define clr_ADCSR_SLOW                    SFRS=0;ADCSR&=CLR_BIT7
#define clr_ADCSR_CMPHIT                  SFRS=0;ADCSR&=CLR_BIT2
#define clr_ADCSR_HDONE                   SFRS=0;ADCSR&=CLR_BIT1
#define set_ADCSR_FDONE                   SFRS=0;ADCSR&=CLR_BIT0
// ;--------------------------------------------- 
// ; P4
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; SC0DR
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; SC0EGT
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; SC0ETURD0
// ;---------------------------------------------
// ;--------------------------------------------- 
// ; SC0ETURD1
// ;---------------------------------------------
// ;---------------------------------------------
// ;SC0IE                                     
// ;---------------------------------------------
#define set_SC0IE_ACERRIEN                 SFRS=0;SC0IE|=SET_BIT4
#define set_SC0IE_BGTIEN                   SFRS=0;SC0IE|=SET_BIT3
#define set_SC0IE_TERRIEN                  SFRS=0;SC0IE|=SET_BIT2
#define set_SC0IE_TBEIEN                   SFRS=0;SC0IE|=SET_BIT1
#define set_SC0IE_RDAIEN                   SFRS=0;SC0IE|=SET_BIT0
   
#define clr_SC0IE_ACERRIEN                 SFRS=0;SC0IE&=CLR_BIT4
#define clr_SC0IE_BGTIEN                   SFRS=0;SC0IE&=CLR_BIT3
#define clr_SC0IE_TERRIEN                  SFRS=0;SC0IE&=CLR_BIT2
#define clr_SC0IE_TBEIEN                   SFRS=0;SC0IE&=CLR_BIT1
#define clr_SC0IE_RDAIEN                   SFRS=0;SC0IE&=CLR_BIT0
// ;---------------------------------------------
// ;SC0IS                                     
// ;---------------------------------------------

#define clr_SC0IS_SIF                      SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;SC0IS|=SET_BIT2;EA=BIT_TMP
#define clr_SC0IS_ACERRIF                  SFRS=0;SC0IS&=CLR_BIT4
#define clr_SC0IS_BGTIF                    SFRS=0;SC0IS&=CLR_BIT3
#define clr_SC0IS_TERRIF                   SFRS=0;SC0IS&=CLR_BIT2
#define clr_SC0IS_TBEIF                    SFRS=0;SC0IS&=CLR_BIT1
#define clr_SC0IS_RDAIF                    SFRS=0;SC0IS&=CLR_BIT0
// ;---------------------------------------------
// ; SC0TSR
// ;---------------------------------------------
// ;---------------------------------------------
// ; PSW
// ;---------------------------------------------
// ;---------------------------------------------
// ;PWM0CON0
// ;---------------------------------------------
#define set_PWM0CON0_PWMRUN                SFRS=0;PWM0CON0|=SET_BIT7
#define set_PWM0CON0_LOAD                  SFRS=0;PWM0CON0|=SET_BIT6
#define set_PWM0CON0_PWMF                  SFRS=0;PWM0CON0|=SET_BIT5
#define set_PWM0CON0_CLRPWM                SFRS=0;PWM0CON0|=SET_BIT4
                                           
#define clr_PWM0CON0_PWMRUN                SFRS=0;PWM0CON0&=CLR_BIT7
#define clr_PWM0CON0_LOAD                  SFRS=0;PWM0CON0&=CLR_BIT6
#define clr_PWM0CON0_PWMF                  SFRS=0;PWM0CON0&=CLR_BIT5
#define clr_PWM0CON0_CLRPWM                SFRS=0;PWM0CON0&=CLR_BIT4
// ;---------------------------------------------
// ;ACMPCR0
// ;---------------------------------------------
#define set_ACMPCR0_POSSEL1                SFRS=0;ACMPCR0 |= SET_BIT7
#define set_ACMPCR0_POSSEL0                SFRS=0;ACMPCR0 |= SET_BIT6
#define set_ACMPCR0_NEGSEL1                SFRS=0;ACMPCR0 |= SET_BIT5
#define set_ACMPCR0_NEGSEL0                SFRS=0;ACMPCR0 |= SET_BIT4
#define set_ACMPCR0_WKEN                   SFRS=0;ACMPCR0 |= SET_BIT3
#define set_ACMPCR0_HYSEN                  SFRS=0;ACMPCR0 |= SET_BIT2
#define set_ACMPCR0_ACMPIE                 SFRS=0;ACMPCR0 |= SET_BIT1
#define set_ACMPCR0_ACMPEN                 SFRS=0;ACMPCR0 |= SET_BIT0
                                           
#define clr_ACMPCR0_POSSEL1                SFRS=0;ACMPCR0 &= CLR_BIT7
#define clr_ACMPCR0_POSSEL0                SFRS=0;ACMPCR0 &= CLR_BIT6
#define clr_ACMPCR0_NEGSEL1                SFRS=0;ACMPCR0 &= CLR_BIT5
#define clr_ACMPCR0_NEGSEL0                SFRS=0;ACMPCR0 &= CLR_BIT4
#define clr_ACMPCR0_WKEN                   SFRS=0;ACMPCR0 &= CLR_BIT3
#define clr_ACMPCR0_HYSEN                  SFRS=0;ACMPCR0 &= CLR_BIT2
#define clr_ACMPCR0_ACMPIE                 SFRS=0;ACMPCR0 &= CLR_BIT1
#define clr_ACMPCR0_ACMPEN                 SFRS=0;ACMPCR0 &= CLR_BIT0
// ;--------------------------------------------- 
// ;ACMPCR1                                       
// ;--------------------------------------------- 
#define set_ACMPCR1_POSSEL1                SFRS=0;ACMPCR1 |= SET_BIT7     
#define set_ACMPCR1_POSSEL0                SFRS=0;ACMPCR1 |= SET_BIT6     
#define set_ACMPCR1_NEGSEL1                SFRS=0;ACMPCR1 |= SET_BIT5     
#define set_ACMPCR1_NEGSEL0                SFRS=0;ACMPCR1 |= SET_BIT4     
#define set_ACMPCR1_WKEN                   SFRS=0;ACMPCR1 |= SET_BIT3     
#define set_ACMPCR1_HYSEN                  SFRS=0;ACMPCR1 |= SET_BIT2     
#define set_ACMPCR1_ACMPIE                 SFRS=0;ACMPCR1 |= SET_BIT1     
#define set_ACMPCR1_ACMPEN                 SFRS=0;ACMPCR1 |= SET_BIT0     
                                                             
#define clr_ACMPCR1_POSSEL1                SFRS=0;ACMPCR1 &= CLR_BIT7     
#define clr_ACMPCR1_POSSEL0                SFRS=0;ACMPCR1 &= CLR_BIT6     
#define clr_ACMPCR1_NEGSEL1                SFRS=0;ACMPCR1 &= CLR_BIT5     
#define clr_ACMPCR1_NEGSEL0                SFRS=0;ACMPCR1 &= CLR_BIT4     
#define clr_ACMPCR1_WKEN                   SFRS=0;ACMPCR1 &= CLR_BIT3     
#define clr_ACMPCR1_HYSEN                  SFRS=0;ACMPCR1 &= CLR_BIT2     
#define clr_ACMPCR1_ACMPIE                 SFRS=0;ACMPCR1 &= CLR_BIT1     
#define clr_ACMPCR1_ACMPEN                 SFRS=0;ACMPCR1 &= CLR_BIT0     
// ;---------------------------------------------
// ; ACMPSR
// ;---------------------------------------------
#define set_ACMPSR_ACMP1O                  SFRS=0;ACMPSR |= SET_BIT3
#define set_ACMPSR_ACMP1IF                 SFRS=0;ACMPSR |= SET_BIT2
#define set_ACMPSR_ACMP0O                  SFRS=0;ACMPSR |= SET_BIT1
#define set_ACMPSR_ACMP0IF                 SFRS=0;ACMPSR |= SET_BIT0
                                           
#define clr_ACMPSR_ACMP1O                  SFRS=0;ACMPSR &= CLR_BIT3
#define clr_ACMPSR_ACMP1IF                 SFRS=0;ACMPSR &= CLR_BIT2
#define clr_ACMPSR_ACMP0O                  SFRS=0;ACMPSR &= CLR_BIT1
#define clr_ACMPSR_ACMP0IF                 SFRS=0;ACMPSR &= CLR_BIT0       
// ;---------------------------------------------
// ;ACMPVREF
// ;---------------------------------------------

// ;---------------------------------------------
// ;SC0CR0                                      
// ;---------------------------------------------
#define set_SC0CR0_NSB                     SFRS=0;SC0CR0|=SET_BIT7
#define set_SC0CR0_T                       SFRS=0;SC0CR0|=SET_BIT6
#define set_SC0CR0_RXBGTEN                 SFRS=0;SC0CR0|=SET_BIT5
#define set_SC0CR0_CONSEL                  SFRS=0;SC0CR0|=SET_BIT4
#define set_SC0CR0_AUTOCEN                 SFRS=0;SC0CR0|=SET_BIT3
#define set_SC0CR0_TXOFF                   SFRS=0;SC0CR0|=SET_BIT2
#define set_SC0CR0_RXOFF                   SFRS=0;SC0CR0|=SET_BIT1         
#define set_SC0CR0_SCEN                     SFRS=0;SC0CR0|=SET_BIT0         
                                                                    
#define clr_SC0CR0_NSB                     SFRS=0;SC0CR0&=CLR_BIT7                              
#define clr_SC0CR0_T                       SFRS=0;SC0CR0&=CLR_BIT6                                
#define clr_SC0CR0_RXBGTEN                 SFRS=0;SC0CR0&=CLR_BIT5                                
#define clr_SC0CR0_CONSEL                  SFRS=0;SC0CR0&=CLR_BIT4         
#define clr_SC0CR0_AUTOCEN                 SFRS=0;SC0CR0&=CLR_BIT3         
#define clr_SC0CR0_TXOFF                   SFRS=0;SC0CR0&=CLR_BIT2         
#define clr_SC0CR0_RXOFF                   SFRS=0;SC0CR0&=CLR_BIT1         
#define clr_SC0CR0_SCEN                     SFRS=0;SC0CR0&=CLR_BIT1         
// ;---------------------------------------------
// ;SC0CR1                                      
// ;---------------------------------------------
#define set_SC0CR1_OPE                     SFRS=0;SC0CR1|=SET_BIT7
#define set_SC0CR1_PBOFF                   SFRS=0;SC0CR1|=SET_BIT6
#define set_SC0CR1_TXDMAEN                 SFRS=0;SC0CR1|=SET_BIT3
#define set_SC0CR1_RXDMAEN                 SFRS=0;SC0CR1|=SET_BIT2
#define set_SC0CR1_CLKKEEP                 SFRS=0;SC0CR1|=SET_BIT1
#define set_SC0CR1_UARTEN                  SFRS=0;SC0CR1|=SET_BIT0
                                           
#define clr_SC0CR1_OPE                     SFRS=0;SC0CR1&=CLR_BIT7
#define clr_SC0CR1_PBOFF                   SFRS=0;SC0CR1&=CLR_BIT6
#define clr_SC0CR1_TXDMAEN                 SFRS=0;SC0CR1&=CLR_BIT3
#define clr_SC0CR1_RXDMAEN                 SFRS=0;SC0CR1&=CLR_BIT2
#define clr_SC0CR1_CLKKEEP                 SFRS=0;SC0CR1&=CLR_BIT1
#define clr_SC0CR1_UARTEN                  SFRS=0;SC0CR1&=CLR_BIT0
// ;---------------------------------------------
// ; T2CON
// ;---------------------------------------------
#define set_T2CON_TF2                      SFRS=0;T2CON|=SET_BIT7
#define set_T2CON_TR2                      SFRS=0;T2CON|=SET_BIT2
#define set_T2CON_CMRL2                    SFRS=0;T2CON|=SET_BIT0
                                           
#define clr_T2CON_TF2                      SFRS=0;T2CON&=CLR_BIT7
#define clr_T2CON_TR2                      SFRS=0;T2CON&=CLR_BIT2
#define clr_T2CON_CMRL2                    SFRS=0;T2CON&=CLR_BIT0
// ;---------------------------------------------
// ; T2MOD
// ;---------------------------------------------
#define set_T2MOD_LDEN                     SFRS=0;T2MOD|=SET_BIT7
#define set_T2MOD_T2DIV2                   SFRS=0;T2MOD|=SET_BIT6
#define set_T2MOD_T2DIV1                   SFRS=0;T2MOD|=SET_BIT5
#define set_T2MOD_T2DIV0                   SFRS=0;T2MOD|=SET_BIT4
#define set_T2MOD_CAPCR                    SFRS=0;T2MOD|=SET_BIT3
#define set_T2MOD_CMPCR                    SFRS=0;T2MOD|=SET_BIT2
#define set_T2MOD_LDTS1                    SFRS=0;T2MOD|=SET_BIT1
#define set_T2MOD_LDTS0                    SFRS=0;T2MOD|=SET_BIT0
                                           
#define clr_T2MOD_LDEN                     SFRS=0;T2MOD&=CLR_BIT7
#define clr_T2MOD_T2DIV2                   SFRS=0;T2MOD&=CLR_BIT6
#define clr_T2MOD_T2DIV1                   SFRS=0;T2MOD&=CLR_BIT5
#define clr_T2MOD_T2DIV0                   SFRS=0;T2MOD&=CLR_BIT4
#define clr_T2MOD_CAPCR                    SFRS=0;T2MOD&=CLR_BIT3
#define clr_T2MOD_CMPCR                    SFRS=0;T2MOD&=CLR_BIT2
#define clr_T2MOD_LDTS1                    SFRS=0;T2MOD&=CLR_BIT1
#define clr_T2MOD_LDTS0                    SFRS=0;T2MOD&=CLR_BIT0
// ;---------------------------------------------
// ; PIF
// ;---------------------------------------------
#define clr_PIF7                           SFRS=0;PIF&=CLR_BIT7
#define clr_PIF6                           SFRS=0;PIF&=CLR_BIT6
#define clr_PIF5                           SFRS=0;PIF&=CLR_BIT5
#define clr_PIF4                           SFRS=0;PIF&=CLR_BIT4
#define clr_PIF3                           SFRS=0;PIF&=CLR_BIT3
#define clr_PIF2                           SFRS=0;PIF&=CLR_BIT2
#define clr_PIF1                           SFRS=0;PIF&=CLR_BIT1
#define clr_PIF0                           SFRS=0;PIF&=CLR_BIT0
// ;---------------------------------------------
// ; ADCBAL
// ;---------------------------------------------
// ;---------------------------------------------
// ; TL2
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; TH2
// ;---------------------------------------------  
// ;---------------------------------------------
// ; ADCMPL
// ;---------------------------------------------       
// ;---------------------------------------------
// ; ADCMPH
// ;---------------------------------------------       
// ;--------------------------------------------- 
// ; I2C0CON
// ;--------------------------------------------- 
#define set_I2C0CON_I2CEN                  SFRS=0;I2C0CON|=SET_BIT6
#define set_I2C0CON_STA                    SFRS=0;I2C0CON|=SET_BIT5
#define set_I2C0CON_STO                    SFRS=0;I2C0CON|=SET_BIT4
#define set_I2C0CON_SI                     SFRS=0;I2C0CON|=SET_BIT3
#define set_I2C0CON_AA                     SFRS=0;I2C0CON|=SET_BIT2
                                           
#define clr_I2C0CON_I2CEN                  SFRS=0;I2C0CON&=CLR_BIT6
#define clr_I2C0CON_STA                    SFRS=0;I2C0CON&=CLR_BIT5
#define clr_I2C0CON_STO                    SFRS=0;I2C0CON&=CLR_BIT4
#define clr_I2C0CON_SI                     SFRS=0;I2C0CON&=CLR_BIT3          
#define clr_I2C0CON_AA                     SFRS=0;I2C0CON&=CLR_BIT2                         
// ;--------------------------------------------- 
// ; I2C0ADDR
// ;---------------------------------------------       
// ;--------------------------------------------- 
// ; ADCRL
// ;---------------------------------------------        
// ;--------------------------------------------- 
// ; ADCRH
// ;--------------------------------------------- 
// ;---------------------------------------------        
// ; T3CON
// ;--------------------------------------------- 
#define set_T3CON_SMOD_1                   SFRS=0;T3CON|=SET_BIT7
#define set_T3CON_SMOD0_1                  SFRS=0;T3CON|=SET_BIT6
#define set_T3CON_BRCK                     SFRS=0;T3CON|=SET_BIT5
#define set_T3CON_TF3                      SFRS=0;T3CON|=SET_BIT4
#define set_T3CON_TR3                      SFRS=0;T3CON|=SET_BIT3
#define set_T3CON_T3PS2                    SFRS=0;T3CON|=SET_BIT2
#define set_T3CON_T3PS1                    SFRS=0;T3CON|=SET_BIT1
#define set_T3CON_T3PS0                    SFRS=0;T3CON|=SET_BIT0
                                           
#define clr_T3CON_SMOD_1                   SFRS=0;T3CON&=CLR_BIT7
#define clr_T3CON_SMOD0_1                  SFRS=0;T3CON&=CLR_BIT6
#define clr_T3CON_BRCK                     SFRS=0;T3CON&=CLR_BIT5
#define clr_T3CON_TF3                      SFRS=0;T3CON&=CLR_BIT4
#define clr_T3CON_TR3                      SFRS=0;T3CON&=CLR_BIT3
#define clr_T3CON_T3PS2                    SFRS=0;T3CON&=CLR_BIT2
#define clr_T3CON_T3PS1                    SFRS=0;T3CON&=CLR_BIT1
#define clr_T3CON_T3PS0                    SFRS=0;T3CON&=CLR_BIT0
// ;---------------------------------------------
// ; RL3
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; RH3
// ;---------------------------------------------        
// ;---------------------------------------------
// ; TA
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; IP
// ;---------------------------------------------        
// ;---------------------------------------------
// ; SADEN
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; S1ADEN
// ;---------------------------------------------        
// ;---------------------------------------------
// ; S1ADDR
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; I2C0DAT
// ;---------------------------------------------        
// ;---------------------------------------------
// ; I2C0STAT                                    
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; I2C0CLK
// ;---------------------------------------------        
// ;---------------------------------------------
// ; I2C0TOC
// ;---------------------------------------------            
#define set_I2C0TOC_I2TOCEN               SFRS=0;I2C0TOC|=SET_BIT2
#define set_I2C0TOC_DIV                   SFRS=0;I2C0TOC|=SET_BIT1
#define set_I2C0TOC_I2TOF                 SFRS=0;I2C0TOC|=SET_BIT0       

#define clr_I2C0TOC_I2TOCEN               SFRS=0;I2C0TOC&=CLR_BIT2
#define clr_I2C0TOC_DIV                   SFRS=0;I2C0TOC&=CLR_BIT1
#define clr_I2C0TOC_I2TOF                 SFRS=0;I2C0TOC&=CLR_BIT0      
// ;---------------------------------------------        
// ; P3                                                 
// ;---------------------------------------------     
// ;--------------------------------------------- 
// ; P5                                          
// ;--------------------------------------------- 
#define set_P57	                          SFRS=0;P5|=SET_BIT7
#define set_P56	                          SFRS=0;P5|=SET_BIT6
#define set_P55	                          SFRS=0;P5|=SET_BIT5
#define set_P54	                          SFRS=0;P5|=SET_BIT4
#define set_P53	                          SFRS=0;P5|=SET_BIT3
#define set_P52	                          SFRS=0;P5|=SET_BIT2   
#define set_P51	                          SFRS=0;P5|=SET_BIT1       
#define set_P59	                          SFRS=0;P5|=SET_BIT0       
                                                       
#define clr_P57	                          SFRS=0;P5&=CLR_BIT7       
#define clr_P56	                          SFRS=0;P5&=CLR_BIT6       
#define clr_P55	                          SFRS=0;P5&=CLR_BIT5       
#define clr_P54	                          SFRS=0;P5&=CLR_BIT4       
#define clr_P53	                          SFRS=0;P5&=CLR_BIT3       
#define clr_P52	                          SFRS=0;P5&=CLR_BIT2       
#define clr_P51	                          SFRS=0;P5&=CLR_BIT1       
#define clr_P59	                          SFRS=0;P5&=CLR_BIT0      
// ;--------------------------------------------- 
// ; I2C1ADDR                                      
// ;--------------------------------------------- 
// ;--------------------------------------------- 
// ; I2C1DAT                                      
// ;--------------------------------------------- 
// ;--------------------------------------------- 
// ; I2C1STAT                                      
// ;--------------------------------------------- 
// ;--------------------------------------------- 
// ; I2C1CLK                                      
// ;--------------------------------------------- 
// ;---------------------------------------------
// ; I2C1TOC
// ;---------------------------------------------            
#define set_I2C1TOC_I2TOCEN               SFRS=0;I2C1TOC|=SET_BIT2
#define set_I2C1TOC_DIV                   SFRS=0;I2C1TOC|=SET_BIT1
#define set_I2C1TOC_I2TOF                 SFRS=0;I2C1TOC|=SET_BIT0       
                                  
#define clr_I2C1TOC_I2TOCEN               SFRS=0;I2C1TOC&=CLR_BIT2
#define clr_I2C1TOC_DIV                   SFRS=0;I2C1TOC&=CLR_BIT1
#define clr_I2C1TOC_I2TOF                 SFRS=0;I2C1TOC&=CLR_BIT0
// ;---------------------------------------------          
// ; IPH
// ;---------------------------------------------  
#define set_IPH_PADCH                     SFRS=0;IPH|=SET_BIT6
#define set_IPH_PBODH                     SFRS=0;IPH|=SET_BIT5
#define set_IPH_PSH                       SFRS=0;IPH|=SET_BIT4
#define set_IPH_PT1H                      SFRS=0;IPH|=SET_BIT3
#define set_IPH_PX11                      SFRS=0;IPH|=SET_BIT2
#define set_IPH_PT0H                      SFRS=0;IPH|=SET_BIT1
#define set_IPH_PX0H                      SFRS=0;IPH|=SET_BIT0
                                           
#define clr_IPH_PADCH                     SFRS=0;IPH&=CLR_BIT6
#define clr_IPH_PBODH                     SFRS=0;IPH&=CLR_BIT5
#define clr_IPH_PSH                       SFRS=0;IPH&=CLR_BIT4
#define clr_IPH_PT1H                      SFRS=0;IPH&=CLR_BIT3
#define clr_IPH_PX11                      SFRS=0;IPH&=CLR_BIT2
#define clr_IPH_PT0H                      SFRS=0;IPH&=CLR_BIT1
#define clr_IPH_PX0H                      SFRS=0;IPH&=CLR_BIT0   
// ;---------------------------------------------     
// ; IE                                          
// ;--------------------------------------------- 
#define set_IE_EA                         SFRS=0;IE|=SET_BIT7
#define set_IE_EADC                       SFRS=0;IE|=SET_BIT6
#define set_IE_EBOD                       SFRS=0;IE|=SET_BIT5
#define set_IE_ES                         SFRS=0;IE|=SET_BIT4
#define set_IE_ET1                        SFRS=0;IE|=SET_BIT3
#define set_IE_EX1                        SFRS=0;IE|=SET_BIT2
#define set_IE_ET0                        SFRS=0;IE|=SET_BIT1
#define set_IE_EX0                        SFRS=0;IE|=SET_BIT0
                                  
#define clr_IE_EA                         SFRS=0;IE&=CLR_BIT7
#define clr_IE_EADC                       SFRS=0;IE&=CLR_BIT6
#define clr_IE_EBOD                       SFRS=0;IE&=CLR_BIT5
#define clr_IE_ES                         SFRS=0;IE&=CLR_BIT4
#define clr_IE_ET1                        SFRS=0;IE&=CLR_BIT3
#define clr_IE_EX1                        SFRS=0;IE&=CLR_BIT2
#define clr_IE_ET0                        SFRS=0;IE&=CLR_BIT1
#define clr_IE_EX0                        SFRS=0;IE&=CLR_BIT0
// ;---------------------------------------------  
// ; SADDR                                            
// ;---------------------------------------------
// ;---------------------------------------------  
// ; WDCON
// ;---------------------------------------------
#define set_WDCON_WDTR                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT7;EA=BIT_TMP
#define set_WDCON_WDCLR                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT6;EA=BIT_TMP
#define set_WDCON_WDTF                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT5;EA=BIT_TMP
#define set_WDCON_WIDPD                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT4;EA=BIT_TMP
#define set_WDCON_WDTRF                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT3;EA=BIT_TMP
#define set_WDCON_WDPS2                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT2;EA=BIT_TMP
#define set_WDCON_WDPS1                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT1;EA=BIT_TMP
#define set_WDCON_WDPS0                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON|=SET_BIT0;EA=BIT_TMP
                                          
#define clr_WDCON_WDTR                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT7;EA=BIT_TMP
#define clr_WDCON_WDCLR                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT6;EA=BIT_TMP
#define clr_WDCON_WDTF                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT5;EA=BIT_TMP
#define clr_WDCON_WIDPD                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT4;EA=BIT_TMP
#define clr_WDCON_WDTRF                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT3;EA=BIT_TMP
#define clr_WDCON_WDPS2                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT2;EA=BIT_TMP
#define clr_WDCON_WDPS1                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT1;EA=BIT_TMP
#define clr_WDCON_WDPS0                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;WDCON&=CLR_BIT0;EA=BIT_TMP
// ;---------------------------------------------
// ; BODCON1
// ;---------------------------------------------
#define set_BODCON1_LPBOD1                SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1|=SET_BIT2;EA=BIT_TMP
#define set_BODCON1_LPBOD0                SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1|=SET_BIT1;EA=BIT_TMP
#define set_BODCON1_BODFLT                SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1|=SET_BIT0;EA=BIT_TMP
                                          
#define clr_BODCON1_LPBOD1                SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1&=CLR_BIT2;EA=BIT_TMP
#define clr_BODCON1_LPBOD0                SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1&=CLR_BIT1;EA=BIT_TMP
#define clr_BODCON1_BODFLT                SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON1&=CLR_BIT0;EA=BIT_TMP
// ;----------------------------------
// ; EIP2                              
// ;----------------------------------
#define set_EIP2_PDMA3                    SFRS=0;EIP2|=SET_BIT6 
#define set_EIP2_PDMA2                    SFRS=0;EIP2|=SET_BIT5 
#define set_EIP2_SMC1                     SFRS=0;EIP2|=SET_BIT4 
#define set_EIP2_PFB1                     SFRS=0;EIP2|=SET_BIT3 
#define set_EIP2_PPWM1                    SFRS=0;EIP2|=SET_BIT2 
#define set_EIP2_PI2C1                    SFRS=0;EIP2|=SET_BIT1 
#define set_EIP2_PACMP                    SFRS=0;EIP2|=SET_BIT0 
                                       
#define clr_EIP2_PDMA3                    SFRS=0;EIP2&=CLR_BIT6 
#define clr_EIP2_PDMA2                    SFRS=0;EIP2&=CLR_BIT5 
#define clr_EIP2_SMC1                     SFRS=0;EIP2&=CLR_BIT4 
#define clr_EIP2_PFB1                     SFRS=0;EIP2&=CLR_BIT3 
#define clr_EIP2_PPWM1                    SFRS=0;EIP2&=CLR_BIT2 
#define clr_EIP2_PI2C1                    SFRS=0;EIP2&=CLR_BIT1 
#define clr_EIP2_PACMP                    SFRS=0;EIP2&=CLR_BIT0 
// ;----------------------------------
// ; EIPH2                              
// ;----------------------------------
#define set_EIPH2_PDMA3H                  SFRS=0;EIPH2|=SET_BIT6 
#define set_EIPH2_PDMA2H                  SFRS=0;EIPH2|=SET_BIT5 
#define set_EIPH2_SMC1H	                  SFRS=0;EIPH2|=SET_BIT4 
#define set_EIPH2_PFB1H	                  SFRS=0;EIPH2|=SET_BIT3 
#define set_EIPH2_PPWM1H                  SFRS=0;EIPH2|=SET_BIT2 
#define set_EIPH2_PI2C1H                  SFRS=0;EIPH2|=SET_BIT1 
#define set_EIPH2_PACMPH                  SFRS=0;EIPH2|=SET_BIT0 
                                                       
#define clr_EIPH2_PDMA3H                  SFRS=0;EIPH2&=CLR_BIT6 
#define clr_EIPH2_PDMA2H                  SFRS=0;EIPH2&=CLR_BIT5 
#define clr_EIPH2_SMC1H                   SFRS=0;EIPH2&=CLR_BIT4 
#define clr_EIPH2_PFB1H                   SFRS=0;EIPH2&=CLR_BIT3 
#define clr_EIPH2_PPWM1H                  SFRS=0;EIPH2&=CLR_BIT2 
#define clr_EIPH2_PI2C1H                  SFRS=0;EIPH2&=CLR_BIT1 
#define clr_EIPH2_PACMPH                  SFRS=0;EIPH2&=CLR_BIT0
// ;----------------------------------
// ; IAPFD                              
// ;----------------------------------
// ;----------------------------------
// ; IAPCN
// ;----------------------------------
#define set_IAPCN_FOEN                    SFRS=0;IAPCN|=SET_BIT5
#define set_IAPCN_FCEN                    SFRS=0;IAPCN|=SET_BIT4
#define set_IAPCN_FCTRL3                  SFRS=0;IAPCN|=SET_BIT3
#define set_IAPCN_FCTRL2                  SFRS=0;IAPCN|=SET_BIT2
#define set_IAPCN_FCTRL1                  SFRS=0;IAPCN|=SET_BIT1
#define set_IAPCN_FCTRL0                  SFRS=0;IAPCN|=SET_BIT0
                                          
#define clr_IAPCN_FOEN                    SFRS=0;IAPCN&=CLR_BIT5
#define clr_IAPCN_FCEN                    SFRS=0;IAPCN&=CLR_BIT4
#define clr_IAPCN_FCTRL3                  SFRS=0;IAPCN&=CLR_BIT3
#define clr_IAPCN_FCTRL2                  SFRS=0;IAPCN&=CLR_BIT2
#define clr_IAPCN_FCTRL1                  SFRS=0;IAPCN&=CLR_BIT1
#define clr_IAPCN_FCTRL0                  SFRS=0;IAPCN&=CLR_BIT0
// ;----------------------------------
// ; P2                             
// ;---------------------------------- 
// ;---------------------------------------
// ;ADCCON0                                
// ;---------------------------------------
#define set_ADCCON0_ADCF                  SFRS=0;ADCCON0|=SET_BIT7  
#define set_ADCCON0_ADCS                  SFRS=0;ADCCON0|=SET_BIT6  
#define set_ADCCON0_ETGSEL1               SFRS=0;ADCCON0|=SET_BIT5  
#define set_ADCCON0_ETGSEL0               SFRS=0;ADCCON0|=SET_BIT4  
#define set_ADCCON0_ADCHS3                SFRS=0;ADCCON0|=SET_BIT3  
#define set_ADCCON0_ADCHS2                SFRS=0;ADCCON0|=SET_BIT2  
#define set_ADCCON0_ADCHS1                SFRS=0;ADCCON0|=SET_BIT1  
#define set_ADCCON0_ADCHS0                SFRS=0;ADCCON0|=SET_BIT0                                   
                                                            
#define clr_ADCCON0_ADCF                  SFRS=0;ADCCON0&=CLR_BIT7  
#define clr_ADCCON0_ADCS                  SFRS=0;ADCCON0&=CLR_BIT6  
#define clr_ADCCON0_ETGSEL1               SFRS=0;ADCCON0&=CLR_BIT5  
#define clr_ADCCON0_ETGSEL0               SFRS=0;ADCCON0&=CLR_BIT4  
#define clr_ADCCON0_ADCHS3                SFRS=0;ADCCON0&=CLR_BIT3  
#define clr_ADCCON0_ADCHS2                SFRS=0;ADCCON0&=CLR_BIT2  
#define clr_ADCCON0_ADCHS1                SFRS=0;ADCCON0&=CLR_BIT1  
#define clr_ADCCON0_ADCHS0                SFRS=0;ADCCON0&=CLR_BIT0  
// ;---------------------------------------
// ; AUXR0                                
// ;---------------------------------------
#define set_AUXR0_DPS                     SFRS=0;AUXR0|=SET_BIT0 
                                          
#define clr_AUXR0_SWRF                    SFRS=0;AUXR0&=CLR_BIT7  
#define clr_AUXR0_RSTPINF                 SFRS=0;AUXR0&=CLR_BIT6  
#define clr_AUXR0_HFRF	                  SFRS=0;AUXR0&=CLR_BIT5  
#define clr_AUXR0_HFIF                    SFRS=0;AUXR0&=CLR_BIT4  
#define clr_AUXR0_GF2                     SFRS=0;AUXR0&=CLR_BIT3  
#define clr_AUXR0_DPS                     SFRS=0;AUXR0&=CLR_BIT0 
// ;---------------------------------------
// ; BODCON0
// ;---------------------------------------
#define set_BODCON0_BODEN                 SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT7;EA=BIT_TMP
#define set_BODCON0_BOV1                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT5;EA=BIT_TMP
#define set_BODCON0_BOV0                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT4;EA=BIT_TMP
#define set_BODCON0_BOF                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT3;EA=BIT_TMP
#define set_BODCON0_BORST                 SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT2;EA=BIT_TMP
#define set_BODCON0_BORF                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT1;EA=BIT_TMP
#define set_BODCON0_BOS                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0|=SET_BIT0;EA=BIT_TMP
                                          
#define clr_BODCON0_BODEN                 SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT7;EA=BIT_TMP
#define clr_BODCON0_BOV1                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT5;EA=BIT_TMP
#define clr_BODCON0_BOV0                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT4;EA=BIT_TMP
#define clr_BODCON0_BOF                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT3;EA=BIT_TMP
#define clr_BODCON0_BORST                 SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT2;EA=BIT_TMP
#define clr_BODCON0_BORF                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT1;EA=BIT_TMP
#define clr_BODCON0_BOS                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;BODCON0&=CLR_BIT0;EA=BIT_TMP
// ;---------------------------------------
// ; IAPTRG
// ;---------------------------------------
#define set_IAPTRG_IAPGO                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPTRG|=SET_BIT0;EA=BIT_TMP
// ;---------------------------------------
// ; IAPUEN
// ;---------------------------------------
#define set_IAPUEN_CFUEN                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN|=SET_BIT2;EA=BIT_TMP
#define set_IAPUEN_LDUEN                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN|=SET_BIT1;EA=BIT_TMP
#define set_IAPUEN_APUEN                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN|=SET_BIT0;EA=BIT_TMP
                                          
#define clr_IAPUEN_CFUEN                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN&=CLR_BIT2;EA=BIT_TMP
#define clr_IAPUEN_LDUEN                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN&=CLR_BIT1;EA=BIT_TMP
#define clr_IAPUEN_APUEN                  SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;IAPUEN&=CLR_BIT0;EA=BIT_TMP
// ;---------------------------------------
// ; IAPAL
// ;---------------------------------------
// ;---------------------------------------
// ; IAPAH
// ;---------------------------------------
// ;---------------------------------------
// ; SCON
// ;---------------------------------------
#define set_SCON_FE                        SFRS=0;SCON|=SET_BIT7
#define set_SCON_SM1                       SFRS=0;SCON|=SET_BIT6
#define set_SCON_SM2                       SFRS=0;SCON|=SET_BIT5
#define set_SCON_REN                       SFRS=0;SCON|=SET_BIT4
#define set_SCON_TB8                       SFRS=0;SCON|=SET_BIT3
#define set_SCON_RB8                       SFRS=0;SCON|=SET_BIT2
#define set_SCON_TI                        SFRS=0;SCON|=SET_BIT1
#define set_SCON_RI                        SFRS=0;SCON|=SET_BIT0
                                           
#define clr_SCON_FE                        SFRS=0;SCON&=CLR_BIT7
#define clr_SCON_SM1                       SFRS=0;SCON&=CLR_BIT6
#define clr_SCON_SM2                       SFRS=0;SCON&=CLR_BIT5
#define clr_SCON_REN                       SFRS=0;SCON&=CLR_BIT4
#define clr_SCON_TB8                       SFRS=0;SCON&=CLR_BIT3
#define clr_SCON_RB8                       SFRS=0;SCON&=CLR_BIT2
#define clr_SCON_TI                        SFRS=0;SCON&=CLR_BIT1
#define clr_SCON_RI                        SFRS=0;SCON&=CLR_BIT0
// ;---------------------------------------
// ; SBUF1
// ;---------------------------------------
// ;---------------------------------------
// ; EIE0
// ;---------------------------------------
#define set_EIE0_ET2                       SFRS=0;EIE0|=SET_BIT7
#define set_EIE0_ESPI0                     SFRS=0;EIE0|=SET_BIT6
#define set_EIE0_EFB0                      SFRS=0;EIE0|=SET_BIT5
#define set_EIE0_EWDT                      SFRS=0;EIE0|=SET_BIT4
#define set_EIE0_EPWM0                     SFRS=0;EIE0|=SET_BIT3
#define set_EIE0_ECAP                      SFRS=0;EIE0|=SET_BIT2
#define set_EIE0_EPI                       SFRS=0;EIE0|=SET_BIT1
#define set_EIE0_EI2C0                     SFRS=0;EIE0|=SET_BIT0
                                           
#define clr_EIE0_ET2                       SFRS=0;EIE0&=CLR_BIT7
#define clr_EIE0_ESPI0                     SFRS=0;EIE0&=CLR_BIT6
#define clr_EIE0_EFB0                      SFRS=0;EIE0&=CLR_BIT5
#define clr_EIE0_EWDT                      SFRS=0;EIE0&=CLR_BIT4
#define clr_EIE0_EPWM0                     SFRS=0;EIE0&=CLR_BIT3
#define clr_EIE0_ECAP                      SFRS=0;EIE0&=CLR_BIT2
#define clr_EIE0_EPI                       SFRS=0;EIE0&=CLR_BIT1
#define clr_EIE0_EI2C0                     SFRS=0;EIE0&=CLR_BIT0
// ;---------------------------------------
//EIE1        
// ;---------------------------------------  
#define set_EIE1_EFB1                      SFRS=0;EIE1|=SET_BIT7
#define set_EIE1_EPWM1                     SFRS=0;EIE1|=SET_BIT6
#define set_EIE1_EI2C1                     SFRS=0;EIE1|=SET_BIT5
#define set_EIE1_ESPI1                     SFRS=0;EIE1|=SET_BIT4
#define set_EIE1_EHFI                      SFRS=0;EIE1|=SET_BIT3 
#define set_EIE1_EWKT                      SFRS=0;EIE1|=SET_BIT2 
#define set_EIE1_ET3                       SFRS=0;EIE1|=SET_BIT1 
#define set_EIE1_ES1                       SFRS=0;EIE1|=SET_BIT0 
                                           
#define clr_EIE1_EFB1                      SFRS=0;EIE1&=CLR_BIT7
#define clr_EIE1_EPWM1                     SFRS=0;EIE1&=CLR_BIT6                                       
#define clr_EIE1_EI2C1                     SFRS=0;EIE1&=CLR_BIT5
#define clr_EIE1_ESPI1                     SFRS=0;EIE1&=CLR_BIT4
#define clr_EIE1_HF                        SFRS=0;EIE1&=CLR_BIT3
#define clr_EIE1_EWKT                      SFRS=0;EIE1&=CLR_BIT2
#define clr_EIE1_ET3                       SFRS=0;EIE1&=CLR_BIT1
#define clr_EIE1_ES1                       SFRS=0;EIE1&=CLR_BIT0
// ;---------------------------------------
// ; RSR        
// ;---------------------------------------   
// ;---------------------------------------
// ; IAPTC        
// ;---------------------------------------   
// ;---------------------------------------   
// ; CHPCON
// ;---------------------------------------   
#define set_CHPCON_SWRST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON|=SET_BIT7 ;EA=BIT_TMP
#define set_CHPCON_IAPFF                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON|=SET_BIT6 ;EA=BIT_TMP
#define set_CHPCON_BS                      SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON|=SET_BIT1 ;EA=BIT_TMP
#define set_CHPCON_IAPEN                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON|=SET_BIT0 ;EA=BIT_TMP
                                                                             
#define clr_CHPCON_SWRST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON&=CLR_BIT7;EA=BIT_TMP
#define clr_CHPCON_IAPFF                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON&=CLR_BIT6;EA=BIT_TMP
#define clr_CHPCON_BS                      SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON&=CLR_BIT1;EA=BIT_TMP
#define clr_CHPCON_IAPEN                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CHPCON&=CLR_BIT0;EA=BIT_TMP
// ;---------------------------------------   
// ; P1
// ;--------------------------------------- 
// ;---------------------------------------   
// ; SFRS
// ;---------------------------------------
// ;---------------------------------------------
// ;DMA0CR
// ;---------------------------------------------
#define set_DMA0CR_HIE                     SFRS=0;DMA0CR|=SET_BIT3
#define set_DMA0CR_FIE                     SFRS=0;DMA0CR|=SET_BIT2
#define set_DMA0CR_RUN                     SFRS=0;DMA0CR|=SET_BIT1
#define set_DMA0CR_EN                      SFRS=0;DMA0CR|=SET_BIT0
                                           
#define clr_DMA0CR_HIE                     SFRS=0;DMA0CR&=CLR_BIT3
#define clr_DMA0CR_FIE                     SFRS=0;DMA0CR&=CLR_BIT2
#define clr_DMA0CR_RUN                     SFRS=0;DMA0CR&=CLR_BIT1
#define clr_DMA0CR_EN                      SFRS=0;DMA0CR&=CLR_BIT0
// ;---------------------------------------------
// ;DMA0MA
// ;--------------------------------------------- 
// ;---------------------------------------------
// ;DMA0CNT
// ;--------------------------------------------- 
// ;---------------------------------------------
// ;DMA0CCNT
// ;---------------------------------------------  
// ;---------------------------------------------
// ; CKSWT
// ;---------------------------------------------
#define set_CKSWT_HXTST	                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT7;EA=BIT_TMP
#define set_CKSWT_LXTST	                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT6;EA=BIT_TMP
#define set_CKSWT_HIRCST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT5;EA=BIT_TMP
#define set_CKSWT_LIRCST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT4;EA=BIT_TMP
#define set_CKSWT_ECLKST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT3;EA=BIT_TMP
#define set_CKSWT_OSC2                     SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT2;EA=BIT_TMP
#define set_CKSWT_OSC1                     SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT1;EA=BIT_TMP
#define set_CKSWT_OSC0                     SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT|=SET_BIT0;EA=BIT_TMP
                                           
#define clr_CKSWT_HXTST                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT7;EA=BIT_TMP
#define clr_CKSWT_LXTST                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT6;EA=BIT_TMP
#define clr_CKSWT_HIRCST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT5;EA=BIT_TMP
#define clr_CKSWT_LIRCST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT4;EA=BIT_TMP
#define clr_CKSWT_ECLKST                   SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT3;EA=BIT_TMP
#define clr_CKSWT_OSC2                     SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT2;EA=BIT_TMP
#define clr_CKSWT_OSC1                     SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT1;EA=BIT_TMP
#define clr_CKSWT_OSC0                     SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKSWT&=CLR_BIT0;EA=BIT_TMP
// ;---------------------------------------------
// ;CKEN
// ;---------------------------------------------   
#define set_CKEN_EHXTEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN|=SET_BIT7;EA=BIT_TMP
#define set_CKEN_ELXTEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN|=SET_BIT6;EA=BIT_TMP
#define set_CKEN_HIRCEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN|=SET_BIT5;EA=BIT_TMP
#define set_CKEN_LIRCEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN|=SET_BIT4;EA=BIT_TMP
#define set_CKEN_ECLKEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN|=SET_BIT3;EA=BIT_TMP
#define set_CKEN_CKSWTF                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN|=SET_BIT0;EA=BIT_TMP
                                           
#define clr_CKEN_EHXTEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN&=CLR_BIT7;EA=BIT_TMP
#define clr_CKEN_ELXTEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN&=CLR_BIT6;EA=BIT_TMP
#define clr_CKEN_HIRCEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN&=CLR_BIT5;EA=BIT_TMP
#define clr_CKEN_LIRCEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN&=CLR_BIT4;EA=BIT_TMP
#define clr_CKEN_ECLKEN                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN&=CLR_BIT3;EA=BIT_TMP
#define clr_CKEN_CKSWTF                    SFRS=0;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;CKEN&=CLR_BIT0;EA=BIT_TMP
// ;--------------------------------------------
// ;TCON
// ;--------------------------------------------- 
#define set_TCON_TF1                       SFRS=0;TCON|=SET_BIT7
#define set_TCON_TR1                       SFRS=0;TCON|=SET_BIT6
#define set_TCON_TF0                       SFRS=0;TCON|=SET_BIT5
#define set_TCON_TR0                       SFRS=0;TCON|=SET_BIT4
#define set_TCON_IE1                       SFRS=0;TCON|=SET_BIT3
#define set_TCON_IT1                       SFRS=0;TCON|=SET_BIT2
#define set_TCON_IE0                       SFRS=0;TCON|=SET_BIT1
#define set_TCON_IT0                       SFRS=0;TCON|=SET_BIT0
                                                                
#define clr_TCON_TF1                       SFRS=0;TCON&=CLR_BIT7
#define clr_TCON_TR1                       SFRS=0;TCON&=CLR_BIT6
#define clr_TCON_TF0                       SFRS=0;TCON&=CLR_BIT5
#define clr_TCON_TR0                       SFRS=0;TCON&=CLR_BIT4
#define clr_TCON_IE1                       SFRS=0;TCON&=CLR_BIT3
#define clr_TCON_IT1                       SFRS=0;TCON&=CLR_BIT2
#define clr_TCON_IE0                       SFRS=0;TCON&=CLR_BIT1
#define clr_TCON_IT0                       SFRS=0;TCON&=CLR_BIT0
// ;---------------------------------------------
// ; TMOD
// ;---------------------------------------------
#define set_TMOD_T1_GATE                   SFRS=0;TMOD|=SET_BIT7
#define set_TMOD_T1_CT                     SFRS=0;TMOD|=SET_BIT6
#define set_TMOD_T1_M1                     SFRS=0;TMOD|=SET_BIT5
#define set_TMOD_T1_M0                     SFRS=0;TMOD|=SET_BIT4
#define set_TMOD_T0_GATE                   SFRS=0;TMOD|=SET_BIT3
#define set_TMOD_T0_CT                     SFRS=0;TMOD|=SET_BIT2
#define set_TMOD_T0_M1                     SFRS=0;TMOD|=SET_BIT1
#define set_TMOD_T0_M0                     SFRS=0;TMOD|=SET_BIT0
                                                  
#define clr_TMOD_T1_GATE                   SFRS=0;TMOD&=CLR_BIT7
#define clr_TMOD_T1_CT                     SFRS=0;TMOD&=CLR_BIT6
#define clr_TMOD_T1_M1                     SFRS=0;TMOD&=CLR_BIT5
#define clr_TMOD_T1_M0                     SFRS=0;TMOD&=CLR_BIT4
#define clr_TMOD_T0_GATE                   SFRS=0;TMOD&=CLR_BIT3
#define clr_TMOD_T0_CT                     SFRS=0;TMOD&=CLR_BIT2
#define clr_TMOD_T0_M1                     SFRS=0;TMOD&=CLR_BIT1
#define clr_TMOD_T0_M0                     SFRS=0;TMOD&=CLR_BIT0
// ;---------------------------------------------
// ; TL0
// ;---------------------------------------------
// ;---------------------------------------------
// ; TL1
// ;---------------------------------------------
// ;---------------------------------------------
// ; TH0
// ;---------------------------------------------
// ;---------------------------------------------
// ; TH1
// ;---------------------------------------------
// ;---------------------------------------------
// ; CKCON
// ;---------------------------------------------
#define set_CKCON_FASTWK                  SFRS=0;CKCON|=SET_BIT7
#define set_CKCON_PWMCKS                  SFRS=0;CKCON|=SET_BIT6
#define set_CKCON_T1OE                    SFRS=0;CKCON|=SET_BIT5
#define set_CKCON_T1M                     SFRS=0;CKCON|=SET_BIT4
#define set_CKCON_T0M                     SFRS=0;CKCON|=SET_BIT3
#define set_CKCON_T0OE                    SFRS=0;CKCON|=SET_BIT2
#define set_CKCON_CLOEN                   SFRS=0;CKCON|=SET_BIT1
                                          
#define clr_CKCON_FASTWK                  SFRS=0;CKCON&=CLR_BIT7
#define clr_CKCON_PWMCKS                  SFRS=0;CKCON&=CLR_BIT6
#define clr_CKCON_T1OE                    SFRS=0;CKCON&=CLR_BIT5
#define clr_CKCON_T1M                     SFRS=0;CKCON&=CLR_BIT4
#define clr_CKCON_T0M                     SFRS=0;CKCON&=CLR_BIT3
#define clr_CKCON_T0OE                    SFRS=0;CKCON&=CLR_BIT2
#define clr_CKCON_CLOEN                   SFRS=0;CKCON&=CLR_BIT1
// ;---------------------------------------------
// ; WKCON               
// ;---------------------------------------------
#define set_WKCON_WKTCK                   SFRS=0;WKCON|=SET_BIT5 
#define set_WKCON_WKTF                    SFRS=0;WKCON|=SET_BIT4 
#define set_WKCON_WKTR                    SFRS=0;WKCON|=SET_BIT3 
#define set_WKCON_WKPS2                   SFRS=0;WKCON|=SET_BIT2 
#define set_WKCON_WKPS1                   SFRS=0;WKCON|=SET_BIT1 
#define set_WKCON_WKPS0                   SFRS=0;WKCON|=SET_BIT0 
                                                
#define clr_WKCON_WKTCK                   SFRS=0;WKCON&=~SET_BIT5
#define clr_WKCON_WKTF                    SFRS=0;WKCON&=~SET_BIT4
#define clr_WKCON_WKTR                    SFRS=0;WKCON&=~SET_BIT3
#define clr_WKCON_WKPS2                   SFRS=0;WKCON&=~SET_BIT2
#define clr_WKCON_WKPS1                   SFRS=0;WKCON&=~SET_BIT1
#define clr_WKCON_WKPS0                   SFRS=0;WKCON&=~SET_BIT0
// ;---------------------------------------------
// ; P0   
// ;---------------------------------------------
// ;---------------------------------------------
// ; SP   
// ;---------------------------------------------
// ;---------------------------------------------
// ; DPL   
// ;---------------------------------------------
// ;---------------------------------------------
// ; DPH   
// ;---------------------------------------------
// ;---------------------------------------------
// ; RCTRIM0   
// ;---------------------------------------------
// ;---------------------------------------------
// ; RCTRIM1   
// ;---------------------------------------------
// ;---------------------------------------------
// ; RWK   
// ;---------------------------------------------
// ;---------------------------------------------
// ; PCON
// ;---------------------------------------------
#define set_PCON_SMOD                     SFRS=0;PCON|=SET_BIT7
#define set_PCON_SMOD0                    SFRS=0;PCON|=SET_BIT6
#define set_PCON_LPR                      SFRS=0;PCON|=SET_BIT5
#define set_PCON_POF                      SFRS=0;PCON|=SET_BIT4
#define set_PCON_GF1                      SFRS=0;PCON|=SET_BIT3
#define set_PCON_GF0                      SFRS=0;PCON|=SET_BIT2
#define set_PCON_PD                       SFRS=0;PCON|=SET_BIT1
#define set_PCON_IDLE                     SFRS=0;PCON|=SET_BIT0
                                          
#define clr_PCON_SMOD                     SFRS=0;PCON&=CLR_BIT7
#define clr_PCON_SMOD0                    SFRS=0;PCON&=CLR_BIT6
#define clr_PCON_LPR                      SFRS=0;PCON&=CLR_BIT5
#define clr_PCON_POF                      SFRS=0;PCON&=CLR_BIT4
#define clr_PCON_GF1                      SFRS=0;PCON&=CLR_BIT3
#define clr_PCON_GF0                      SFRS=0;PCON&=CLR_BIT2
#define clr_PCON_PD                       SFRS=0;PCON&=CLR_BIT1
#define clr_PCON_IDLE                     SFRS=0;PCON&=CLR_BIT0
/**********************/
/*	PAGE1         */
/**********************/
// ;---------------------------------------------
// ; PWM0DTEN
// ;---------------------------------------------
#define set_PWM0DTEN_PDT45EN	            SFRS=1;PWM0DTEN|=SET_BIT2
#define set_PWM0DTEN_PDT23EN	            SFRS=1;PWM0DTEN|=SET_BIT1
#define set_PWM0DTEN_PDT01EN              SFRS=1;PWM0DTEN|=SET_BIT0
                                          
#define clr_PWM0DTEN_PDT45EN              SFRS=1;PWM0DTEN&=CLR_BIT2
#define clr_PWM0DTEN_PDT23EN              SFRS=1;PWM0DTEN&=CLR_BIT1
#define clr_PWM0DTEN_PDT01EN              SFRS=1;PWM0DTEN&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0DTCNT
// ;---------------------------------------------
#define set_PWM0DTCNT_PMEN5               SFRS=1;PWM0DTCNT|=SET_BIT4
#define set_PWM0DTCNT_PMEN4               SFRS=1;PWM0DTCNT|=SET_BIT3
#define set_PWM0DTCNT_PMEN3               SFRS=1;PWM0DTCNT|=SET_BIT2
#define set_PWM0DTCNT_PMEN2               SFRS=1;PWM0DTCNT|=SET_BIT1
#define set_PWM0DTCNT_PMEN1               SFRS=1;PWM0DTCNT|=SET_BIT0
                                          
#define clr_PWM0DTCNT_PMEN5               SFRS=1;PWM0DTCNT&=CLR_BIT4
#define clr_PWM0DTCNT_PMEN4               SFRS=1;PWM0DTCNT&=CLR_BIT3
#define clr_PWM0DTCNT_PMEN3               SFRS=1;PWM0DTCNT&=CLR_BIT2
#define clr_PWM0DTCNT_PMEN2               SFRS=1;PWM0DTCNT&=CLR_BIT1
#define clr_PWM0DTCNT_PMEN1               SFRS=1;PWM0DTCNT&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0MD
// ;---------------------------------------------
#define set_PWM0MD_PMEN5                  SFRS=1;PWM0MD|=SET_BIT4
#define set_PWM0MD_PMEN4                  SFRS=1;PWM0MD|=SET_BIT3
#define set_PWM0MD_PMEN3                  SFRS=1;PWM0MD|=SET_BIT2
#define set_PWM0MD_PMEN2                  SFRS=1;PWM0MD|=SET_BIT1
#define set_PWM0MD_PMEN1                  SFRS=1;PWM0MD|=SET_BIT0
                                          
#define clr_PWM0MD_PMEN5                  SFRS=1; PWM0MD&=CLR_BIT4
#define clr_PWM0MD_PMEN4                  SFRS=1; PWM0MD&=CLR_BIT3
#define clr_PWM0MD_PMEN3                  SFRS=1; PWM0MD&=CLR_BIT2
#define clr_PWM0MD_PMEN2                  SFRS=1; PWM0MD&=CLR_BIT1
#define clr_PWM0MD_PMEN1                  SFRS=1; PWM0MD&=CLR_BIT0
// ;---------------------------------------------
// ; LVRFLTEN
// ;---------------------------------------------
// ;---------------------------------------------
// ; LVRDIS
// ;---------------------------------------------
// ;---------------------------------------------
// ; SPI0CR1
// ;---------------------------------------------
#define set_SPI0CR1_SPR3                 SFRS=1;SPI0CR1|=SET_BIT5
#define set_SPI0CR1_SPR2                 SFRS=1;SPI0CR1|=SET_BIT4
#define set_SPI0CR1_TXDMAEN              SFRS=1;SPI0CR1|=SET_BIT3
#define set_SPI0CR1_RXDMAEN              SFRS=1;SPI0CR1|=SET_BIT2
#define set_SPI0CR1_SPIS1                SFRS=1;SPI0CR1|=SET_BIT1
#define set_SPI0CR1_SPIS0                SFRS=1;SPI0CR1|=SET_BIT0
                                        
#define clr_SPI0CR1_SPR3                 SFRS=1;SPI0CR1&=CLR_BIT5
#define clr_SPI0CR1_SPR2                 SFRS=1;SPI0CR1&=CLR_BIT4
#define clr_SPI0CR1_TXDMAEN              SFRS=1;SPI0CR1&=CLR_BIT3
#define clr_SPI0CR1_RXDMAEN              SFRS=1;SPI0CR1&=CLR_BIT2
#define clr_SPI0CR1_SPIS1                SFRS=1;SPI0CR1&=CLR_BIT1
#define clr_SPI0CR1_SPIS0                SFRS=1;SPI0CR1&=CLR_BIT0
// ;---------------------------------------------
// ; LDOOEN
// ;---------------------------------------------
// ;---------------------------------------------
// ; PICON
// ;---------------------------------------------
#define set_PICON_PIT7                  SFRS=1;PICON|=SET_BIT7
#define set_PICON_PIT6                  SFRS=1;PICON|=SET_BIT6
#define set_PICON_PIT5                  SFRS=1;PICON|=SET_BIT5
#define set_PICON_PIT4                  SFRS=1;PICON|=SET_BIT4
#define set_PICON_PIT3                  SFRS=1;PICON|=SET_BIT3
#define set_PICON_PIT2                  SFRS=1;PICON|=SET_BIT2
#define set_PICON_PIT1                  SFRS=1;PICON|=SET_BIT1
#define set_PICON_PIT0                  SFRS=1;PICON|=SET_BIT0
                                        
#define clr_PICON_PIT7                  SFRS=1;PICON&=CLR_BIT7
#define clr_PICON_PIT6                  SFRS=1;PICON&=CLR_BIT6
#define clr_PICON_PIT5                  SFRS=1;PICON&=CLR_BIT5
#define clr_PICON_PIT4                  SFRS=1;PICON&=CLR_BIT4
#define clr_PICON_PIT3                  SFRS=1;PICON&=CLR_BIT3
#define clr_PICON_PIT2                  SFRS=1;PICON&=CLR_BIT2
#define clr_PICON_PIT1                  SFRS=1;PICON&=CLR_BIT1
#define clr_PICON_PIT0                  SFRS=1;PICON&=CLR_BIT0
// ;---------------------------------------------
// ; PINEN
// ;---------------------------------------------
#define set_PINEN_PINEN7                      SFRS=1;PINEN|=SET_BIT7
#define set_PINEN_PINEN6                      SFRS=1;PINEN|=SET_BIT6
#define set_PINEN_PINEN5                      SFRS=1;PINEN|=SET_BIT5
#define set_PINEN_PINEN4                      SFRS=1;PINEN|=SET_BIT4
#define set_PINEN_PINEN3                      SFRS=1;PINEN|=SET_BIT3
#define set_PINEN_PINEN2                      SFRS=1;PINEN|=SET_BIT2
#define set_PINEN_PINEN1                      SFRS=1;PINEN|=SET_BIT1
#define set_PINEN_PINEN0                      SFRS=1;PINEN|=SET_BIT0
                                               
#define clr_PINEN_PINEN7                      SFRS=1;PINEN&=CLR_BIT7
#define clr_PINEN_PINEN6                      SFRS=1;PINEN&=CLR_BIT6
#define clr_PINEN_PINEN5                      SFRS=1;PINEN&=CLR_BIT5
#define clr_PINEN_PINEN4                      SFRS=1;PINEN&=CLR_BIT4
#define clr_PINEN_PINEN3                      SFRS=1;PINEN&=CLR_BIT3
#define clr_PINEN_PINEN2                      SFRS=1;PINEN&=CLR_BIT2
#define clr_PINEN_PINEN1                      SFRS=1;PINEN&=CLR_BIT1
#define clr_PINEN_PINEN0                      SFRS=1;PINEN&=CLR_BIT0
// ;---------------------------------------------
// ; PIPEN
// ;---------------------------------------------
#define set_PIPEN_PIPEN7                      SFRS=1;PIPEN|=SET_BIT7
#define set_PIPEN_PIPEN6                      SFRS=1;PIPEN|=SET_BIT6
#define set_PIPEN_PIPEN5                      SFRS=1;PIPEN|=SET_BIT5
#define set_PIPEN_PIPEN4                      SFRS=1;PIPEN|=SET_BIT4
#define set_PIPEN_PIPEN3                      SFRS=1;PIPEN|=SET_BIT3
#define set_PIPEN_PIPEN2                      SFRS=1;PIPEN|=SET_BIT2
#define set_PIPEN_PIPEN1                      SFRS=1;PIPEN|=SET_BIT1
#define set_PIPEN_PIPEN0                      SFRS=1;PIPEN|=SET_BIT0
                                               
#define clr_PIPEN_PIPEN7                      SFRS=1;PIPEN&=CLR_BIT7
#define clr_PIPEN_PIPEN6                      SFRS=1;PIPEN&=CLR_BIT6
#define clr_PIPEN_PIPEN5                      SFRS=1;PIPEN&=CLR_BIT5
#define clr_PIPEN_PIPEN4                      SFRS=1;PIPEN&=CLR_BIT4
#define clr_PIPEN_PIPEN3                      SFRS=1;PIPEN&=CLR_BIT3
#define clr_PIPEN_PIPEN2                      SFRS=1;PIPEN&=CLR_BIT2
#define clr_PIPEN_PIPEN1                      SFRS=1;PIPEN&=CLR_BIT1
#define clr_PIPEN_PIPEN0                      SFRS=1;PIPEN&=CLR_BIT0
// ;---------------------------------------------
// ; C2L                               
// ;---------------------------------------------
// ;---------------------------------------------
// ; C2H
// ;---------------------------------------------
// ;---------------------------------------------
// ; LDOTRIM
// ;---------------------------------------------
// ;---------------------------------------------
// ; CAPCON0
// ;---------------------------------------------
#define set_CAPCON0_CAPEN2              SFRS=1;CAPCON0|=SET_BIT6
#define set_CAPCON0_CAPEN1              SFRS=1;CAPCON0|=SET_BIT5
#define set_CAPCON0_CAPEN0              SFRS=1;CAPCON0|=SET_BIT4
#define set_CAPCON0_CAPF2               SFRS=1;CAPCON0|=SET_BIT2
#define set_CAPCON0_CAPF1               SFRS=1;CAPCON0|=SET_BIT1
#define set_CAPCON0_CAPF0               SFRS=1;CAPCON0|=SET_BIT0
                                               
#define clr_CAPCON0_CAPEN2              SFRS=1;CAPCON0&=CLR_BIT6
#define clr_CAPCON0_CAPEN1              SFRS=1;CAPCON0&=CLR_BIT5
#define clr_CAPCON0_CAPEN0              SFRS=1;CAPCON0&=CLR_BIT4
#define clr_CAPCON0_CAPF2               SFRS=1;CAPCON0&=CLR_BIT2
#define clr_CAPCON0_CAPF1               SFRS=1;CAPCON0&=CLR_BIT1
#define clr_CAPCON0_CAPF0               SFRS=1;CAPCON0&=CLR_BIT0
// ;---------------------------------------------
// ; CAPCON1
// ;---------------------------------------------
#define set_CAPCON1_CAP2LS1             SFRS=1;CAPCON1|=SET_BIT5
#define set_CAPCON1_CAP2LS0             SFRS=1;CAPCON1|=SET_BIT4
#define set_CAPCON1_CAP1LS1             SFRS=1;CAPCON1|=SET_BIT3
#define set_CAPCON1_CAP1LS0             SFRS=1;CAPCON1|=SET_BIT2
#define set_CAPCON1_CAP0LS1             SFRS=1;CAPCON1|=SET_BIT1
#define set_CAPCON1_CAP0LS0             SFRS=1;CAPCON1|=SET_BIT0
                                               
#define clr_CAPCON1_CAP2LS1             SFRS=1;CAPCON1&=CLR_BIT5
#define clr_CAPCON1_CAP2LS0             SFRS=1;CAPCON1&=CLR_BIT4
#define clr_CAPCON1_CAP1LS1             SFRS=1;CAPCON1&=CLR_BIT3
#define clr_CAPCON1_CAP1LS0             SFRS=1;CAPCON1&=CLR_BIT2
#define clr_CAPCON1_CAP0LS1             SFRS=1;CAPCON1&=CLR_BIT1
#define clr_CAPCON1_CAP0LS0             SFRS=1;CAPCON1&=CLR_BIT0

// ;---------------------------------------------
// ; CAPCON2
// ;---------------------------------------------
#define set_CAPCON2_ENF2                SFRS=1;CAPCON2|=SET_BIT6
#define set_CAPCON2_ENF1                SFRS=1;CAPCON2|=SET_BIT5
#define set_CAPCON2_ENF0                SFRS=1;CAPCON2|=SET_BIT4
                                               
#define clr_CAPCON2_ENF2                SFRS=1;CAPCON1&=CLR_BIT6
#define clr_CAPCON2_ENF1                SFRS=1;CAPCON1&=CLR_BIT5
#define clr_CAPCON2_ENF0                SFRS=1;CAPCON1&=CLR_BIT4

// ;---------------------------------------------
// ; C0L
// ;---------------------------------------------
// ;---------------------------------------------
// ; C0H
// ;---------------------------------------------
// ;---------------------------------------------
// ; C1L
// ;---------------------------------------------
// ;---------------------------------------------
// ; C1H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0PL
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C0L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C1L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C2L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C3L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0IOCON
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0CON1
// ;---------------------------------------------
#define set_PWM0CON1_GP                 SFRS=1;PWM0CON1|=SET_BIT5
#define set_PWM0CON1_PWMTYP             SFRS=1;PWM0CON1|=SET_BIT4
#define set_PWM0CON1_FBINEN             SFRS=1;PWM0CON1|=SET_BIT3
#define set_PWM0CON1_PWMDIV2            SFRS=1;PWM0CON1|=SET_BIT2
#define set_PWM0CON1_PWMDIV1            SFRS=1;PWM0CON1|=SET_BIT1
#define set_PWM0CON1_PWMDIV0            SFRS=1;PWM0CON1|=SET_BIT0
                                               
#define clr_PWM0CON1_GP                 SFRS=1;PWM0CON1&=CLR_BIT5
#define clr_PWM0CON1_PWMTYP             SFRS=1;PWM0CON1&=CLR_BIT4
#define clr_PWM0CON1_FBINEN             SFRS=1;PWM0CON1&=CLR_BIT3
#define clr_PWM0CON1_PWMDIV2            SFRS=1;PWM0CON1&=CLR_BIT2
#define clr_PWM0CON1_PWMDIV1            SFRS=1;PWM0CON1&=CLR_BIT1
#define clr_PWM0CON1_PWMDIV0            SFRS=1;PWM0CON1&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0PH
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C0H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C1H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C2H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C3H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0NP
// ;---------------------------------------------
#define set_PWM0NP_PNP5                SFRS=1;PWM0NP|=SET_BIT5
#define set_PWM0NP_PNP4                SFRS=1;PWM0NP|=SET_BIT4
#define set_PWM0NP_PNP3                SFRS=1;PWM0NP|=SET_BIT3
#define set_PWM0NP_PNP2                SFRS=1;PWM0NP|=SET_BIT2
#define set_PWM0NP_PNP1                SFRS=1;PWM0NP|=SET_BIT1
#define set_PWM0NP_PNP0                SFRS=1;PWM0NP|=SET_BIT0
                                       
#define clr_PWM0NP_PNP5	               SFRS=1;PWM0NP&=CLR_BIT5
#define clr_PWM0NP_PNP4	               SFRS=1;PWM0NP&=CLR_BIT4
#define clr_PWM0NP_PNP3	               SFRS=1;PWM0NP&=CLR_BIT3
#define clr_PWM0NP_PNP2	               SFRS=1;PWM0NP&=CLR_BIT2
#define clr_PWM0NP_PNP1	               SFRS=1;PWM0NP&=CLR_BIT1
#define clr_PWM0NP_PNP0	               SFRS=1;PWM0NP&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0FBD                                      
// ;---------------------------------------------
#define set_PWM0FBD_FBINLS             SFRS=1;PWM0FBD|=SET_BIT6
#define set_PWM0FBD_FBD5               SFRS=1;PWM0FBD|=SET_BIT5
#define set_PWM0FBD_FBD4               SFRS=1;PWM0FBD|=SET_BIT4
#define set_PWM0FBD_FBD3               SFRS=1;PWM0FBD|=SET_BIT3
#define set_PWM0FBD_FBD2               SFRS=1;PWM0FBD|=SET_BIT2
#define set_PWM0FBD_FBD1               SFRS=1;PWM0FBD|=SET_BIT1
#define set_PWM0FBD_FBD0               SFRS=1;PWM0FBD|=SET_BIT0
                                       
#define clr_PWM0FBD_FBF                SFRS=1;PWM0FBD&=CLR_BIT7
#define clr_PWM0FBD_FBINLS             SFRS=1;PWM0FBD&=CLR_BIT6
#define clr_PWM0FBD_FBD5               SFRS=1;PWM0FBD&=CLR_BIT5
#define clr_PWM0FBD_FBD4               SFRS=1;PWM0FBD&=CLR_BIT4
#define clr_PWM0FBD_FBD3               SFRS=1;PWM0FBD&=CLR_BIT3
#define clr_PWM0FBD_FBD2               SFRS=1;PWM0FBD&=CLR_BIT2
#define clr_PWM0FBD_FBD1               SFRS=1;PWM0FBD&=CLR_BIT1
#define clr_PWM0FBD_FBD0               SFRS=1;PWM0FBD&=CLR_BIT0
// ;---------------------------------------------
// ; AUXR1                                      
// ;---------------------------------------------
#define set_AUXR1_SWRF                 SFRS=1;AUXR1|=SET_BIT7
#define set_AUXR1_RSTPINF              SFRS=1;AUXR1|=SET_BIT6
#define set_AUXR1_HardF                SFRS=1;AUXR1|=SET_BIT5
#define set_AUXR1_GF2                  SFRS=1;AUXR1|=SET_BIT3
#define set_AUXR1_UART0PX              SFRS=1;AUXR1|=SET_BIT2
#define set_AUXR1_DPS                  SFRS=1;AUXR1|=SET_BIT0
                                    
#define clr_AUXR1_SWRF                 SFRS=1;AUXR1&=CLR_BIT7
#define clr_AUXR1_RSTPINF              SFRS=1;AUXR1&=CLR_BIT6
#define clr_AUXR1_HardF                SFRS=1;AUXR1&=CLR_BIT5
#define clr_AUXR1_GF2                  SFRS=1;AUXR1&=CLR_BIT3
#define clr_AUXR1_UART0PX              SFRS=1;AUXR1&=CLR_BIT2
#define clr_AUXR1_DPS                  SFRS=1;AUXR1&=CLR_BIT0
// ;---------------------------------------------
// ; RCMP2L                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; RCMP2H                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C4L                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C5L                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; AINDIDS                                      
// ;---------------------------------------------
#define set_AINDIDS_P17DIDS            SFRS=1;AINDIDS|=SET_BIT7
#define set_AINDIDS_P30DIDS            SFRS=1;AINDIDS|=SET_BIT6
#define set_AINDIDS_P07DIDS            SFRS=1;AINDIDS|=SET_BIT5
#define set_AINDIDS_P06DIDS            SFRS=1;AINDIDS|=SET_BIT4
#define set_AINDIDS_P05DIDS            SFRS=1;AINDIDS|=SET_BIT3
#define set_AINDIDS_P04DIDS            SFRS=1;AINDIDS|=SET_BIT2
#define set_AINDIDS_P03DIDS            SFRS=1;AINDIDS|=SET_BIT1
#define set_AINDIDS_P11DIDS            SFRS=1;AINDIDS|=SET_BIT0
                                       
#define clr_AINDIDS_P17DIDS            SFRS=1;AINDIDS&=CLR_BIT7
#define clr_AINDIDS_P30DIDS            SFRS=1;AINDIDS&=CLR_BIT6
#define clr_AINDIDS_P07DIDS            SFRS=1;AINDIDS&=CLR_BIT5
#define clr_AINDIDS_P06DIDS            SFRS=1;AINDIDS&=CLR_BIT4
#define clr_AINDIDS_P05DIDS            SFRS=1;AINDIDS&=CLR_BIT3
#define clr_AINDIDS_P04DIDS            SFRS=1;AINDIDS&=CLR_BIT2
#define clr_AINDIDS_P03DIDS            SFRS=1;AINDIDS&=CLR_BIT1
#define clr_AINDIDS_P11DIDS            SFRS=1;AINDIDS&=CLR_BIT0
// ;---------------------------------------------
// ; CKDIV                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; P3M1                                      
// ;---------------------------------------------
#define set_P3M1_7                     SFRS=1;P3M1|=SET_BIT7
#define set_P3M1_6                     SFRS=1;P3M1|=SET_BIT6
#define set_P3M1_5                     SFRS=1;P3M1|=SET_BIT5
#define set_P3M1_4                     SFRS=1;P3M1|=SET_BIT4
#define set_P3M1_3                     SFRS=1;P3M1|=SET_BIT3
#define set_P3M1_2                     SFRS=1;P3M1|=SET_BIT2
#define set_P3M1_1                     SFRS=1;P3M1|=SET_BIT1
#define set_P3M1_0                     SFRS=1;P3M1|=SET_BIT0
                                       
#define clr_P3M1_7                     SFRS=1;P3M1&=CLR_BIT7
#define clr_P3M1_6                     SFRS=1;P3M1&=CLR_BIT6
#define clr_P3M1_5                     SFRS=1;P3M1&=CLR_BIT5
#define clr_P3M1_4                     SFRS=1;P3M1&=CLR_BIT4
#define clr_P3M1_3                     SFRS=1;P3M1&=CLR_BIT3
#define clr_P3M1_2                     SFRS=1;P3M1&=CLR_BIT2
#define clr_P3M1_1                     SFRS=1;P3M1&=CLR_BIT1
#define clr_P3M1_0                     SFRS=1;P3M1&=CLR_BIT0
// ;---------------------------------------------
// ; P3M2                                      
// ;---------------------------------------------
#define set_P3M2_7                     SFRS=1;P3M2|=SET_BIT7
#define set_P3M2_6                     SFRS=1;P3M2|=SET_BIT6
#define set_P3M2_5                     SFRS=1;P3M2|=SET_BIT5
#define set_P3M2_4                     SFRS=1;P3M2|=SET_BIT4
#define set_P3M2_3                     SFRS=1;P3M2|=SET_BIT3
#define set_P3M2_2                     SFRS=1;P3M2|=SET_BIT2
#define set_P3M2_1                     SFRS=1;P3M2|=SET_BIT1
#define set_P3M2_0                     SFRS=1;P3M2|=SET_BIT0
                                              
#define clr_P3M2_7                     SFRS=1;P3M2&=CLR_BIT7
#define clr_P3M2_6                     SFRS=1;P3M2&=CLR_BIT6
#define clr_P3M2_5                     SFRS=1;P3M2&=CLR_BIT5
#define clr_P3M2_4                     SFRS=1;P3M2&=CLR_BIT4
#define clr_P3M2_3                     SFRS=1;P3M2&=CLR_BIT3
#define clr_P3M2_2                     SFRS=1;P3M2&=CLR_BIT2
#define clr_P3M2_1                     SFRS=1;P3M2&=CLR_BIT1
#define clr_P3M2_0                     SFRS=1;P3M2&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0C4H                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM0C5H                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; PORDIS                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; TA                                      
// ;---------------------------------------------
// ;---------------------------------------------
// ; P4M1                                      
// ;---------------------------------------------
#define set_P4M1_7                     SFRS=1;P4M1|=SET_BIT7
#define set_P4M1_6                     SFRS=1;P4M1|=SET_BIT6
#define set_P4M1_5                     SFRS=1;P4M1|=SET_BIT5
#define set_P4M1_4                     SFRS=1;P4M1|=SET_BIT4
#define set_P4M1_3                     SFRS=1;P4M1|=SET_BIT3
#define set_P4M1_2                     SFRS=1;P4M1|=SET_BIT2
#define set_P4M1_1                     SFRS=1;P4M1|=SET_BIT1
#define set_P4M1_0                     SFRS=1;P4M1|=SET_BIT0
                                              
#define clr_P4M1_7                     SFRS=1;P4M1&=CLR_BIT7
#define clr_P4M1_6                     SFRS=1;P4M1&=CLR_BIT6
#define clr_P4M1_5                     SFRS=1;P4M1&=CLR_BIT5
#define clr_P4M1_4                     SFRS=1;P4M1&=CLR_BIT4
#define clr_P4M1_3                     SFRS=1;P4M1&=CLR_BIT3
#define clr_P4M1_2                     SFRS=1;P4M1&=CLR_BIT2
#define clr_P4M1_1                     SFRS=1;P4M1&=CLR_BIT1
#define clr_P4M1_0                     SFRS=1;P4M1&=CLR_BIT0
// ;---------------------------------------------
// ; P4M2                                      
// ;---------------------------------------------
#define set_P4M2_7                     SFRS=1;P4M2|=SET_BIT7
#define set_P4M2_6                     SFRS=1;P4M2|=SET_BIT6
#define set_P4M2_5                     SFRS=1;P4M2|=SET_BIT5
#define set_P4M2_4                     SFRS=1;P4M2|=SET_BIT4
#define set_P4M2_3                     SFRS=1;P4M2|=SET_BIT3
#define set_P4M2_2                     SFRS=1;P4M2|=SET_BIT2
#define set_P4M2_1                     SFRS=1;P4M2|=SET_BIT1
#define set_P4M2_0                     SFRS=1;P4M2|=SET_BIT0
                                              
#define clr_P4M2_7                     SFRS=1;P4M2&=CLR_BIT7
#define clr_P4M2_6                     SFRS=1;P4M2&=CLR_BIT6
#define clr_P4M2_5                     SFRS=1;P4M2&=CLR_BIT5
#define clr_P4M2_4                     SFRS=1;P4M2&=CLR_BIT4
#define clr_P4M2_3                     SFRS=1;P4M2&=CLR_BIT3
#define clr_P4M2_2                     SFRS=1;P4M2&=CLR_BIT2
#define clr_P4M2_1                     SFRS=1;P4M2&=CLR_BIT1
#define clr_P4M2_0                     SFRS=1;P4M2&=CLR_BIT0
// ;---------------------------------------------  
// ; P4S                                          
// ;---------------------------------------------  
#define set_P4S_7                      SFRS=1;P4S|=SET_BIT7                  
#define set_P4S_6                      SFRS=1;P4S|=SET_BIT6                  
#define set_P4S_5                      SFRS=1;P4S|=SET_BIT5                  
#define set_P4S_4                      SFRS=1;P4S|=SET_BIT4                  
#define set_P4S_3                      SFRS=1;P4S|=SET_BIT3                  
#define set_P4S_2                      SFRS=1;P4S|=SET_BIT2                  
#define set_P4S_1                      SFRS=1;P4S|=SET_BIT1                  
#define set_P4S_0                      SFRS=1;P4S|=SET_BIT0                  
                                                           
#define clr_P4S_7                      SFRS=1;P4S&=CLR_BIT7                  
#define clr_P4S_6                      SFRS=1;P4S&=CLR_BIT6                  
#define clr_P4S_5                      SFRS=1;P4S&=CLR_BIT5                  
#define clr_P4S_4                      SFRS=1;P4S&=CLR_BIT4                  
#define clr_P4S_3                      SFRS=1;P4S&=CLR_BIT3                  
#define clr_P4S_2                      SFRS=1;P4S&=CLR_BIT2                  
#define clr_P4S_1                      SFRS=1;P4S&=CLR_BIT1                  
#define clr_P4S_0                      SFRS=1;P4S&=CLR_BIT0                  
// ;---------------------------------------------  
// ; P4SR                                   
// ;---------------------------------------------  
#define set_P4SR_7                      SFRS=1;P4SR|=SET_BIT7                  
#define set_P4SR_6                      SFRS=1;P4SR|=SET_BIT6                  
#define set_P4SR_5                      SFRS=1;P4SR|=SET_BIT5                  
#define set_P4SR_4                      SFRS=1;P4SR|=SET_BIT4                  
#define set_P4SR_3                      SFRS=1;P4SR|=SET_BIT3                  
#define set_P4SR_2                      SFRS=1;P4SR|=SET_BIT2                  
#define set_P4SR_1                      SFRS=1;P4SR|=SET_BIT1                  
#define set_P4SR_0                      SFRS=1;P4SR|=SET_BIT0                  
                                                           
#define clr_P4SR_7                      SFRS=1;P4SR&=CLR_BIT7                  
#define clr_P4SR_6                      SFRS=1;P4SR&=CLR_BIT6                  
#define clr_P4SR_5                      SFRS=1;P4SR&=CLR_BIT5                  
#define clr_P4SR_4                      SFRS=1;P4SR&=CLR_BIT4                  
#define clr_P4SR_3                      SFRS=1;P4SR&=CLR_BIT3                  
#define clr_P4SR_2                      SFRS=1;P4SR&=CLR_BIT2                  
#define clr_P4SR_1                      SFRS=1;P4SR&=CLR_BIT1                  
#define clr_P4SR_0                      SFRS=1;P4SR&=CLR_BIT0                  
// ;---------------------------------------------
// ; P5M1                                        
// ;---------------------------------------------
#define set_P5M1_6                      SFRS=1;P5M1|=SET_BIT6
#define set_P5M1_5                      SFRS=1;P5M1|=SET_BIT5
#define set_P5M1_4                      SFRS=1;P5M1|=SET_BIT4
#define set_P5M1_3                      SFRS=1;P5M1|=SET_BIT3
#define set_P5M1_2                      SFRS=1;P5M1|=SET_BIT2
#define set_P5M1_1                      SFRS=1;P5M1|=SET_BIT1
#define set_P5M1_0                      SFRS=1;P5M1|=SET_BIT0
                                               
#define clr_P5M1_6                      SFRS=1;P5M1&=CLR_BIT6
#define clr_P5M1_5                      SFRS=1;P5M1&=CLR_BIT5
#define clr_P5M1_4                      SFRS=1;P5M1&=CLR_BIT4
#define clr_P5M1_3                      SFRS=1;P5M1&=CLR_BIT3
#define clr_P5M1_2                      SFRS=1;P5M1&=CLR_BIT2
#define clr_P5M1_1                      SFRS=1;P5M1&=CLR_BIT1
#define clr_P5M1_0                      SFRS=1;P5M1&=CLR_BIT0
// ;---------------------------------------------
// ; P5M2                                        
// ;---------------------------------------------
#define set_P5M2_6                      SFRS=1;P5M2|=SET_BIT6
#define set_P5M2_5                      SFRS=1;P5M2|=SET_BIT5
#define set_P5M2_4                      SFRS=1;P5M2|=SET_BIT4
#define set_P5M2_3                      SFRS=1;P5M2|=SET_BIT3
#define set_P5M2_2                      SFRS=1;P5M2|=SET_BIT2
#define set_P5M2_1                      SFRS=1;P5M2|=SET_BIT1
#define set_P5M2_0                      SFRS=1;P5M2|=SET_BIT0
                                                
#define clr_P5M2_6                      SFRS=1;P5M2&=CLR_BIT6
#define clr_P5M2_5                      SFRS=1;P5M2&=CLR_BIT5
#define clr_P5M2_4                      SFRS=1;P5M2&=CLR_BIT4
#define clr_P5M2_3                      SFRS=1;P5M2&=CLR_BIT3
#define clr_P5M2_2                      SFRS=1;P5M2&=CLR_BIT2
#define clr_P5M2_1                      SFRS=1;P5M2&=CLR_BIT1
#define clr_P5M2_0                      SFRS=1;P5M2&=CLR_BIT0                     
// ;--------------------------------------------  
// ; P5S                                            
// ;--------------------------------------------  
#define set_P5S_7                       SFRS=1;P5S|=SET_BIT7                  
#define set_P5S_6                       SFRS=1;P5S|=SET_BIT6                  
#define set_P5S_5                       SFRS=1;P5S|=SET_BIT5                  
#define set_P5S_4                       SFRS=1;P5S|=SET_BIT4                  
#define set_P5S_3                       SFRS=1;P5S|=SET_BIT3                  
#define set_P5S_2                       SFRS=1;P5S|=SET_BIT2                  
#define set_P5S_1                       SFRS=1;P5S|=SET_BIT1                  
#define set_P5S_0                       SFRS=1;P5S|=SET_BIT0                  
                                                            
#define clr_P5S_7                       SFRS=1;P5S&=CLR_BIT7                  
#define clr_P5S_6                       SFRS=1;P5S&=CLR_BIT6                  
#define clr_P5S_5                       SFRS=1;P5S&=CLR_BIT5                  
#define clr_P5S_4                       SFRS=1;P5S&=CLR_BIT4                  
#define clr_P5S_3                       SFRS=1;P5S&=CLR_BIT3                  
#define clr_P5S_2                       SFRS=1;P5S&=CLR_BIT2                  
#define clr_P5S_1                       SFRS=1;P5S&=CLR_BIT1                  
#define clr_P5S_0                       SFRS=1;P5S&=CLR_BIT0  
// ;---------------------------------------------
// ; P0M1                                        
// ;---------------------------------------------
#define set_P0M1_7                      SFRS=1;P0M1|=SET_BIT7
#define set_P0M1_6                      SFRS=1;P0M1|=SET_BIT6
#define set_P0M1_5                      SFRS=1;P0M1|=SET_BIT5
#define set_P0M1_4                      SFRS=1;P0M1|=SET_BIT4
#define set_P0M1_3                      SFRS=1;P0M1|=SET_BIT3
#define set_P0M1_2                      SFRS=1;P0M1|=SET_BIT2
#define set_P0M1_1                      SFRS=1;P0M1|=SET_BIT1
#define set_P0M1_0                      SFRS=1;P0M1|=SET_BIT0
                                               
#define clr_P0M1_7                      SFRS=1;P0M1&=CLR_BIT7
#define clr_P0M1_6                      SFRS=1;P0M1&=CLR_BIT6
#define clr_P0M1_5                      SFRS=1;P0M1&=CLR_BIT5
#define clr_P0M1_4                      SFRS=1;P0M1&=CLR_BIT4
#define clr_P0M1_3                      SFRS=1;P0M1&=CLR_BIT3
#define clr_P0M1_2                      SFRS=1;P0M1&=CLR_BIT2
#define clr_P0M1_1                      SFRS=1;P0M1&=CLR_BIT1
#define clr_P0M1_0                      SFRS=1;P0M1&=CLR_BIT0
// ;---------------------------------------------
// ; P0M2                                      
// ;---------------------------------------------
#define set_P0M2_7                      SFRS=1;P0M2|=SET_BIT7
#define set_P0M2_6                      SFRS=1;P0M2|=SET_BIT6
#define set_P0M2_5                      SFRS=1;P0M2|=SET_BIT5
#define set_P0M2_4                      SFRS=1;P0M2|=SET_BIT4
#define set_P0M2_3                      SFRS=1;P0M2|=SET_BIT3
#define set_P0M2_2                      SFRS=1;P0M2|=SET_BIT2
#define set_P0M2_1                      SFRS=1;P0M2|=SET_BIT1
#define set_P0M2_0                      SFRS=1;P0M2|=SET_BIT0
                                               
#define clr_P0M2_7                      SFRS=1;P0M2&=CLR_BIT7
#define clr_P0M2_6                      SFRS=1;P0M2&=CLR_BIT6
#define clr_P0M2_5                      SFRS=1;P0M2&=CLR_BIT5
#define clr_P0M2_4                      SFRS=1;P0M2&=CLR_BIT4
#define clr_P0M2_3                      SFRS=1;P0M2&=CLR_BIT3
#define clr_P0M2_2                      SFRS=1;P0M2&=CLR_BIT2
#define clr_P0M2_1                      SFRS=1;P0M2&=CLR_BIT1
#define clr_P0M2_0                      SFRS=1;P0M2&=CLR_BIT0
// ;---------------------------------------------
// ; P1M1                                      
// ;---------------------------------------------
#define set_P1M1_7                      SFRS=1;P1M1|=SET_BIT7
#define set_P1M1_6                      SFRS=1;P1M1|=SET_BIT6
#define set_P1M1_5                      SFRS=1;P1M1|=SET_BIT5
#define set_P1M1_4                      SFRS=1;P1M1|=SET_BIT4
#define set_P1M1_3                      SFRS=1;P1M1|=SET_BIT3
#define set_P1M1_2                      SFRS=1;P1M1|=SET_BIT2
#define set_P1M1_1                      SFRS=1;P1M1|=SET_BIT1
#define set_P1M1_0                      SFRS=1;P1M1|=SET_BIT0
                                               
#define clr_P1M1_7                      SFRS=1;P1M1&=CLR_BIT7
#define clr_P1M1_6                      SFRS=1;P1M1&=CLR_BIT6
#define clr_P1M1_5                      SFRS=1;P1M1&=CLR_BIT5
#define clr_P1M1_4                      SFRS=1;P1M1&=CLR_BIT4
#define clr_P1M1_3                      SFRS=1;P1M1&=CLR_BIT3
#define clr_P1M1_2                      SFRS=1;P1M1&=CLR_BIT2
#define clr_P1M1_1                      SFRS=1;P1M1&=CLR_BIT1
#define clr_P1M1_0                      SFRS=1;P1M1&=CLR_BIT0
// ;---------------------------------------------
// ; P1M2                                      
// ;---------------------------------------------
#define set_P1M2_7                      SFRS=1;P1M2|=SET_BIT7
#define set_P1M2_6                      SFRS=1;P1M2|=SET_BIT6
#define set_P1M2_5                      SFRS=1;P1M2|=SET_BIT5
#define set_P1M2_4                      SFRS=1;P1M2|=SET_BIT4
#define set_P1M2_3                      SFRS=1;P1M2|=SET_BIT3
#define set_P1M2_2                      SFRS=1;P1M2|=SET_BIT2
#define set_P1M2_1                      SFRS=1;P1M2|=SET_BIT1
#define set_P1M2_0                      SFRS=1;P1M2|=SET_BIT0
                                               
#define clr_P1M2_7                      SFRS=1;P1M2&=CLR_BIT7
#define clr_P1M2_6                      SFRS=1;P1M2&=CLR_BIT6
#define clr_P1M2_5                      SFRS=1;P1M2&=CLR_BIT5
#define clr_P1M2_4                      SFRS=1;P1M2&=CLR_BIT4
#define clr_P1M2_3                      SFRS=1;P1M2&=CLR_BIT3
#define clr_P1M2_2                      SFRS=1;P1M2&=CLR_BIT2
#define clr_P1M2_1                      SFRS=1;P1M2&=CLR_BIT1
#define clr_P1M2_0                      SFRS=1;P1M2&=CLR_BIT0
// ;---------------------------------------------
// ; P2M1                                      
// ;---------------------------------------------
#define set_P2M1_7                      SFRS=1;P2M1|=SET_BIT7
#define set_P2M1_6                      SFRS=1;P2M1|=SET_BIT6
#define set_P2M1_5                      SFRS=1;P2M1|=SET_BIT5
#define set_P2M1_4                      SFRS=1;P2M1|=SET_BIT4
#define set_P2M1_3                      SFRS=1;P2M1|=SET_BIT3
#define set_P2M1_2                      SFRS=1;P2M1|=SET_BIT2
#define set_P2M1_1                      SFRS=1;P2M1|=SET_BIT1
#define set_P2M1_0                      SFRS=1;P2M1|=SET_BIT0
                                               
#define clr_P2M1_7                      SFRS=1;P2M1&=CLR_BIT7
#define clr_P2M1_6                      SFRS=1;P2M1&=CLR_BIT6
#define clr_P2M1_5                      SFRS=1;P2M1&=CLR_BIT5
#define clr_P2M1_4                      SFRS=1;P2M1&=CLR_BIT4
#define clr_P2M1_3                      SFRS=1;P2M1&=CLR_BIT3
#define clr_P2M1_2                      SFRS=1;P2M1&=CLR_BIT2
#define clr_P2M1_1                      SFRS=1;P2M1&=CLR_BIT1
#define clr_P2M1_0                      SFRS=1;P2M1&=CLR_BIT0
// ;---------------------------------------------
// ; P2M2                                      
// ;---------------------------------------------
#define set_P2M2_7                      SFRS=1;P2M2|=SET_BIT7
#define set_P2M2_6                      SFRS=1;P2M2|=SET_BIT6
#define set_P2M2_5                      SFRS=1;P2M2|=SET_BIT5
#define set_P2M2_4                      SFRS=1;P2M2|=SET_BIT4
#define set_P2M2_3                      SFRS=1;P2M2|=SET_BIT3
#define set_P2M2_2                      SFRS=1;P2M2|=SET_BIT2
#define set_P2M2_1                      SFRS=1;P2M2|=SET_BIT1
#define set_P2M2_0                      SFRS=1;P2M2|=SET_BIT0
                                               
#define clr_P2M2_7                      SFRS=1;P2M2&=CLR_BIT7
#define clr_P2M2_6                      SFRS=1;P2M2&=CLR_BIT6
#define clr_P2M2_5                      SFRS=1;P2M2&=CLR_BIT5
#define clr_P2M2_4                      SFRS=1;P2M2&=CLR_BIT4
#define clr_P2M2_3                      SFRS=1;P2M2&=CLR_BIT3
#define clr_P2M2_2                      SFRS=1;P2M2&=CLR_BIT2
#define clr_P2M2_1                      SFRS=1;P2M2&=CLR_BIT1
#define clr_P2M2_0                      SFRS=1;P2M2&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0INTC                                      
// ;---------------------------------------------
#define set_PWM0INTC_INTTYP1            SFRS=1;PWM0INTC|=SET_BIT5
#define set_PWM0INTC_INTTYP0            SFRS=1;PWM0INTC|=SET_BIT4
#define set_PWM0INTC_INTSEL2            SFRS=1;PWM0INTC|=SET_BIT2
#define set_PWM0INTC_INTSEL1            SFRS=1;PWM0INTC|=SET_BIT1
#define set_PWM0INTC_INTSEL0            SFRS=1;PWM0INTC|=SET_BIT0
                                        
#define clr_PWM0INTC_INTTYP1            SFRS=1;PWM0INTC&=CLR_BIT5
#define clr_PWM0INTC_INTTYP0            SFRS=1;PWM0INTC&=CLR_BIT4
#define clr_PWM0INTC_INTSEL2            SFRS=1;PWM0INTC&=CLR_BIT2
#define clr_PWM0INTC_INTSEL1            SFRS=1;PWM0INTC&=CLR_BIT1
#define clr_PWM0INTC_INTSEL0            SFRS=1;PWM0INTC&=CLR_BIT0
// ;---------------------------------------------
// ; ACMPCR2                                      
// ;---------------------------------------------
#define set_ACMPCR2_AO1PEN	        SFRS=1;ACMPCR2|=SET_BIT5
#define set_ACMPCR2_AO0PEN	        SFRS=1;ACMPCR2|=SET_BIT4
#define set_ACMPCR2_CRVSSEL	        SFRS=1;ACMPCR2|=SET_BIT1
#define set_ACMPCR2_CRVEN	        SFRS=1;ACMPCR2|=SET_BIT0
                                        
#define clr_ACMPCR2_AO1PEN	        SFRS=1;ACMPCR2&=CLR_BIT5
#define clr_ACMPCR2_AO0PEN	        SFRS=1;ACMPCR2&=CLR_BIT4
#define clr_ACMPCR2_CRVSSEL	        SFRS=1;ACMPCR2&=CLR_BIT1
#define clr_ACMPCR2_CRVEN	        SFRS=1;ACMPCR2&=CLR_BIT0
// ;---------------------------------------------  
// ; P3S                                          
// ;---------------------------------------------  
#define set_P3S_7                       SFRS=1;P3S|=SET_BIT7                  
#define set_P3S_6                       SFRS=1;P3S|=SET_BIT6                  
#define set_P3S_5                       SFRS=1;P3S|=SET_BIT5                  
#define set_P3S_4                       SFRS=1;P3S|=SET_BIT4                  
#define set_P3S_3                       SFRS=1;P3S|=SET_BIT3                  
#define set_P3S_2                       SFRS=1;P3S|=SET_BIT2                  
#define set_P3S_1                       SFRS=1;P3S|=SET_BIT1                  
#define set_P3S_0                       SFRS=1;P3S|=SET_BIT0                  
                                               
#define clr_P3S_7                       SFRS=1;P3S&=CLR_BIT7                  
#define clr_P3S_6                       SFRS=1;P3S&=CLR_BIT6                  
#define clr_P3S_5                       SFRS=1;P3S&=CLR_BIT5                  
#define clr_P3S_4                       SFRS=1;P3S&=CLR_BIT4                  
#define clr_P3S_3                       SFRS=1;P3S&=CLR_BIT3                  
#define clr_P3S_2                       SFRS=1;P3S&=CLR_BIT2                  
#define clr_P3S_1                       SFRS=1;P3S&=CLR_BIT1                  
#define clr_P3S_0                       SFRS=1;P3S&=CLR_BIT0
// ;---------------------------------------------  
// ; P3SR                                   
// ;---------------------------------------------  
#define set_P3SR_7                      SFRS=1;P3SR|=SET_BIT7                  
#define set_P3SR_6                      SFRS=1;P3SR|=SET_BIT6                  
#define set_P3SR_5                      SFRS=1;P3SR|=SET_BIT5                  
#define set_P3SR_4                      SFRS=1;P3SR|=SET_BIT4                  
#define set_P3SR_3                      SFRS=1;P3SR|=SET_BIT3                  
#define set_P3SR_2                      SFRS=1;P3SR|=SET_BIT2                  
#define set_P3SR_1                      SFRS=1;P3SR|=SET_BIT1                  
#define set_P3SR_0                      SFRS=1;P3SR|=SET_BIT0                  
                                               
#define clr_P3SR_7                      SFRS=1;P3SR&=CLR_BIT7                  
#define clr_P3SR_6                      SFRS=1;P3SR&=CLR_BIT6                  
#define clr_P3SR_5                      SFRS=1;P3SR&=CLR_BIT5                  
#define clr_P3SR_4                      SFRS=1;P3SR&=CLR_BIT4                  
#define clr_P3SR_3                      SFRS=1;P3SR&=CLR_BIT3                  
#define clr_P3SR_2                      SFRS=1;P3SR&=CLR_BIT2                  
#define clr_P3SR_1                      SFRS=1;P3SR&=CLR_BIT1                  
#define clr_P3SR_0                      SFRS=1;P3SR&=CLR_BIT0       
// ;---------------------------------------------  
// ; P5SR                                   
// ;---------------------------------------------  
#define set_P5SR_7                      SFRS=1;P5SR|=SET_BIT7                  
#define set_P5SR_6                      SFRS=1;P5SR|=SET_BIT6                  
#define set_P5SR_5                      SFRS=1;P5SR|=SET_BIT5                  
#define set_P5SR_4                      SFRS=1;P5SR|=SET_BIT4                  
#define set_P5SR_3                      SFRS=1;P5SR|=SET_BIT3                  
#define set_P5SR_2                      SFRS=1;P5SR|=SET_BIT2                  
#define set_P5SR_1                      SFRS=1;P5SR|=SET_BIT1                  
#define set_P5SR_0                      SFRS=1;P5SR|=SET_BIT0                  
                                               
#define clr_P5SR_7                      SFRS=1;P5SR&=CLR_BIT7                  
#define clr_P5SR_6                      SFRS=1;P5SR&=CLR_BIT6                  
#define clr_P5SR_5                      SFRS=1;P5SR&=CLR_BIT5                  
#define clr_P5SR_4                      SFRS=1;P5SR&=CLR_BIT4                  
#define clr_P5SR_3                      SFRS=1;P5SR&=CLR_BIT3                  
#define clr_P5SR_2                      SFRS=1;P5SR&=CLR_BIT2                  
#define clr_P5SR_1                      SFRS=1;P5SR&=CLR_BIT1                  
#define clr_P5SR_0                      SFRS=1;P5SR&=CLR_BIT0       
// ;---------------------------------------------  
// ; PIPS7                                   
// ;---------------------------------------------  
#define set_PIPS7_PSEL2                 SFRS=1;PIPS7|=SET_BIT6                  
#define set_PIPS7_PSEL1                 SFRS=1;PIPS7|=SET_BIT5                  
#define set_PIPS7_PSEL0                 SFRS=1;PIPS7|=SET_BIT4                  
#define set_PIPS7_BSEL2                 SFRS=1;PIPS7|=SET_BIT2                  
#define set_PIPS7_BSEL1                 SFRS=1;PIPS7|=SET_BIT1                  
#define set_PIPS7_BSEL0                 SFRS=1;PIPS7|=SET_BIT0                  
                                               
#define clr_PIPS7_PSEL2                 SFRS=1;PIPS7&=CLR_BIT6                  
#define clr_PIPS7_PSEL1                 SFRS=1;PIPS7&=CLR_BIT5                  
#define clr_PIPS7_PSEL0                 SFRS=1;PIPS7&=CLR_BIT4                  
#define clr_PIPS7_BSEL2                 SFRS=1;PIPS7&=CLR_BIT2                  
#define clr_PIPS7_BSEL1                 SFRS=1;PIPS7&=CLR_BIT1                  
#define clr_PIPS7_BSEL0                 SFRS=1;PIPS7&=CLR_BIT0       
// ;---------------------------------------------  
// ; PIPS0  
// ;---------------------------------------------  
#define set_PIPS0_PSEL2                 SFRS=1;PIPS0|=SET_BIT6                  
#define set_PIPS0_PSEL1                 SFRS=1;PIPS0|=SET_BIT5                  
#define set_PIPS0_PSEL0                 SFRS=1;PIPS0|=SET_BIT4                  
#define set_PIPS0_BSEL2                 SFRS=1;PIPS0|=SET_BIT2                  
#define set_PIPS0_BSEL1                 SFRS=1;PIPS0|=SET_BIT1                  
#define set_PIPS0_BSEL0                 SFRS=1;PIPS0|=SET_BIT0                  
                                               
#define clr_PIPS0_PSEL2                 SFRS=1;PIPS0&=CLR_BIT6                  
#define clr_PIPS0_PSEL1                 SFRS=1;PIPS0&=CLR_BIT5                  
#define clr_PIPS0_PSEL0                 SFRS=1;PIPS0&=CLR_BIT4                  
#define clr_PIPS0_BSEL2                 SFRS=1;PIPS0&=CLR_BIT2                  
#define clr_PIPS0_BSEL1                 SFRS=1;PIPS0&=CLR_BIT1                  
#define clr_PIPS0_BSEL0                 SFRS=1;PIPS0&=CLR_BIT0  
// ;---------------------------------------------  
// ; PIPS1                                   
// ;---------------------------------------------  
#define set_PIPS1_PSEL2                 SFRS=1;PIPS1|=SET_BIT6                  
#define set_PIPS1_PSEL1                 SFRS=1;PIPS1|=SET_BIT5                  
#define set_PIPS1_PSEL0                 SFRS=1;PIPS1|=SET_BIT4                  
#define set_PIPS1_BSEL2                 SFRS=1;PIPS1|=SET_BIT2                  
#define set_PIPS1_BSEL1                 SFRS=1;PIPS1|=SET_BIT1                  
#define set_PIPS1_BSEL0                 SFRS=1;PIPS1|=SET_BIT0                  
                                               
#define clr_PIPS1_PSEL2                 SFRS=1;PIPS1&=CLR_BIT6                  
#define clr_PIPS1_PSEL1                 SFRS=1;PIPS1&=CLR_BIT5                  
#define clr_PIPS1_PSEL0                 SFRS=1;PIPS1&=CLR_BIT4                  
#define clr_PIPS1_BSEL2                 SFRS=1;PIPS1&=CLR_BIT2                  
#define clr_PIPS1_BSEL1                 SFRS=1;PIPS1&=CLR_BIT1                  
#define clr_PIPS1_BSEL0                 SFRS=1;PIPS1&=CLR_BIT0  
// ;---------------------------------------------  
// ; PIPS2                                   
// ;---------------------------------------------  
#define set_PIPS2_PSEL2                 SFRS=1;PIPS2|=SET_BIT6                  
#define set_PIPS2_PSEL1                 SFRS=1;PIPS2|=SET_BIT5                  
#define set_PIPS2_PSEL0                 SFRS=1;PIPS2|=SET_BIT4                  
#define set_PIPS2_BSEL2                 SFRS=1;PIPS2|=SET_BIT2                  
#define set_PIPS2_BSEL1                 SFRS=1;PIPS2|=SET_BIT1                  
#define set_PIPS2_BSEL0                 SFRS=1;PIPS2|=SET_BIT0
                                                
#define clr_PIPS2_PSEL2                 SFRS=1;PIPS2&=CLR_BIT6                  
#define clr_PIPS2_PSEL1                 SFRS=1;PIPS2&=CLR_BIT5                  
#define clr_PIPS2_PSEL0                 SFRS=1;PIPS2&=CLR_BIT4                  
#define clr_PIPS2_BSEL2                 SFRS=1;PIPS2&=CLR_BIT2                  
#define clr_PIPS2_BSEL1                 SFRS=1;PIPS2&=CLR_BIT1                  
#define clr_PIPS2_BSEL0                 SFRS=1;PIPS2&=CLR_BIT0  
// ;---------------------------------------------  
// ; PIPS3                                   
// ;---------------------------------------------  
#define set_PIPS3_PSEL2                 SFRS=1;PIPS3|=SET_BIT6                  
#define set_PIPS3_PSEL1                 SFRS=1;PIPS3|=SET_BIT5                  
#define set_PIPS3_PSEL0                 SFRS=1;PIPS3|=SET_BIT4                  
#define set_PIPS3_BSEL2                 SFRS=1;PIPS3|=SET_BIT2                  
#define set_PIPS3_BSEL1                 SFRS=1;PIPS3|=SET_BIT1                  
#define set_PIPS3_BSEL0                 SFRS=1;PIPS3|=SET_BIT0                  
                                               
#define clr_PIPS3_PSEL2                 SFRS=1;PIPS3&=CLR_BIT6                  
#define clr_PIPS3_PSEL1                 SFRS=1;PIPS3&=CLR_BIT5                  
#define clr_PIPS3_PSEL0                 SFRS=1;PIPS3&=CLR_BIT4                  
#define clr_PIPS3_BSEL2                 SFRS=1;PIPS3&=CLR_BIT2                  
#define clr_PIPS3_BSEL1                 SFRS=1;PIPS3&=CLR_BIT1                  
#define clr_PIPS3_BSEL0                 SFRS=1;PIPS3&=CLR_BIT0  
// ;---------------------------------------------  
// ; PIPS4                                   
// ;---------------------------------------------  
#define set_PIPS4_PSEL2                 SFRS=1;PIPS4|=SET_BIT6                  
#define set_PIPS4_PSEL1                 SFRS=1;PIPS4|=SET_BIT5                  
#define set_PIPS4_PSEL0                 SFRS=1;PIPS4|=SET_BIT4                  
#define set_PIPS4_BSEL2                 SFRS=1;PIPS4|=SET_BIT2                  
#define set_PIPS4_BSEL1                 SFRS=1;PIPS4|=SET_BIT1                  
#define set_PIPS4_BSEL0                 SFRS=1;PIPS4|=SET_BIT0                  
                                               
#define clr_PIPS4_PSEL2                 SFRS=1;PIPS4&=CLR_BIT6                  
#define clr_PIPS4_PSEL1                 SFRS=1;PIPS4&=CLR_BIT5                  
#define clr_PIPS4_PSEL0                 SFRS=1;PIPS4&=CLR_BIT4                  
#define clr_PIPS4_BSEL2                 SFRS=1;PIPS4&=CLR_BIT2                  
#define clr_PIPS4_BSEL1                 SFRS=1;PIPS4&=CLR_BIT1                  
#define clr_PIPS4_BSEL0                 SFRS=1;PIPS4&=CLR_BIT0  
// ;---------------------------------------------  
// ; PIPS5                                   
// ;---------------------------------------------  
#define set_PIPS5_PSEL2                 SFRS=1;PIPS5|=SET_BIT6                  
#define set_PIPS5_PSEL1                 SFRS=1;PIPS5|=SET_BIT5                  
#define set_PIPS5_PSEL0                 SFRS=1;PIPS5|=SET_BIT4                  
#define set_PIPS5_BSEL2                 SFRS=1;PIPS5|=SET_BIT2                  
#define set_PIPS5_BSEL1                 SFRS=1;PIPS5|=SET_BIT1                  
#define set_PIPS5_BSEL0                 SFRS=1;PIPS5|=SET_BIT0                  
                                               
#define clr_PIPS5_PSEL2                 SFRS=1;PIPS5&=CLR_BIT6                  
#define clr_PIPS5_PSEL1                 SFRS=1;PIPS5&=CLR_BIT5                  
#define clr_PIPS5_PSEL0                 SFRS=1;PIPS5&=CLR_BIT4                  
#define clr_PIPS5_BSEL2                 SFRS=1;PIPS5&=CLR_BIT2                  
#define clr_PIPS5_BSEL1                 SFRS=1;PIPS5&=CLR_BIT1                  
#define clr_PIPS5_BSEL0                 SFRS=1;PIPS5&=CLR_BIT0  
// ;---------------------------------------------  
// ; PIPS6                                   
// ;---------------------------------------------  
#define set_PIPS6_PSEL2                 SFRS=1;PIPS6|=SET_BIT6                  
#define set_PIPS6_PSEL1                 SFRS=1;PIPS6|=SET_BIT5                  
#define set_PIPS6_PSEL0                 SFRS=1;PIPS6|=SET_BIT4                  
#define set_PIPS6_BSEL2                 SFRS=1;PIPS6|=SET_BIT2                  
#define set_PIPS6_BSEL1                 SFRS=1;PIPS6|=SET_BIT1                  
#define set_PIPS6_BSEL0                 SFRS=1;PIPS6|=SET_BIT0                  
                                               
#define clr_PIPS6_PSEL2                 SFRS=1;PIPS6&=CLR_BIT6                  
#define clr_PIPS6_PSEL1                 SFRS=1;PIPS6&=CLR_BIT5                  
#define clr_PIPS6_PSEL0                 SFRS=1;PIPS6&=CLR_BIT4                  
#define clr_PIPS6_BSEL2                 SFRS=1;PIPS6&=CLR_BIT2                  
#define clr_PIPS6_BSEL1                 SFRS=1;PIPS6&=CLR_BIT1                  
#define clr_PIPS6_BSEL0                 SFRS=1;PIPS6&=CLR_BIT0  
// ;---------------------------------------------  
// ; P0S                                          
// ;---------------------------------------------  
#define set_P0S_7                       SFRS=1;P0S|=SET_BIT7                  
#define set_P0S_6                       SFRS=1;P0S|=SET_BIT6                  
#define set_P0S_5                       SFRS=1;P0S|=SET_BIT5                  
#define set_P0S_4                       SFRS=1;P0S|=SET_BIT4                  
#define set_P0S_3                       SFRS=1;P0S|=SET_BIT3                  
#define set_P0S_2                       SFRS=1;P0S|=SET_BIT2                  
#define set_P0S_1                       SFRS=1;P0S|=SET_BIT1                  
#define set_P0S_0                       SFRS=1;P0S|=SET_BIT0                  
                                               
#define clr_P0S_7                       SFRS=1;P0S&=CLR_BIT7                  
#define clr_P0S_6                       SFRS=1;P0S&=CLR_BIT6                  
#define clr_P0S_5                       SFRS=1;P0S&=CLR_BIT5                  
#define clr_P0S_4                       SFRS=1;P0S&=CLR_BIT4                  
#define clr_P0S_3                       SFRS=1;P0S&=CLR_BIT3                  
#define clr_P0S_2                       SFRS=1;P0S&=CLR_BIT2                  
#define clr_P0S_1                       SFRS=1;P0S&=CLR_BIT1                  
#define clr_P0S_0                       SFRS=1;P0S&=CLR_BIT0
// ;---------------------------------------------  
// ; P0SR                                   
// ;---------------------------------------------  
#define set_P0SR_7                      SFRS=1;P0SR|=SET_BIT7                  
#define set_P0SR_6                      SFRS=1;P0SR|=SET_BIT6                  
#define set_P0SR_5                      SFRS=1;P0SR|=SET_BIT5                  
#define set_P0SR_4                      SFRS=1;P0SR|=SET_BIT4                  
#define set_P0SR_3                      SFRS=1;P0SR|=SET_BIT3                  
#define set_P0SR_2                      SFRS=1;P0SR|=SET_BIT2                  
#define set_P0SR_1                      SFRS=1;P0SR|=SET_BIT1                  
#define set_P0SR_0                      SFRS=1;P0SR|=SET_BIT0                  
                                               
#define clr_P0SR_7                      SFRS=1;P0SR&=CLR_BIT7                  
#define clr_P0SR_6                      SFRS=1;P0SR&=CLR_BIT6                  
#define clr_P0SR_5                      SFRS=1;P0SR&=CLR_BIT5                  
#define clr_P0SR_4                      SFRS=1;P0SR&=CLR_BIT4                  
#define clr_P0SR_3                      SFRS=1;P0SR&=CLR_BIT3                  
#define clr_P0SR_2                      SFRS=1;P0SR&=CLR_BIT2                  
#define clr_P0SR_1                      SFRS=1;P0SR&=CLR_BIT1                  
#define clr_P0SR_0                      SFRS=1;P0SR&=CLR_BIT0       
// ;---------------------------------------------  
// ; P1S                                          
// ;---------------------------------------------  
#define set_P1S_7                       SFRS=1;P1S|=SET_BIT7                  
#define set_P1S_6                       SFRS=1;P1S|=SET_BIT6                  
#define set_P1S_5                       SFRS=1;P1S|=SET_BIT5                  
#define set_P1S_4                       SFRS=1;P1S|=SET_BIT4                  
#define set_P1S_3                       SFRS=1;P1S|=SET_BIT3                  
#define set_P1S_2                       SFRS=1;P1S|=SET_BIT2                  
#define set_P1S_1                       SFRS=1;P1S|=SET_BIT1                  
#define set_P1S_0                       SFRS=1;P1S|=SET_BIT0                  
                                                 
#define clr_P1S_7                       SFRS=1;P1S&=CLR_BIT7                  
#define clr_P1S_6                       SFRS=1;P1S&=CLR_BIT6                  
#define clr_P1S_5                       SFRS=1;P1S&=CLR_BIT5                  
#define clr_P1S_4                       SFRS=1;P1S&=CLR_BIT4                  
#define clr_P1S_3                       SFRS=1;P1S&=CLR_BIT3                  
#define clr_P1S_2                       SFRS=1;P1S&=CLR_BIT2                  
#define clr_P1S_1                       SFRS=1;P1S&=CLR_BIT1                  
#define clr_P1S_0                       SFRS=1;P1S&=CLR_BIT0
// ;---------------------------------------------  
// ; P1SR                                   
// ;---------------------------------------------  
#define set_P1SR_7                      SFRS=1;P1SR|=SET_BIT7                  
#define set_P1SR_6                      SFRS=1;P1SR|=SET_BIT6                  
#define set_P1SR_5                      SFRS=1;P1SR|=SET_BIT5                  
#define set_P1SR_4                      SFRS=1;P1SR|=SET_BIT4                  
#define set_P1SR_3                      SFRS=1;P1SR|=SET_BIT3                  
#define set_P1SR_2                      SFRS=1;P1SR|=SET_BIT2                  
#define set_P1SR_1                      SFRS=1;P1SR|=SET_BIT1                  
#define set_P1SR_0                      SFRS=1;P1SR|=SET_BIT0                  
                                                 
#define clr_P1SR_7                      SFRS=1;P1SR&=CLR_BIT7                  
#define clr_P1SR_6                      SFRS=1;P1SR&=CLR_BIT6                  
#define clr_P1SR_5                      SFRS=1;P1SR&=CLR_BIT5                  
#define clr_P1SR_4                      SFRS=1;P1SR&=CLR_BIT4                  
#define clr_P1SR_3                      SFRS=1;P1SR&=CLR_BIT3                  
#define clr_P1SR_2                      SFRS=1;P1SR&=CLR_BIT2                  
#define clr_P1SR_1                      SFRS=1;P1SR&=CLR_BIT1                  
#define clr_P1SR_0                      SFRS=1;P1SR&=CLR_BIT0       
// ;---------------------------------------------  
// ; P2S                                          
// ;---------------------------------------------  
#define set_P2S_7                       SFRS=1;P2S|=SET_BIT7                  
#define set_P2S_6                       SFRS=1;P2S|=SET_BIT6                  
#define set_P2S_5                       SFRS=1;P2S|=SET_BIT5                  
#define set_P2S_4                       SFRS=1;P2S|=SET_BIT4                  
#define set_P2S_3                       SFRS=1;P2S|=SET_BIT3                  
#define set_P2S_2                       SFRS=1;P2S|=SET_BIT2                  
#define set_P2S_1                       SFRS=1;P2S|=SET_BIT1                  
#define set_P2S_0                       SFRS=1;P2S|=SET_BIT0                  
                                                 
#define clr_P2S_7                       SFRS=1;P2S&=CLR_BIT7                  
#define clr_P2S_6                       SFRS=1;P2S&=CLR_BIT6                  
#define clr_P2S_5                       SFRS=1;P2S&=CLR_BIT5                  
#define clr_P2S_4                       SFRS=1;P2S&=CLR_BIT4                  
#define clr_P2S_3                       SFRS=1;P2S&=CLR_BIT3                  
#define clr_P2S_2                       SFRS=1;P2S&=CLR_BIT2                  
#define clr_P2S_1                       SFRS=1;P2S&=CLR_BIT1                  
#define clr_P2S_0                       SFRS=1;P2S&=CLR_BIT0
// ;---------------------------------------------  
// ; P2SR                                   
// ;---------------------------------------------  
#define set_P2SR_7                      SFRS=1;P2SR|=SET_BIT7                  
#define set_P2SR_6                      SFRS=1;P2SR|=SET_BIT6                  
#define set_P2SR_5                      SFRS=1;P2SR|=SET_BIT5                  
#define set_P2SR_4                      SFRS=1;P2SR|=SET_BIT4                  
#define set_P2SR_3                      SFRS=1;P2SR|=SET_BIT3                  
#define set_P2SR_2                      SFRS=1;P2SR|=SET_BIT2                  
#define set_P2SR_1                      SFRS=1;P2SR|=SET_BIT1                  
#define set_P2SR_0                      SFRS=1;P2SR|=SET_BIT0                  
                                                 
#define clr_P2SR_7                      SFRS=1;P2SR&=CLR_BIT7                  
#define clr_P2SR_6                      SFRS=1;P2SR&=CLR_BIT6                  
#define clr_P2SR_5                      SFRS=1;P2SR&=CLR_BIT5                  
#define clr_P2SR_4                      SFRS=1;P2SR&=CLR_BIT4                  
#define clr_P2SR_3                      SFRS=1;P2SR&=CLR_BIT3                  
#define clr_P2SR_2                      SFRS=1;P2SR&=CLR_BIT2                  
#define clr_P2SR_1                      SFRS=1;P2SR&=CLR_BIT1                  
#define clr_P2SR_0                      SFRS=1;P2SR&=CLR_BIT0       
// ;---------------------------------------------  
// ; P0UP                                   
// ;---------------------------------------------  
#define set_P0UP_7                      SFRS=1;P0UP|=SET_BIT7                  
#define set_P0UP_6                      SFRS=1;P0UP|=SET_BIT6                  
#define set_P0UP_5                      SFRS=1;P0UP|=SET_BIT5                  
#define set_P0UP_4                      SFRS=1;P0UP|=SET_BIT4                  
#define set_P0UP_3                      SFRS=1;P0UP|=SET_BIT3                  
#define set_P0UP_2                      SFRS=1;P0UP|=SET_BIT2                  
#define set_P0UP_1                      SFRS=1;P0UP|=SET_BIT1                  
#define set_P0UP_0                      SFRS=1;P0UP|=SET_BIT0                  
                                                 
#define clr_P0UP_7                      SFRS=1;P0UP&=CLR_BIT7                  
#define clr_P0UP_6                      SFRS=1;P0UP&=CLR_BIT6                  
#define clr_P0UP_5                      SFRS=1;P0UP&=CLR_BIT5                  
#define clr_P0UP_4                      SFRS=1;P0UP&=CLR_BIT4                  
#define clr_P0UP_3                      SFRS=1;P0UP&=CLR_BIT3                  
#define clr_P0UP_2                      SFRS=1;P0UP&=CLR_BIT2                  
#define clr_P0UP_1                      SFRS=1;P0UP&=CLR_BIT1                  
#define clr_P0UP_0                      SFRS=1;P0UP&=CLR_BIT0   
// ;---------------------------------------------  
// ; P1UP                                   
// ;---------------------------------------------  
#define set_P1UP_7                      SFRS=1;P1UP|=SET_BIT7                  
#define set_P1UP_6                      SFRS=1;P1UP|=SET_BIT6                  
#define set_P1UP_5                      SFRS=1;P1UP|=SET_BIT5                  
#define set_P1UP_4                      SFRS=1;P1UP|=SET_BIT4                  
#define set_P1UP_3                      SFRS=1;P1UP|=SET_BIT3                  
#define set_P1UP_2                      SFRS=1;P1UP|=SET_BIT2                  
#define set_P1UP_1                      SFRS=1;P1UP|=SET_BIT1                  
#define set_P1UP_0                      SFRS=1;P1UP|=SET_BIT0                  
                                                 
#define clr_P1UP_7                      SFRS=1;P1UP&=CLR_BIT7                  
#define clr_P1UP_6                      SFRS=1;P1UP&=CLR_BIT6                  
#define clr_P1UP_5                      SFRS=1;P1UP&=CLR_BIT5                  
#define clr_P1UP_4                      SFRS=1;P1UP&=CLR_BIT4                  
#define clr_P1UP_3                      SFRS=1;P1UP&=CLR_BIT3                  
#define clr_P1UP_2                      SFRS=1;P1UP&=CLR_BIT2                  
#define clr_P1UP_1                      SFRS=1;P1UP&=CLR_BIT1                  
#define clr_P1UP_0                      SFRS=1;P1UP&=CLR_BIT0
// ;---------------------------------------------  
// ; P2UP                                   
// ;---------------------------------------------  
#define set_P2UP_7                      SFRS=1;P2UP|=SET_BIT7                  
#define set_P2UP_6                      SFRS=1;P2UP|=SET_BIT6                  
#define set_P2UP_5                      SFRS=1;P2UP|=SET_BIT5                  
#define set_P2UP_4                      SFRS=1;P2UP|=SET_BIT4                  
#define set_P2UP_3                      SFRS=1;P2UP|=SET_BIT3                  
#define set_P2UP_2                      SFRS=1;P2UP|=SET_BIT2                  
#define set_P2UP_1                      SFRS=1;P2UP|=SET_BIT1                  
#define set_P2UP_0                      SFRS=1;P2UP|=SET_BIT0                  
                                                 
#define clr_P2UP_7                      SFRS=1;P2UP&=CLR_BIT7                  
#define clr_P2UP_6                      SFRS=1;P2UP&=CLR_BIT6                  
#define clr_P2UP_5                      SFRS=1;P2UP&=CLR_BIT5                  
#define clr_P2UP_4                      SFRS=1;P2UP&=CLR_BIT4                  
#define clr_P2UP_3                      SFRS=1;P2UP&=CLR_BIT3                  
#define clr_P2UP_2                      SFRS=1;P2UP&=CLR_BIT2                  
#define clr_P2UP_1                      SFRS=1;P2UP&=CLR_BIT1                  
#define clr_P2UP_0                      SFRS=1;P2UP&=CLR_BIT0
// ;---------------------------------------------  
// ; P3UP                                   
// ;---------------------------------------------  
#define set_P3UP_7                      SFRS=1;P3UP|=SET_BIT7                  
#define set_P3UP_6                      SFRS=1;P3UP|=SET_BIT6                  
#define set_P3UP_5                      SFRS=1;P3UP|=SET_BIT5                  
#define set_P3UP_4                      SFRS=1;P3UP|=SET_BIT4                  
#define set_P3UP_3                      SFRS=1;P3UP|=SET_BIT3                  
#define set_P3UP_2                      SFRS=1;P3UP|=SET_BIT2                  
#define set_P3UP_1                      SFRS=1;P3UP|=SET_BIT1                  
#define set_P3UP_0                      SFRS=1;P3UP|=SET_BIT0                  
                                                 
#define clr_P3UP_7                      SFRS=1;P3UP&=CLR_BIT7                  
#define clr_P3UP_6                      SFRS=1;P3UP&=CLR_BIT6                  
#define clr_P3UP_5                      SFRS=1;P3UP&=CLR_BIT5                  
#define clr_P3UP_4                      SFRS=1;P3UP&=CLR_BIT4                  
#define clr_P3UP_3                      SFRS=1;P3UP&=CLR_BIT3                  
#define clr_P3UP_2                      SFRS=1;P3UP&=CLR_BIT2                  
#define clr_P3UP_1                      SFRS=1;P3UP&=CLR_BIT1                  
#define clr_P3UP_0                      SFRS=1;P3UP&=CLR_BIT0
// ;---------------------------------------------  
// ; P4UP                                   
// ;---------------------------------------------  
#define set_P4UP_7                      SFRS=1;P4UP|=SET_BIT7                  
#define set_P4UP_6                      SFRS=1;P4UP|=SET_BIT6                  
#define set_P4UP_5                      SFRS=1;P4UP|=SET_BIT5                  
#define set_P4UP_4                      SFRS=1;P4UP|=SET_BIT4                  
#define set_P4UP_3                      SFRS=1;P4UP|=SET_BIT3                  
#define set_P4UP_2                      SFRS=1;P4UP|=SET_BIT2                  
#define set_P4UP_1                      SFRS=1;P4UP|=SET_BIT1                  
#define set_P4UP_0                      SFRS=1;P4UP|=SET_BIT0                  
                                                 
#define clr_P4UP_7                      SFRS=1;P4UP&=CLR_BIT7                  
#define clr_P4UP_6                      SFRS=1;P4UP&=CLR_BIT6                  
#define clr_P4UP_5                      SFRS=1;P4UP&=CLR_BIT5                  
#define clr_P4UP_4                      SFRS=1;P4UP&=CLR_BIT4                  
#define clr_P4UP_3                      SFRS=1;P4UP&=CLR_BIT3                  
#define clr_P4UP_2                      SFRS=1;P4UP&=CLR_BIT2                  
#define clr_P4UP_1                      SFRS=1;P4UP&=CLR_BIT1                  
#define clr_P4UP_0                      SFRS=1;P4UP&=CLR_BIT0
// ;---------------------------------------------  
// ; P5UP                                   
// ;---------------------------------------------  
#define set_P5UP_7                      SFRS=1;P5UP|=SET_BIT7                  
#define set_P5UP_6                      SFRS=1;P5UP|=SET_BIT6                  
#define set_P5UP_5                      SFRS=1;P5UP|=SET_BIT5                  
#define set_P5UP_4                      SFRS=1;P5UP|=SET_BIT4                  
#define set_P5UP_3                      SFRS=1;P5UP|=SET_BIT3                  
#define set_P5UP_2                      SFRS=1;P5UP|=SET_BIT2                  
#define set_P5UP_1                      SFRS=1;P5UP|=SET_BIT1                  
#define set_P5UP_0                      SFRS=1;P5UP|=SET_BIT0                  
                                                 
#define clr_P5UP_7                      SFRS=1;P5UP&=CLR_BIT7                  
#define clr_P5UP_6                      SFRS=1;P5UP&=CLR_BIT6                  
#define clr_P5UP_5                      SFRS=1;P5UP&=CLR_BIT5                  
#define clr_P5UP_4                      SFRS=1;P5UP&=CLR_BIT4                  
#define clr_P5UP_3                      SFRS=1;P5UP&=CLR_BIT3                  
#define clr_P5UP_2                      SFRS=1;P5UP&=CLR_BIT2                  
#define clr_P5UP_1                      SFRS=1;P5UP&=CLR_BIT1                  
#define clr_P5UP_0                      SFRS=1;P5UP&=CLR_BIT0
                                          
// ;---------------------------------------------  
// ; P0DW                                   
// ;---------------------------------------------  
#define set_P0DW_7                      SFRS=1;P0DW|=SET_BIT7                  
#define set_P0DW_6                      SFRS=1;P0DW|=SET_BIT6                  
#define set_P0DW_5                      SFRS=1;P0DW|=SET_BIT5                  
#define set_P0DW_4                      SFRS=1;P0DW|=SET_BIT4                  
#define set_P0DW_3                      SFRS=1;P0DW|=SET_BIT3                  
#define set_P0DW_2                      SFRS=1;P0DW|=SET_BIT2                  
#define set_P0DW_1                      SFRS=1;P0DW|=SET_BIT1                  
#define set_P0DW_0                      SFRS=1;P0DW|=SET_BIT0                  
                                                 
#define clr_P0DW_7                      SFRS=1;P0DW&=CLR_BIT7                  
#define clr_P0DW_6                      SFRS=1;P0DW&=CLR_BIT6                  
#define clr_P0DW_5                      SFRS=1;P0DW&=CLR_BIT5                  
#define clr_P0DW_4                      SFRS=1;P0DW&=CLR_BIT4                  
#define clr_P0DW_3                      SFRS=1;P0DW&=CLR_BIT3                  
#define clr_P0DW_2                      SFRS=1;P0DW&=CLR_BIT2                  
#define clr_P0DW_1                      SFRS=1;P0DW&=CLR_BIT1                  
#define clr_P0DW_0                      SFRS=1;P0DW&=CLR_BIT0   
// ;---------------------------------------------  
// ; P1DW                                   
// ;---------------------------------------------  
#define set_P1DW_7                      SFRS=1;P1DW|=SET_BIT7                  
#define set_P1DW_6                      SFRS=1;P1DW|=SET_BIT6                  
#define set_P1DW_5                      SFRS=1;P1DW|=SET_BIT5                  
#define set_P1DW_4                      SFRS=1;P1DW|=SET_BIT4                  
#define set_P1DW_3                      SFRS=1;P1DW|=SET_BIT3                  
#define set_P1DW_2                      SFRS=1;P1DW|=SET_BIT2                  
#define set_P1DW_1                      SFRS=1;P1DW|=SET_BIT1                  
#define set_P1DW_0                      SFRS=1;P1DW|=SET_BIT0                  
                                                 
#define clr_P1DW_7                      SFRS=1;P1DW&=CLR_BIT7                  
#define clr_P1DW_6                      SFRS=1;P1DW&=CLR_BIT6                  
#define clr_P1DW_5                      SFRS=1;P1DW&=CLR_BIT5                  
#define clr_P1DW_4                      SFRS=1;P1DW&=CLR_BIT4                  
#define clr_P1DW_3                      SFRS=1;P1DW&=CLR_BIT3                  
#define clr_P1DW_2                      SFRS=1;P1DW&=CLR_BIT2                  
#define clr_P1DW_1                      SFRS=1;P1DW&=CLR_BIT1                  
#define clr_P1DW_0                      SFRS=1;P1DW&=CLR_BIT0
// ;---------------------------------------------  
// ; P2DW                                   
// ;---------------------------------------------  
#define set_P2DW_7                      SFRS=1;P2DW|=SET_BIT7                  
#define set_P2DW_6                      SFRS=1;P2DW|=SET_BIT6                  
#define set_P2DW_5                      SFRS=1;P2DW|=SET_BIT5                  
#define set_P2DW_4                      SFRS=1;P2DW|=SET_BIT4                  
#define set_P2DW_3                      SFRS=1;P2DW|=SET_BIT3                  
#define set_P2DW_2                      SFRS=1;P2DW|=SET_BIT2                  
#define set_P2DW_1                      SFRS=1;P2DW|=SET_BIT1                  
#define set_P2DW_0                      SFRS=1;P2DW|=SET_BIT0                  
                                                 
#define clr_P2DW_7                      SFRS=1;P2DW&=CLR_BIT7                  
#define clr_P2DW_6                      SFRS=1;P2DW&=CLR_BIT6                  
#define clr_P2DW_5                      SFRS=1;P2DW&=CLR_BIT5                  
#define clr_P2DW_4                      SFRS=1;P2DW&=CLR_BIT4                  
#define clr_P2DW_3                      SFRS=1;P2DW&=CLR_BIT3                  
#define clr_P2DW_2                      SFRS=1;P2DW&=CLR_BIT2                  
#define clr_P2DW_1                      SFRS=1;P2DW&=CLR_BIT1                  
#define clr_P2DW_0                      SFRS=1;P2DW&=CLR_BIT0
// ;---------------------------------------------  
// ; P3DW                                   
// ;---------------------------------------------  
#define set_P3DW_7                      SFRS=1;P3DW|=SET_BIT7                  
#define set_P3DW_6                      SFRS=1;P3DW|=SET_BIT6                  
#define set_P3DW_5                      SFRS=1;P3DW|=SET_BIT5                  
#define set_P3DW_4                      SFRS=1;P3DW|=SET_BIT4                  
#define set_P3DW_3                      SFRS=1;P3DW|=SET_BIT3                  
#define set_P3DW_2                      SFRS=1;P3DW|=SET_BIT2                  
#define set_P3DW_1                      SFRS=1;P3DW|=SET_BIT1                  
#define set_P3DW_0                      SFRS=1;P3DW|=SET_BIT0                  
                                                 
#define clr_P3DW_7                      SFRS=1;P3DW&=CLR_BIT7                  
#define clr_P3DW_6                      SFRS=1;P3DW&=CLR_BIT6                  
#define clr_P3DW_5                      SFRS=1;P3DW&=CLR_BIT5                  
#define clr_P3DW_4                      SFRS=1;P3DW&=CLR_BIT4                  
#define clr_P3DW_3                      SFRS=1;P3DW&=CLR_BIT3                  
#define clr_P3DW_2                      SFRS=1;P3DW&=CLR_BIT2                  
#define clr_P3DW_1                      SFRS=1;P3DW&=CLR_BIT1                  
#define clr_P3DW_0                      SFRS=1;P3DW&=CLR_BIT0
// ;---------------------------------------------  
// ; P4DW                                   
// ;---------------------------------------------  
#define set_P4DW_7                      SFRS=1;P4DW|=SET_BIT7                  
#define set_P4DW_6                      SFRS=1;P4DW|=SET_BIT6                  
#define set_P4DW_5                      SFRS=1;P4DW|=SET_BIT5                  
#define set_P4DW_4                      SFRS=1;P4DW|=SET_BIT4                  
#define set_P4DW_3                      SFRS=1;P4DW|=SET_BIT3                  
#define set_P4DW_2                      SFRS=1;P4DW|=SET_BIT2                  
#define set_P4DW_1                      SFRS=1;P4DW|=SET_BIT1                  
#define set_P4DW_0                      SFRS=1;P4DW|=SET_BIT0                  
                                                 
#define clr_P4DW_7                      SFRS=1;P4DW&=CLR_BIT7                  
#define clr_P4DW_6                      SFRS=1;P4DW&=CLR_BIT6                  
#define clr_P4DW_5                      SFRS=1;P4DW&=CLR_BIT5                  
#define clr_P4DW_4                      SFRS=1;P4DW&=CLR_BIT4                  
#define clr_P4DW_3                      SFRS=1;P4DW&=CLR_BIT3                  
#define clr_P4DW_2                      SFRS=1;P4DW&=CLR_BIT2                  
#define clr_P4DW_1                      SFRS=1;P4DW&=CLR_BIT1                  
#define clr_P4DW_0                      SFRS=1;P4DW&=CLR_BIT0
// ;---------------------------------------------  
// ; P5DW                                   
// ;---------------------------------------------  
#define set_P5DW_7                      SFRS=1;P5DW|=SET_BIT7                  
#define set_P5DW_6                      SFRS=1;P5DW|=SET_BIT6                  
#define set_P5DW_5                      SFRS=1;P5DW|=SET_BIT5                  
#define set_P5DW_4                      SFRS=1;P5DW|=SET_BIT4                  
#define set_P5DW_3                      SFRS=1;P5DW|=SET_BIT3                  
#define set_P5DW_2                      SFRS=1;P5DW|=SET_BIT2                  
#define set_P5DW_1                      SFRS=1;P5DW|=SET_BIT1                  
#define set_P5DW_0                      SFRS=1;P5DW|=SET_BIT0                  
                                                 
#define clr_P5DW_7                      SFRS=1;P5DW&=CLR_BIT7                  
#define clr_P5DW_6                      SFRS=1;P5DW&=CLR_BIT6                  
#define clr_P5DW_5                      SFRS=1;P5DW&=CLR_BIT5                  
#define clr_P5DW_4                      SFRS=1;P5DW&=CLR_BIT4                  
#define clr_P5DW_3                      SFRS=1;P5DW&=CLR_BIT3                  
#define clr_P5DW_2                      SFRS=1;P5DW&=CLR_BIT2                  
#define clr_P5DW_1                      SFRS=1;P5DW&=CLR_BIT1                  
#define clr_P5DW_0                      SFRS=1;P5DW&=CLR_BIT0
// ;---------------------------------------------  
// ; CWK                                   
// ;---------------------------------------------
/**********************/
/*	PAGE2         */
/**********************/
// ;--------------------------------------------
// ;SC1CR0                                      
// ;--------------------------------------------
#define set_SC1CR0_NSB                  SFRS=2;SC1CR0|=SET_BIT7
#define set_SC1CR0_T                    SFRS=2;SC1CR0|=SET_BIT6
#define set_SC1CR0_RXBGTEN              SFRS=2;SC1CR0|=SET_BIT5
#define set_SC1CR0_CONSEL               SFRS=2;SC1CR0|=SET_BIT4
#define set_SC1CR0_AUTOCEN               SFRS=2;SC1CR0|=SET_BIT3
#define set_SC1CR0_TXOFF                SFRS=2;SC1CR0|=SET_BIT2
#define set_SC1CR0_RXOFF                SFRS=2;SC1CR0|=SET_BIT1
#define set_SC1CR0_SCEN                  SFRS=2;SC1CR0|=SET_BIT1
                                       
#define clr_SC1CR0_NSB		              SFRS=2;SC1CR0&=CLR_BIT7
#define clr_SC1CR0_T		                SFRS=2;SC1CR0&=CLR_BIT6
#define clr_SC1CR0_RXBGTEN	            SFRS=2;SC1CR0&=CLR_BIT5
#define clr_SC1CR0_CONSEL	              SFRS=2;SC1CR0&=CLR_BIT4
#define clr_SC1CR0_AUTOCEN               SFRS=2;SC1CR0&=CLR_BIT3
#define clr_SC1CR0_TXOFF                SFRS=2;SC1CR0&=CLR_BIT2
#define clr_SC1CR0_RXOFF	              SFRS=2;SC1CR0&=CLR_BIT1
#define clr_SC1CR0_SCEN                  SFRS=2;SC1CR0&=CLR_BIT1
// ;--------------------------------------------
// ;SC1CR1                                      
// ;--------------------------------------------
#define set_SC1CR1_OPE		              SFRS=2;SC1CR1|=SET_BIT7
#define set_SC1CR1_PBOFF	              SFRS=2;SC1CR1|=SET_BIT6
#define set_SC1CR1_TXDMAEN	            SFRS=2;SC1CR1|=SET_BIT3
#define set_SC1CR1_RXDMAEN              SFRS=2;SC1CR1|=SET_BIT2
#define set_SC1CR1_CLKKEEP              SFRS=2;SC1CR1|=SET_BIT1
#define set_SC1CR1_UARTEN               SFRS=2;SC1CR1|=SET_BIT0
                                                              
#define clr_SC1CR1_OPE		              SFRS=2;SC1CR1&=CLR_BIT7
#define clr_SC1CR1_PBOFF	              SFRS=2;SC1CR1&=CLR_BIT6
#define clr_SC1CR1_TXDMAEN	            SFRS=2;SC1CR1&=CLR_BIT3
#define clr_SC1CR1_RXDMAEN              SFRS=2;SC1CR1&=CLR_BIT2
#define clr_SC1CR1_CLKKEEP              SFRS=2;SC1CR1&=CLR_BIT1
#define clr_SC1CR1_UARTEN               SFRS=2;SC1CR1&=CLR_BIT0
// ;--------------------------------------------
// ;SC1DR                                      
// ;--------------------------------------------
// ;--------------------------------------------
// ;SC1EGT                                      
// ;--------------------------------------------
// ;--------------------------------------------
// ;SC1ETURD0                                      
// ;--------------------------------------------
// ;--------------------------------------------
// ;SC1ETURD1                                      
// ;--------------------------------------------
// ;-------------------------------------------
// ;SC1IE                                      
// ;-------------------------------------------
#define set_SC1IE_ACERRIEN               SFRS=2;SC1IE|=SET_BIT4
#define set_SC1IE_BGTIEN                 SFRS=2;SC1IE|=SET_BIT3
#define set_SC1IE_TERRIEN                SFRS=2;SC1IE|=SET_BIT2
#define set_SC1IE_TBEIEN                 SFRS=2;SC1IE|=SET_BIT1
#define set_SC1IE_RDAIEN                 SFRS=2;SC1IE|=SET_BIT0
                                                
#define clr_SC1IE_ACERRIEN	             SFRS=2;SC1IE&=CLR_BIT4
#define clr_SC1IE_BGTIEN	               SFRS=2;SC1IE&=CLR_BIT3
#define clr_SC1IE_TERRIEN                SFRS=2;SC1IE&=CLR_BIT2
#define clr_SC1IE_TBEIEN                 SFRS=2;SC1IE&=CLR_BIT1
#define clr_SC1IE_RDAIEN                 SFRS=2;SC1IE&=CLR_BIT0
// ;---------------------------------------------
// ;SC1IS                                     
// ;---------------------------------------------
#define set_SC1IS_LOOP		               SFRS=2;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;SC1IS|=SET_BIT1;EA=BIT_TMP
#define set_SC1IS_SIF		                 SFRS=2;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;SC1IS|=SET_BIT2;EA=BIT_TMP
                                                                       
#define clr_SC1IS_LOOP		               SFRS=2;BIT_TMP=EA;EA=0;TA=0xAA;TA=0x55;SC1IS&=CLR_BIT1;EA=BIT_TMP
// ;---------------------------------------------
// ;SC1TSR                                     
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1PL
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C0L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C1L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C2L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C3L
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C4L                                     
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C5L                                     
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1PH
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C0H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C1H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C2H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C3H
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C4H                                     
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1C5H                                    
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1DTEN
// ;---------------------------------------------
#define set_PWM1DTEN_PDT45EN	          SFRS=2;PWM1DTEN|=SET_BIT2
#define set_PWM1DTEN_PDT23EN	          SFRS=2;PWM1DTEN|=SET_BIT1
#define set_PWM1DTEN_PDT01EN            SFRS=2;PWM1DTEN|=SET_BIT0
                                        
#define clr_PWM1DTEN_PDT45EN            SFRS=2;PWM1DTEN&=CLR_BIT2
#define clr_PWM1DTEN_PDT23EN            SFRS=2;PWM1DTEN&=CLR_BIT1
#define clr_PWM1DTEN_PDT01EN            SFRS=2;PWM1DTEN&=CLR_BIT0
// ;---------------------------------------------
// ; PWM1DTCNT                          
// ;---------------------------------------------
#define set_PWM1DTCNT_PMEN5             SFRS=2;PWM1DTCNT|=SET_BIT4
#define set_PWM1DTCNT_PMEN4             SFRS=2;PWM1DTCNT|=SET_BIT3
#define set_PWM1DTCNT_PMEN3             SFRS=2;PWM1DTCNT|=SET_BIT2
#define set_PWM1DTCNT_PMEN2             SFRS=2;PWM1DTCNT|=SET_BIT1
#define set_PWM1DTCNT_PMEN1             SFRS=2;PWM1DTCNT|=SET_BIT0
                                        
#define clr_PWM1DTCNT_PMEN5             SFRS=2;PWM1DTCNT&=CLR_BIT4
#define clr_PWM1DTCNT_PMEN4             SFRS=2;PWM1DTCNT&=CLR_BIT3
#define clr_PWM1DTCNT_PMEN3             SFRS=2;PWM1DTCNT&=CLR_BIT2
#define clr_PWM1DTCNT_PMEN2             SFRS=2;PWM1DTCNT&=CLR_BIT1
#define clr_PWM1DTCNT_PMEN1             SFRS=2;PWM1DTCNT&=CLR_BIT0
// ;---------------------------------------------
// ; PWM1IOCON
// ;---------------------------------------------
// ;---------------------------------------------
// ; PWM1CON1
// ;---------------------------------------------
#define set_PWM1CON1_GP		              SFRS=2;PWM1CON1|=SET_BIT5
#define set_PWM1CON1_PWMTYP	            SFRS=2;PWM1CON1|=SET_BIT4
#define set_PWM1CON1_FBINEN	            SFRS=2;PWM1CON1|=SET_BIT3
#define set_PWM1CON1_PWMDIV2	          SFRS=2;PWM1CON1|=SET_BIT2
#define set_PWM1CON1_PWMDIV1	          SFRS=2;PWM1CON1|=SET_BIT1
#define set_PWM1CON1_PWMDIV0	          SFRS=2;PWM1CON1|=SET_BIT0
                                        
#define clr_PWM1CON1_GP		              SFRS=2;PWM1CON1&=CLR_BIT5
#define clr_PWM1CON1_PWMTYP	            SFRS=2;PWM1CON1&=CLR_BIT4
#define clr_PWM1CON1_FBINEN	            SFRS=2;PWM1CON1&=CLR_BIT3
#define clr_PWM1CON1_PWMDIV2	          SFRS=2;PWM1CON1&=CLR_BIT2
#define clr_PWM1CON1_PWMDIV1	          SFRS=2;PWM1CON1&=CLR_BIT1
#define clr_PWM1CON1_PWMDIV0	          SFRS=2;PWM1CON1&=CLR_BIT0

// ;---------------------------------------------
// ; PWM1NP
// ;---------------------------------------------
#define set_PWM1NP_PNP5	                SFRS=2;PWM1NP|=SET_BIT5
#define set_PWM1NP_PNP4	                SFRS=2;PWM1NP|=SET_BIT4
#define set_PWM1NP_PNP3	                SFRS=2;PWM1NP|=SET_BIT3
#define set_PWM1NP_PNP2	                SFRS=2;PWM1NP|=SET_BIT2
#define set_PWM1NP_PNP1	                SFRS=2;PWM1NP|=SET_BIT1
#define set_PWM1NP_PNP0	                SFRS=2;PWM1NP|=SET_BIT0
                                               
#define clr_PWM1NP_PNP5	                SFRS=2;PWM1NP&=CLR_BIT5
#define clr_PWM1NP_PNP4	                SFRS=2;PWM1NP&=CLR_BIT4
#define clr_PWM1NP_PNP3	                SFRS=2;PWM1NP&=CLR_BIT3
#define clr_PWM1NP_PNP2	                SFRS=2;PWM1NP&=CLR_BIT2
#define clr_PWM1NP_PNP1	                SFRS=2;PWM1NP&=CLR_BIT1
#define clr_PWM1NP_PNP0	                SFRS=2;PWM1NP&=CLR_BIT0
// ;---------------------------------------------
// ; PWM0FBD                                      
// ;---------------------------------------------
#define set_PWM1FBD_FBINLS              SFRS=2;PWM1FBD|=SET_BIT6
#define set_PWM1FBD_FBD5                SFRS=2;PWM1FBD|=SET_BIT5
#define set_PWM1FBD_FBD4                SFRS=2;PWM1FBD|=SET_BIT4
#define set_PWM1FBD_FBD3                SFRS=2;PWM1FBD|=SET_BIT3
#define set_PWM1FBD_FBD2                SFRS=2;PWM1FBD|=SET_BIT2
#define set_PWM1FBD_FBD1                SFRS=2;PWM1FBD|=SET_BIT1
#define set_PWM1FBD_FBD0                SFRS=2;PWM1FBD|=SET_BIT0
                                        
#define clr_PWM1FBD_FBF                 SFRS=2;PWM1FBD&=CLR_BIT7
#define clr_PWM1FBD_FBINLS              SFRS=2;PWM1FBD&=CLR_BIT6
#define clr_PWM1FBD_FBD5                SFRS=2;PWM1FBD&=CLR_BIT5
#define clr_PWM1FBD_FBD4                SFRS=2;PWM1FBD&=CLR_BIT4
#define clr_PWM1FBD_FBD3                SFRS=2;PWM1FBD&=CLR_BIT3
#define clr_PWM1FBD_FBD2                SFRS=2;PWM1FBD&=CLR_BIT2
#define clr_PWM1FBD_FBD1                SFRS=2;PWM1FBD&=CLR_BIT1
#define clr_PWM1FBD_FBD0                SFRS=2;PWM1FBD&=CLR_BIT0
// ;---------------------------------------------
// ; PWM1MD
// ;---------------------------------------------
#define set_PWM1MD_PMEN5                SFRS=2;PWM1MD|=SET_BIT4
#define set_PWM1MD_PMEN4                SFRS=2;PWM1MD|=SET_BIT3
#define set_PWM1MD_PMEN3                SFRS=2;PWM1MD|=SET_BIT2
#define set_PWM1MD_PMEN2                SFRS=2;PWM1MD|=SET_BIT1
#define set_PWM1MD_PMEN1                SFRS=2;PWM1MD|=SET_BIT0
                                               
#define clr_PWM1MD_PMEN5                SFRS=2;PWM1MD&=CLR_BIT4
#define clr_PWM1MD_PMEN4                SFRS=2;PWM1MD&=CLR_BIT3
#define clr_PWM1MD_PMEN3                SFRS=2;PWM1MD&=CLR_BIT2
#define clr_PWM1MD_PMEN2                SFRS=2;PWM1MD&=CLR_BIT1
#define clr_PWM1MD_PMEN1                SFRS=2;PWM1MD&=CLR_BIT0
// ;---------------------------------------------
// ;PWM1CON0
// ;---------------------------------------------
#define set_PWM1CON0_PWMRUN             SFRS=2;PWM1CON0|=SET_BIT7
#define set_PWM1CON0_LOAD	              SFRS=2;PWM1CON0|=SET_BIT6
#define set_PWM1CON0_PWMF	              SFRS=2;PWM1CON0|=SET_BIT5
#define set_PWM1CON0_CLRPWM	            SFRS=2;PWM1CON0|=SET_BIT4
                                          
#define clr_PWM1CON0_PWMRUN	            SFRS=2;PWM1CON0&=CLR_BIT7
#define clr_PWM1CON0_LOAD	              SFRS=2;PWM1CON0&=CLR_BIT6
#define clr_PWM1CON0_PWMF	              SFRS=2;PWM1CON0&=CLR_BIT5
#define clr_PWM1CON0_CLRPWM	            SFRS=2;PWM1CON0&=CLR_BIT4
// ;---------------------------------------------
// ; PWM1CON1
// ;---------------------------------------------
#define set_PWM1CON1_GP		              SFRS=2;PWM1CON1|=SET_BIT5
#define set_PWM1CON1_PWMTYP             SFRS=2;PWM1CON1|=SET_BIT4
#define set_PWM1CON1_FBINEN	            SFRS=2;PWM1CON1|=SET_BIT3
#define set_PWM1CON1_PWMDIV2	          SFRS=2;PWM1CON1|=SET_BIT2
#define set_PWM1CON1_PWMDIV1	          SFRS=2;PWM1CON1|=SET_BIT1
#define set_PWM1CON1_PWMDIV0	          SFRS=2;PWM1CON1|=SET_BIT0
                                          
#define clr_PWM1CON1_GP		              SFRS=2;PWM1CON1&=CLR_BIT5
#define clr_PWM1CON1_PWMTYP	            SFRS=2;PWM1CON1&=CLR_BIT4
#define clr_PWM1CON1_FBINEN	            SFRS=2;PWM1CON1&=CLR_BIT3
#define clr_PWM1CON1_PWMDIV2	          SFRS=2;PWM1CON1&=CLR_BIT2
#define clr_PWM1CON1_PWMDIV1	          SFRS=2;PWM1CON1&=CLR_BIT1
#define clr_PWM1CON1_PWMDIV0	          SFRS=2;PWM1CON1&=CLR_BIT0
// ;---------------------------------------------
// ; PWM1NP                     
// ;---------------------------------------------
#define set_PWM1NP_PNP5	                SFRS=2;PWM1NP|=SET_BIT5
#define set_PWM1NP_PNP4	                SFRS=2;PWM1NP|=SET_BIT4
#define set_PWM1NP_PNP3	                SFRS=2;PWM1NP|=SET_BIT3
#define set_PWM1NP_PNP2	                SFRS=2;PWM1NP|=SET_BIT2
#define set_PWM1NP_PNP1	                SFRS=2;PWM1NP|=SET_BIT1
#define set_PWM1NP_PNP0	                SFRS=2;PWM1NP|=SET_BIT0
                                          
#define clr_PWM1NP_PNP5	                SFRS=2;PWM1NP&=CLR_BIT5
#define clr_PWM1NP_PNP4	                SFRS=2;PWM1NP&=CLR_BIT4
#define clr_PWM1NP_PNP3	                SFRS=2;PWM1NP&=CLR_BIT3
#define clr_PWM1NP_PNP2	                SFRS=2;PWM1NP&=CLR_BIT2
#define clr_PWM1NP_PNP1	                SFRS=2;PWM1NP&=CLR_BIT1
#define clr_PWM1NP_PNP0	                SFRS=2;PWM1NP&=CLR_BIT0
// ;---------------------------------------------
// ; PWM1FBD                                      
// ;---------------------------------------------
#define set_PWM1FBD_FBINLS              SFRS=2;PWM1FBD|=SET_BIT6
#define set_PWM1FBD_FBD5                SFRS=2;PWM1FBD|=SET_BIT5
#define set_PWM1FBD_FBD4                SFRS=2;PWM1FBD|=SET_BIT4
#define set_PWM1FBD_FBD3                SFRS=2;PWM1FBD|=SET_BIT3
#define set_PWM1FBD_FBD2                SFRS=2;PWM1FBD|=SET_BIT2
#define set_PWM1FBD_FBD1                SFRS=2;PWM1FBD|=SET_BIT1
#define set_PWM1FBD_FBD0                SFRS=2;PWM1FBD|=SET_BIT0

#define clr_PWM1FBD_FBF                 SFRS=2;PWM1FBD&=CLR_BIT7
#define clr_PWM1FBD_FBINLS              SFRS=2;PWM1FBD&=CLR_BIT6
#define clr_PWM1FBD_FBD5                SFRS=2;PWM1FBD&=CLR_BIT5
#define clr_PWM1FBD_FBD4                SFRS=2;PWM1FBD&=CLR_BIT4
#define clr_PWM1FBD_FBD3                SFRS=2;PWM1FBD&=CLR_BIT3
#define clr_PWM1FBD_FBD2                SFRS=2;PWM1FBD&=CLR_BIT2
#define clr_PWM1FBD_FBD1                SFRS=2;PWM1FBD&=CLR_BIT1
#define clr_PWM1FBD_FBD0                SFRS=2;PWM1FBD&=CLR_BIT0
// ;---------------------------------------------
// ; PWM1INTC                                      
// ;---------------------------------------------
#define set_PWM1INTC_INTTYP1             SFRS=2;PWM1INTC|=SET_BIT5
#define set_PWM1INTC_INTTYP0             SFRS=2;PWM1INTC|=SET_BIT4
#define set_PWM1INTC_INTSEL2             SFRS=2;PWM1INTC|=SET_BIT2
#define set_PWM1INTC_INTSEL1             SFRS=2;PWM1INTC|=SET_BIT1
#define set_PWM1INTC_INTSEL0             SFRS=2;PWM1INTC|=SET_BIT0
                                    
#define clr_PWM1INTC_INTTYP1             SFRS=2;PWM1INTC&=CLR_BIT5
#define clr_PWM1INTC_INTTYP0             SFRS=2;PWM1INTC&=CLR_BIT4
#define clr_PWM1INTC_INTSEL2             SFRS=2;PWM1INTC&=CLR_BIT2
#define clr_PWM1INTC_INTSEL1             SFRS=2;PWM1INTC&=CLR_BIT1
#define clr_PWM1INTC_INTSEL0             SFRS=2;PWM1INTC&=CLR_BIT0
// ;---------------------------------------------
// ; DMA2TSR                                      
// ;---------------------------------------------
#define clr_DMA2TSR_ACT		              SFRS=2;DMA2TSR&=CLR_BIT2
#define clr_DMA2TSR_HDONE	              SFRS=2;DMA2TSR&=CLR_BIT1
#define clr_DMA2TSR_FDONE	              SFRS=2;DMA2TSR&=CLR_BIT0
// ;---------------------------------------------
// ; DMA2BAH
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA2CR
// ;---------------------------------------------
#define set_DMA2CR_HIE	                SFRS=2;DMA2CR|=SET_BIT3
#define set_DMA2CR_FIE	                SFRS=2;DMA2CR|=SET_BIT2
#define set_DMA2CR_RUN	                SFRS=2;DMA2CR|=SET_BIT1
#define set_DMA2CR_EN	                  SFRS=2;DMA2CR|=SET_BIT0
                        
#define clr_DMA2CR_HIE	                SFRS=2;DMA2CR&=CLR_BIT3
#define clr_DMA2CR_FIE	                SFRS=2;DMA2CR&=CLR_BIT2
#define clr_DMA2CR_RUN	                SFRS=2;DMA2CR&=CLR_BIT1
#define clr_DMA2CR_EN	                  SFRS=2;DMA2CR&=CLR_BIT0
// ;---------------------------------------------
// ; DMA2MA
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA2CNT
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA2CCNT
// ;---------------------------------------------
// ;---------------------------------------------
// ; MTM2DA
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA3TSR                                      
// ;---------------------------------------------
#define clr_DMA3TSR_ACT		              SFRS=2;DMA3TSR&=CLR_BIT2
#define clr_DMA3TSR_HDONE	              SFRS=2;DMA3TSR&=CLR_BIT1
#define clr_DMA3TSR_FDONE	              SFRS=2;DMA3TSR&=CLR_BIT0
// ;---------------------------------------------
// ; DMA3BAH
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA3CR
// ;---------------------------------------------
#define set_DMA3CR_HIE                  SFRS=2;DMA3CR|=SET_BIT3
#define set_DMA3CR_FIE                  SFRS=2;DMA3CR|=SET_BIT2
#define set_DMA3CR_RUN                  SFRS=2;DMA3CR|=SET_BIT1
#define set_DMA3CR_EN                   SFRS=2;DMA3CR|=SET_BIT0

#define clr_DMA3CR_HIE                  SFRS=2;DMA3CR&=CLR_BIT3
#define clr_DMA3CR_FIE                  SFRS=2;DMA3CR&=CLR_BIT2
#define clr_DMA3CR_RUN                  SFRS=2;DMA3CR&=CLR_BIT1
#define clr_DMA3CR_EN                   SFRS=2;DMA3CR&=CLR_BIT0
// ;---------------------------------------------
// ; DMA3MA
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA3CNT
// ;---------------------------------------------
// ;---------------------------------------------
// ; DMA3CCNT
// ;---------------------------------------------
// ;---------------------------------------------
// ; MTM3DA
// ;---------------------------------------------

