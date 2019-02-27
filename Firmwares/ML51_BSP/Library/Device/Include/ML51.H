//ML51.H
//Header file for Nuvoton ML51
//
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
//1C8H  T2CON    AUXR1     RCMP2L    RCMP2H    PWM0C4L  PWM0C5L    AINDIDS   -
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


/*  BYTE Registers  */
/*  SFR page 0      */
sfr P0          = 0x80; // All page
sfr SP          = 0x81; // All page
sfr DPL         = 0x82; // All page
sfr DPH         = 0x83; // All page
sfr RCTRIM0     = 0x84; // SFR page 0 //TA Protection
sfr RCTRIM1     = 0x85; // SFR page 0 //TA Protection
sfr RWK         = 0x86; // SFR page 0
sfr PCON        = 0x87; // All page

sfr TCON        = 0x88; // All page
sfr TMOD        = 0x89; // SFR page 0
sfr TL0         = 0x8A; // SFR page 0
sfr TL1         = 0x8B; // SFR page 0
sfr TH0         = 0x8C; // SFR page 0
sfr TH1         = 0x8D; // SFR page 0
sfr CKCON       = 0x8E; // SFR page 0
sfr WKCON       = 0x8F; // SFR page 0

sfr P1          = 0x90; // All page
sfr SFRS        = 0x91; // All page
sfr DMA0CR      = 0x92; // SFR page 0
sfr DMA0MA      = 0x93; // SFR page 0
sfr DMA0CNT     = 0x94; // SFR page 0
sfr DMA0CCNT    = 0x95; // SFR page 0
sfr CKSWT       = 0x96; // SFR page 0//TA Protection
sfr CKEN        = 0x97; // SFR page 0//TA Protection

sfr SCON        = 0x98; // All page
sfr SBUF        = 0x99; // SFR page 0
sfr SBUF1       = 0x9A; // SFR page 0
sfr EIE0        = 0x9B; // SFR page 0
sfr EIE1        = 0x9C; // SFR page 0
sfr RSR         = 0x9D; // SFR page 0
sfr IAPTC       = 0x9E; // SFR page 0//TA Protection
sfr CHPCON      = 0x9F; // All page  //TA Protection

sfr P2          = 0xA0; // All page
sfr ADCCON0     = 0xA1; // SFR page 0
sfr AUXR0       = 0xA2; // SFR page 0
sfr BODCON0     = 0xA3; // SFR page 0//TA Protection
sfr IAPTRG      = 0xA4; // SFR page 0//TA Protection
sfr IAPUEN      = 0xA5; // SFR page 0//TA Protection
sfr IAPAL       = 0xA6; // SFR page 0
sfr IAPAH       = 0xA7; // SFR page 0

sfr IE          = 0xA8; // All page
sfr SADDR       = 0xA9; // SFR page 0
sfr WDCON       = 0xAA; // SFR page 0//TA Protection
sfr BODCON1     = 0xAB; // SFR page 0//TA Protection
sfr EIP2        = 0xAC; // SFR page 0
sfr EIPH2       = 0xAD; // SFR page 0
sfr IAPFD       = 0xAE; // SFR page 0
sfr IAPCN       = 0xAF; // SFR page 0

sfr P3          = 0xB0; // All page
sfr P5          = 0xB1; // SFR page 0
sfr I2C1ADDR0   = 0xB2; // SFR page 0
sfr I2C1DAT     = 0xB3; // SFR page 0
sfr I2C1STAT    = 0xB4; // SFR page 0
sfr I2C1CLK     = 0xB5; // SFR page 0
sfr I2C1TOC     = 0xB6; // SFR page 0
sfr IPH         = 0xB7; // SFR page 0

sfr IP          = 0xB8; // All page
sfr SADEN       = 0xB9; // SFR page 0
sfr SADEN1      = 0xBA; // SFR page 0
sfr SADDR1      = 0xBB; // SFR page 0
sfr I2C0DAT     = 0xBC; // SFR page 0
sfr I2C0STAT    = 0xBD; // SFR page 0
sfr I2C0CLK     = 0xBE; // SFR page 0
sfr I2C0TOC     = 0xBF; // SFR page 0

sfr I2C0CON     = 0xC0; // All page
sfr I2C0ADDR0   = 0xC1; // SFR page 0
sfr ADCRL       = 0xC2; // SFR page 0
sfr ADCRH       = 0xC3; // SFR page 0
sfr T3CON       = 0xC4; // SFR page 0
sfr RL3         = 0xC5; // SFR page 0
sfr RH3         = 0xC6; // SFR page 0
sfr TA          = 0xC7; // All page

sfr T2CON       = 0xC8; // All page
sfr T2MOD       = 0xC9; // SFR page 0
sfr PIF         = 0xCA; // SFR page 0
sfr ADCBAL      = 0xCB; // SFR page 0
sfr TL2         = 0xCC; // SFR page 0
sfr TH2         = 0xCD; // SFR page 0
sfr ADCMPL      = 0xCE; // SFR page 0
sfr ADCMPH      = 0xCF; // SFR page 0

sfr PSW         = 0xD0; // All page
sfr PWM0CON0    = 0xD1; // SFR page 0
sfr ACMPCR0     = 0xD2; // SFR page 0
sfr ACMPCR1     = 0xD3; // SFR page 0
sfr ACMPSR      = 0xD4; // SFR page 0
sfr ACMPVREF    = 0xD5; // SFR page 0
sfr SC0CR0      = 0xD6; // SFR page 0
sfr SC0CR1      = 0xD7; // SFR page 0

sfr P4          = 0xD8; // All page
sfr SC0DR       = 0xD9; // SFR page 0
sfr SC0EGT      = 0xDA; // SFR page 0
sfr SC0ETURD0   = 0xDB; // SFR page 0
sfr SC0ETURD1   = 0xDC; // SFR page 0
sfr SC0IE       = 0xDD; // SFR page 0
sfr SC0IS       = 0xDE; // SFR page 0
sfr SC0TSR      = 0xDF; // SFR page 0

sfr ACC         = 0xE0; // All page
sfr ADCCON1     = 0xE1; // SFR page 0
sfr ADCCON2     = 0xE2; // SFR page 0
sfr ADCDLY      = 0xE3; // SFR page 0
sfr ADCBAH      = 0xE4; // SFR page 0
sfr ADCSN       = 0xE5; // SFR page 0
sfr ADCCN       = 0xE6; // SFR page 0
sfr ADCSR       = 0xE7; // SFR page 0

sfr I2C1CON     = 0xE8; // All page
sfr DMA0TSR     = 0xE9; // SFR page 0
sfr MTM0DA      = 0xEA; // SFR page 0
sfr DMA1CR      = 0xEB; // SFR page 0
sfr DMA1MA      = 0xEC; // SFR page 0
sfr DMA1CNT     = 0xED; // SFR page 0
sfr DMA1CCNT    = 0xEE; // SFR page 0
sfr EIP0        = 0xEF; // SFR page 0

sfr B           = 0xF0; // All page
sfr DMA1TSR     = 0xF1; // SFR page 0
sfr MTM1DA      = 0xF2; // SFR page 0
sfr SPI0CR0     = 0xF3; // SFR page 0
sfr SPI0SR      = 0xF4; // SFR page 0
sfr SPI0DR      = 0xF5; // SFR page 0
sfr DMA0BAH     = 0xF6; // SFR page 0
sfr EIPH0       = 0xF7; // SFR page 0

sfr S1CON       = 0xF8; // All page
sfr SPI1CR0     = 0xF9; // SFR page 0
sfr SPI1CR1     = 0xFA; // SFR page 0
sfr SPI1SR      = 0xFB; // SFR page 0
sfr SPI1DR      = 0xFC; // SFR page 0
sfr DMA1BAH     = 0xFD; // SFR page 0
sfr EIP1        = 0xFE; // SFR page 0
sfr EIPH1       = 0xFF; // SFR page 0

/*  SFR page 1      */
sfr LIRCTRIM    = 0x84; //TA Protection
sfr XLTCON      = 0x85;
sfr CWK         = 0x86;

sfr P0DW        = 0x8A; // SFR page 1
sfr P1DW        = 0x8B; // SFR page 1
sfr P2DW        = 0x8C; // SFR page 1
sfr P3DW        = 0x8D; // SFR page 1
sfr P4DW        = 0x8E; // SFR page 1
sfr P5DW        = 0x8F; // SFR page 1

sfr P0UP        = 0x92; // SFR page 1
sfr P1UP        = 0x93; // SFR page 1
sfr P2UP        = 0x94; // SFR page 1
sfr P3UP        = 0x95; // SFR page 1
sfr P4UP        = 0x96; // SFR page 1
sfr P5UP        = 0x97; // SFR page 1

sfr P0S         = 0x99; // SFR page 1
sfr P0SR        = 0x9A; // SFR page 1
sfr P1S         = 0x9B; // SFR page 1
sfr P1SR        = 0x9C; // SFR page 1
sfr P2S         = 0x9D; // SFR page 1
sfr P2SR        = 0x9E; // SFR page 1

sfr PIPS0       = 0xA1; // SFR page 1
sfr PIPS1       = 0xA2; // SFR page 1
sfr PIPS2       = 0xA3; // SFR page 1
sfr PIPS3       = 0xA4; // SFR page 1
sfr PIPS4       = 0xA5; // SFR page 1
sfr PIPS5       = 0xA6; // SFR page 1
sfr PIPS6       = 0xA7; // SFR page 1

sfr VRFCON      = 0xA9; // SFR page 1
sfr VRFTRIM     = 0xAA; // SFR page 1//TA Protection
sfr ACMPCR2     = 0xAB; // SFR page 1
sfr P3S         = 0xAC; // SFR page 1
sfr P3SR        = 0xAD; // SFR page 1
sfr P5SR        = 0xAE; // SFR page 1
sfr PIPS7       = 0xAF; // SFR page 1

sfr P0M1        = 0xB1; // SFR page 1
sfr P0M2        = 0xB2; // SFR page 1
sfr P1M1        = 0xB3; // SFR page 1
sfr P1M2        = 0xB4; // SFR page 1
sfr P2M1        = 0xB5; // SFR page 1
sfr P2M2        = 0xB6; // SFR page 1
sfr PWM0INTC    = 0xB7; // SFR page 1

sfr P4M1        = 0xB9; // SFR page 1
sfr P4M2        = 0xBA; // SFR page 1
sfr P4S         = 0xBB; // SFR page 1
sfr P4SR        = 0xBC; // SFR page 1
sfr P5M1        = 0xBD; // SFR page 1
sfr P5M2        = 0xBE; // SFR page 1
sfr P5S         = 0xBF; // SFR page 1

sfr CKDIV       = 0xC1; // SFR page 1
sfr P3M1        = 0xC2; // SFR page 1
sfr P3M2        = 0xC3; // SFR page 1
sfr PWM0C4H     = 0xC4; // SFR page 1
sfr PWM0C5H     = 0xC5; // SFR page 1
sfr PORDIS      = 0xC6; // SFR page 1//TA Protection

sfr AUXR1       = 0xC9; // SFR page 1
sfr RCMP2L      = 0xCA; // SFR page 1
sfr RCMP2H      = 0xCB; // SFR page 1
sfr PWM0C4L     = 0xCC; // SFR page 1
sfr PWM0C5L     = 0xCD; // SFR page 1
sfr AINDIDS     = 0xCE; // SFR page 1


sfr PWM0PH      = 0xD1; // SFR page 1
sfr PWM0C0H     = 0xD2; // SFR page 1
sfr PWM0C1H     = 0xD3; // SFR page 1
sfr PWM0C2H     = 0xD4; // SFR page 1
sfr PWM0C3H     = 0xD5; // SFR page 1
sfr PWM0NP      = 0xD6; // SFR page 1
sfr PWM0FBD     = 0xD7; // SFR page 1

sfr PWM0PL      = 0xD9; // SFR page 1
sfr PWM0C0L     = 0xDA; // SFR page 1
sfr PWM0C1L     = 0xDB; // SFR page 1
sfr PWM0C2L     = 0xDC; // SFR page 1
sfr PWM0C3L     = 0xDD; // SFR page 1
sfr PIOCON0     = 0xDE; // SFR page 1
sfr PWM0CON1    = 0xDF; // SFR page 1

sfr CAPCON0     = 0xE1; // SFR page 1
sfr CAPCON1     = 0xE2; // SFR page 1
sfr CAPCON2     = 0xE3; // SFR page 1
sfr C0L         = 0xE4; // SFR page 1
sfr C0H         = 0xE5; // SFR page 1
sfr C1L         = 0xE6; // SFR page 1
sfr C1H         = 0xE7; // SFR page 1

sfr PICON       = 0xE9; // SFR page 1
sfr PINEN       = 0xEA; // SFR page 1
sfr PIPEN       = 0xEB; // SFR page 1
sfr BCKCON      = 0xEC; // SFR page 1
sfr C2L         = 0xED; // SFR page 1
sfr C2H         = 0xEE; // SFR page 1
sfr LDOTRIM     = 0xEF; // SFR page 1 //TA Protection

sfr CAPCON3     = 0xF1; // SFR page 1
sfr CAPCON4     = 0xF2; // SFR page 1
sfr SPI0CR1     = 0xF3; // SFR page 1
sfr AUXR2       = 0xF4; // SFR page 1
sfr LDOOEN      = 0xF5; // SFR page 1
sfr ROMMAP11    = 0xF6; // SFR page 1 //Don't release & TA Protection


sfr PWM0DTEN    = 0xF9; // SFR page 1
sfr PWM0DTCNT   = 0xFA; // SFR page 1
sfr PWM0MEN     = 0xFB; // SFR page 1
sfr PWM0MD      = 0xFC; // SFR page 1
sfr LVRFLTEN    = 0xFD; // SFR page 1 //TA Protection
sfr RCNT        = 0xFE; // SFR page 1 //Don't release & TA Protection
sfr LVRDIS      = 0xFF; // SFR page 1 //TA Protection

/*  SFR page 2      */

sfr I2C0ADDR1   = 0xA1; // SFR page 2
sfr I2C0ADDR2   = 0xA2; // SFR page 2
sfr I2C0ADDR3   = 0xA3; // SFR page 2
sfr I2C1ADDR1   = 0xA4; // SFR page 2
sfr I2C1ADDR2   = 0xA5; // SFR page 2
sfr I2C1ADDR3   = 0xA6; // SFR page 2

sfr DMA3TSR     = 0xA9; // SFR page 2
sfr DMA3BAH     = 0xAA; // SFR page 2
sfr DMA3CR      = 0xAB; // SFR page 2
sfr DMA3MA      = 0xAC; // SFR page 2
sfr DMA3CNT     = 0xAD; // SFR page 2
sfr DMA3CCNT    = 0xAE; // SFR page 2
sfr MTM3DA      = 0xAF; // SFR page 2

sfr DMA2TSR     = 0xB1; // SFR page 2
sfr DMA2BAH     = 0xB2; // SFR page 2
sfr DMA2CR      = 0xB3; // SFR page 2
sfr DMA2MA      = 0xB4; // SFR page 2
sfr DMA2CNT     = 0xB5; // SFR page 2
sfr DMA2CCNT    = 0xB6; // SFR page 2
sfr MTM2DA      = 0xB7; // SFR page 2

sfr PWM1IOCON   = 0xB9; // SFR page 2
sfr PWM1NP      = 0xBA; // SFR page 2
sfr PWM1FBD     = 0xBB; // SFR page 2
sfr PWM1INTC    = 0xBC; // SFR page 2

sfr PWM1DTEN    = 0xC1; // SFR page 2
sfr PWM1DTCNT   = 0xC2; // SFR page 2
sfr PWM1MEN     = 0xC3; // SFR page 2
sfr PWM1MD      = 0xC4; // SFR page 2
sfr PWM1CON0    = 0xC5; // SFR page 2
sfr PWM1CON1    = 0xC6; // SFR page 2

sfr PWM1PH      = 0xC9; // SFR page 2
sfr PWM1C0H     = 0xCA; // SFR page 2
sfr PWM1C1H     = 0xCB; // SFR page 2
sfr PWM1C2H     = 0xCC; // SFR page 2
sfr PWM1C3H     = 0xCD; // SFR page 2
sfr PWM1C4H     = 0xCE; // SFR page 2
sfr PWM1C5H     = 0xCF; // SFR page 2

sfr PWM1PL      = 0xD1; // SFR page 2
sfr PWM1C0L     = 0xD2; // SFR page 2
sfr PWM1C1L     = 0xD3; // SFR page 2
sfr PWM1C2L     = 0xD4; // SFR page 2
sfr PWM1C3L     = 0xD5; // SFR page 2
sfr PWM1C4L     = 0xD6; // SFR page 2
sfr PWM1C5L     = 0xD7; // SFR page 2

sfr SC1DR       = 0xD9; // SFR page 2
sfr SC1EGT      = 0xDA; // SFR page 2
sfr SC1ETURD0   = 0xDB; // SFR page 2
sfr SC1ETURD1   = 0xDC; // SFR page 2
sfr SC1IE       = 0xDD; // SFR page 2
sfr SC1IS       = 0xDE; // SFR page 2
sfr SC1TSR      = 0xDF; // SFR page 2

sfr P5MF32      = 0xE1; // SFR page 2
sfr P5MF54      = 0xE2; // SFR page 2
sfr P5MF76      = 0xE3; // SFR page 2
sfr ADCCALI     = 0xE5; // SFR page 2
sfr SC1CR0      = 0xE6; // SFR page 2
sfr SC1CR1      = 0xE7; // SFR page 2

sfr P3MF54      = 0xE9; // SFR page 2
sfr P3MF76      = 0xEA; // SFR page 2
sfr P4MF10      = 0xEB; // SFR page 2
sfr P4MF32      = 0xEC; // SFR page 2
sfr P4MF54      = 0xED; // SFR page 2
sfr P4MF76      = 0xEE; // SFR page 2
sfr P5MF10      = 0xEF; // SFR page 2

sfr P1MF76      = 0xF1; // SFR page 2
sfr P2MF10      = 0xF2; // SFR page 2
sfr P2MF32      = 0xF3; // SFR page 2
sfr P2MF54      = 0xF4; // SFR page 2
sfr P2MF76      = 0xF5; // SFR page 2
sfr P3MF10      = 0xF6; // SFR page 2
sfr P3MF32      = 0xF7; // SFR page 2

sfr P0MF10      = 0xF9; // SFR page 2
sfr P0MF32      = 0xFA; // SFR page 2
sfr P0MF54      = 0xFB; // SFR page 2
sfr P0MF76      = 0xFC; // SFR page 2
sfr P1MF10      = 0xFD; // SFR page 2
sfr P1MF32      = 0xFE; // SFR page 2
sfr P1MF54      = 0xFF; // SFR page 2

/*  BIT Registers  */
/*  P0  */
sbit P07        = P0^7;
sbit P06        = P0^6;
sbit P05        = P0^5;
sbit P04        = P0^4;
sbit P03        = P0^3;
sbit P02        = P0^2;
sbit P01        = P0^1;
sbit P00        = P0^0;

/*  TCON  */
sbit TF1        = TCON^7;
sbit TR1        = TCON^6;
sbit TF0        = TCON^5;
sbit TR0        = TCON^4;
sbit IE1        = TCON^3;
sbit IT1        = TCON^2;
sbit IE0        = TCON^1;
sbit IT0        = TCON^0;

/*  P1  */
sbit P17        = P1^7;
sbit P16        = P1^6;
sbit P15        = P1^5;
sbit P14        = P1^4;
sbit P13        = P1^3;
sbit P12        = P1^2;
sbit P11        = P1^1;
sbit P10        = P1^0;

/*  SCON  */
sbit SM0        = SCON^7;
sbit FE         = SCON^7;
sbit SM1        = SCON^6;
sbit SM2        = SCON^5;
sbit REN        = SCON^4;
sbit TB8        = SCON^3;
sbit RB8        = SCON^2;
sbit TI         = SCON^1;
sbit RI         = SCON^0;

/*  P20 */
sbit P27        = P2^7;
sbit P26        = P2^6;
sbit P25        = P2^5;
sbit P24        = P2^4;
sbit P23        = P2^3;
sbit P22        = P2^2;
sbit P21        = P2^1;
sbit P20        = P2^0;

/*  IE  */
sbit EA         = IE^7;
sbit EADC       = IE^6;
sbit EBOD       = IE^5;
sbit ES         = IE^4;
sbit ET1        = IE^3;
sbit EX1        = IE^2;
sbit ET0        = IE^1;
sbit EX0        = IE^0;

/*  P3  */
sbit P37        = P3^7;
sbit P36        = P3^6;
sbit P35        = P3^5;
sbit P34        = P3^4;
sbit P33        = P3^3;
sbit P32        = P3^2;
sbit P31        = P3^1;
sbit P30        = P3^0; 

/*  IP  */
sbit PADC       = IP^6;
sbit PBOD       = IP^5;
sbit PS         = IP^4;
sbit PT1        = IP^3;
sbit PX1        = IP^2;
sbit PT0        = IP^1;
sbit PX0        = IP^0;

/*  I2C0CON  */
sbit I2C0EN     = I2C0CON^6;
sbit STA0       = I2C0CON^5;
sbit STO0       = I2C0CON^4;
sbit SI0        = I2C0CON^3;
sbit AA0        = I2C0CON^2;

/*  T2CON  */
sbit TF2        = T2CON^7;
sbit TR2        = T2CON^2;
sbit CM_RL2      = T2CON^0;

/*  PSW */
sbit CY         = PSW^7;
sbit AC         = PSW^6;
sbit F0         = PSW^5;
sbit RS1        = PSW^4;
sbit RS0        = PSW^3;
sbit OV         = PSW^2;
sbit P          = PSW^0;

/*  P4  */
sbit P47        = P4^7;
sbit P46        = P4^6;
sbit P45        = P4^5;
sbit P44        = P4^4;
sbit P43        = P4^3;
sbit P42        = P4^2;
sbit P41        = P4^1;
sbit P40        = P4^0;

/*  I2C1CON  */
sbit I2C1EN      = I2C1CON^6;
sbit STA1        = I2C1CON^5;
sbit STO1        = I2C1CON^4;
sbit SI1         = I2C1CON^3;
sbit AA1         = I2C1CON^2;

/*  S1CON  */
sbit SM0_1      = S1CON^7;
sbit FE_1       = S1CON^7;
sbit SM1_1      = S1CON^6;
sbit SM2_1      = S1CON^5;
sbit REN_1      = S1CON^4;
sbit TB8_1      = S1CON^3;
sbit RB8_1      = S1CON^2;
sbit TI_1       = S1CON^1;
sbit RI_1       = S1CON^0;


// /*  Other Pin Name  */
sbit AIN0       = P2^5;
sbit AIN1       = P2^4;
sbit AIN2       = P2^3;
sbit AIN3       = P2^2;
sbit AIN4       = P2^1;
sbit AIN5       = P2^0;
sbit AIN6       = P3^1;
sbit AIN7       = P3^2;

sbit ACMP_P0    = P2^5;
sbit ACMP_P1    = P2^3;
sbit ACMP_P2    = P2^1;
sbit ACMP_P3    = P3^0;

sbit ACMP0_N0   = P2^4;
sbit ACMP0_N1   = P2^0;
sbit ACMP1_N0   = P2^2;
sbit ACMP1_N1   = P3^1;


/******************************************************************************/
/*                         Keil C51 include files                            */
/******************************************************************************/
#include <intrins.h>
#include <stdio.h>
#include <absacc.h>
#include <string.h>

/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "SFR_Macro.h"
#include "Function_Define_ML51.h"
#include "sys.h"
#include "isp_uart0.h"
