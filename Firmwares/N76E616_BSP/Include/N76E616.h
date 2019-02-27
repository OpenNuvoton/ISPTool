/*--------------------------------------------------------------------------
N76E616.H

Header file for Nuvoton N76E616
--------------------------------------------------------------------------*/
/*  BYTE Registers  */
sfr P0          = 0x80;        
sfr SP          = 0x81;
sfr DPL         = 0x82;
sfr DPH         = 0x83;
sfr RWK         = 0x86;
sfr PCON        = 0x87;

sfr TCON        = 0x88;
sfr TMOD        = 0x89;
sfr TL0         = 0x8A;
sfr TL1         = 0x8B;
sfr TH0         = 0x8C;
sfr TH1         = 0x8D;
sfr CKCON       = 0x8E;
sfr WKCON       = 0x8F;

sfr P1          = 0x90;
sfr CKDIV       = 0x95;
sfr CKSWT       = 0x96;
sfr CKEN        = 0x97;

sfr SCON        = 0x98;
sfr SBUF        = 0x99;
sfr SBUF_1      = 0x9A;
sfr EIE         = 0x9B;
sfr EIE1        = 0x9C;
sfr CHPCON      = 0x9F;

sfr P2          = 0xA0;
sfr AUXR1       = 0xA2;
sfr BODCON0     = 0xA3;
sfr IAPTRG      = 0xA4;
sfr IAPUEN      = 0xA5;
sfr IAPAL       = 0xA6;
sfr IAPAH       = 0xA7;

sfr IE          = 0xA8;
sfr SADDR       = 0xA9;
sfr WDCON       = 0xAA;
sfr BODCON1     = 0xAB; 
sfr P3M1        = 0xAC;
sfr P3S         = 0xAC;
sfr P3M2        = 0xAD;
sfr IAPFD       = 0xAE;
sfr IAPCN       = 0xAF;

sfr P3          = 0xB0;
sfr P0M1        = 0xB1;
sfr P0S         = 0xB1;
sfr P0M2        = 0xB2;
sfr P1M1        = 0xB3;
sfr P1S         = 0xB3;
sfr P1M2        = 0xB4;
sfr P2M1        = 0xB5;
sfr P2S         = 0xB5;
sfr P2M2        = 0xB6;
sfr IPH         = 0xB7;

sfr IP          = 0xB8;
sfr SADEN       = 0xB9;
sfr SADEN_1     = 0xBA;
sfr SADDR_1     = 0xBB;
sfr I2DAT       = 0xBC;
sfr I2STAT      = 0xBD;
sfr I2CLK       = 0xBE;
sfr I2TOC       = 0xBF;

sfr I2CON       = 0xC0;
sfr I2ADDR      = 0xC1;
sfr ADCRL       = 0xC2;
sfr ADCRH       = 0xC3;
sfr T3CON       = 0xC4;
sfr R3L         = 0xC5;
sfr R3H         = 0xC6;
sfr TA          = 0xC7;

sfr T2CON       = 0xC8;
sfr T2MOD0      = 0xC9;
sfr T2MOD1      = 0xCA;
sfr T2OE        = 0xCB;
sfr R2AL        = 0xCC;
sfr R2AH        = 0xCD;
sfr R2BL        = 0xCE;
sfr R2BH        = 0xCF;

sfr PSW         = 0xD0;
sfr R2CL        = 0xD4;
sfr R2CH        = 0xD5;
sfr R2DL        = 0xD6;
sfr R2DH        = 0xD7;

sfr P5          = 0xD8;
sfr P4          = 0xD9;
sfr P4M1        = 0xDA;
sfr P4S         = 0xDA;
sfr P4M2        = 0xDB;
sfr P5M1        = 0xDC;
sfr P5S         = 0xDC;
sfr P5M2        = 0xDD;

sfr ACC         = 0xE0;
sfr ADCCON1     = 0xE1;
sfr ADCCON2     = 0xE2;
sfr ADCMPL      = 0xE3;
sfr ADCMPH      = 0xE4;
sfr LCDSEG0     = 0xE5;
sfr LCDSEG1     = 0xE6;
sfr LCDSEG2     = 0xE7;

sfr ADCCON0     = 0xE8;
sfr PICON       = 0xE9;
sfr PINEN       = 0xEA;
sfr PIPEN       = 0xEB;
sfr PIF         = 0xEC;
sfr PITYP       = 0xED;
sfr LCDSEG3     = 0xEE;
sfr EIP         = 0xEF;

sfr B           = 0xF0;
sfr ADCAQT      = 0xF2;
sfr P0DIDS      = 0xF6;
sfr EIPH        = 0xF7;

sfr SCON_1      = 0xF8;
sfr LCDCON      = 0xF9;
sfr LCDCLK      = 0xFA;
sfr LCDPTR      = 0xFB;
sfr LCDDAT      = 0xFC;
sfr EIP1        = 0xFE;
sfr EIPH1       = 0xFF;

/*  BIT Registers  */
/*  TCON  */
sbit TF1        = TCON^7;
sbit TR1        = TCON^6;
sbit TF0        = TCON^5;
sbit TR0        = TCON^4;
sbit IE1        = TCON^3;
sbit IT1        = TCON^2;
sbit IE0        = TCON^1;
sbit IT0        = TCON^0;

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

/*  IE  */
sbit EA         = IE^7;
sbit EADC       = IE^6;
sbit EBOD       = IE^5;
sbit ES         = IE^4;
sbit ET1        = IE^3;
sbit EX1        = IE^2;
sbit ET0        = IE^1;
sbit EX0        = IE^0;

/*  IP  */  
sbit PADC       = IP^6;
sbit PBOD       = IP^5;
sbit PS         = IP^4;
sbit PT1        = IP^3;
sbit PX1        = IP^2;
sbit PT0        = IP^1;
sbit PX0        = IP^0;

/*  I2CON  */
sbit I2CEN      = I2CON^6;
sbit STA        = I2CON^5;
sbit STO        = I2CON^4;
sbit SI         = I2CON^3;
sbit AA         = I2CON^2;

/*  T2CON  */
sbit TF2D       = T2CON^7;
sbit TF2C       = T2CON^6;
sbit TF2B       = T2CON^5;
sbit TF2A       = T2CON^4;
sbit TR2D       = T2CON^3;
sbit TR2C       = T2CON^2;
sbit TR2B       = T2CON^1;
sbit TR2A       = T2CON^0;

/*  PSW */
sbit CY         = PSW^7;
sbit AC         = PSW^6;
sbit F0         = PSW^5;
sbit RS1        = PSW^4;
sbit RS0        = PSW^3;
sbit OV         = PSW^2;
sbit P          = PSW^0;                

/*  ADCCON0  */
sbit ADCF       = ADCCON0^7;
sbit ADCS       = ADCCON0^6;
sbit ADCHS3     = ADCCON0^3;
sbit ADCHS2     = ADCCON0^2;
sbit ADCHS1     = ADCCON0^1;
sbit ADCHS0     = ADCCON0^0;                

/*  SCON_1  */
sbit SM0_1      = SCON_1^7;
sbit FE_1       = SCON_1^7; 
sbit SM1_1      = SCON_1^6; 
sbit SM2_1      = SCON_1^5; 
sbit REN_1      = SCON_1^4; 
sbit TB8_1      = SCON_1^3; 
sbit RB8_1      = SCON_1^2; 
sbit TI_1       = SCON_1^1; 
sbit RI_1       = SCON_1^0; 
 

/*  P0  */  
sbit P00        = P0^0;
sbit AIN0       = P0^0;
sbit T0         = P0^0;
sbit P01        = P0^1;
sbit AIN1       = P0^1;
sbit INT0       = P0^1;
sbit P02        = P0^2;
sbit AIN2       = P0^2;
sbit P03        = P0^3;
sbit AIN3       = P0^3;
sbit P04        = P0^4;
sbit AIN4       = P0^4;
sbit P05        = P0^5;
sbit AIN5       = P0^5;
sbit P06        = P0^6;
sbit AIN6       = P0^6;
sbit P07        = P0^7;
sbit AIN7       = P0^7;
sbit CLO        = P0^7;
                
/*  P1  */                      
sbit P10        = P1^0;
sbit P11        = P1^1;
sbit P12        = P1^2;
sbit P13        = P1^3;
sbit INT1       = P1^3;
sbit P14        = P1^4;
sbit T1         = P1^4;
sbit P15        = P1^5;
sbit T2AO1      = P1^5;
sbit P16        = P1^6;
sbit T2AO2      = P1^6;
sbit P17        = P1^7;
sbit T2BO1      = P1^7;

/*  P2  */ 
sbit P20        = P2^0; 
sbit T2BO2      = P2^0; 
sbit P21        = P2^1;
sbit RXD        = P2^1;
sbit P22        = P2^2;
sbit TXD        = P2^2;
sbit P23        = P2^3;
sbit SDA        = P2^3;
sbit P24        = P2^4;
sbit SCL        = P2^4;
sbit P25        = P2^5;
sbit P26        = P2^6;
sbit T2CO1      = P2^6;
sbit P27        = P2^7;
sbit T2CO2      = P2^7;

/*  P3  */  
sbit P30        = P3^0;
sbit T2DO1      = P3^0;
sbit P31        = P3^1;
sbit T2DO2      = P3^1;
sbit P32        = P3^2;
sbit P33        = P3^3;
sbit P34        = P3^4;
sbit P35        = P3^5;
sbit P36        = P3^6;
sbit P37        = P3^7;

/*  P5  */  
sbit P50        = P5^0;
sbit STADC      = P5^0;
sbit P51        = P5^1;
sbit P52        = P5^2;
sbit P53        = P5^3;
sbit P54        = P5^4;
sbit P55        = P5^5;
sbit P56        = P5^6;
sbit RXD_1      = P5^6;
sbit P57        = P5^7;
sbit TXD_1      = P5^7;