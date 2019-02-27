
#define ACMP0		0
#define ACMP1		1

/*---------------------------------------------------------------------------------------------------------*/
/* ACMP_CTL constant definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define ACMP_CTL_POSSEL_P0           0
#define ACMP_CTL_POSSEL_P1           1
#define ACMP_CTL_POSSEL_P2           2
#define ACMP_CTL_POSSEL_P3           3
#define ACMP_CTL_NEGSEL_PIN_N0       0
#define ACMP_CTL_NEGSEL_CRV          1
#define ACMP_CTL_NEGSEL_VBG          2
#define ACMP_CTL_NEGSEL_PIN_N1       3
#define ACMP_CTL_WAKEUP_ENABLE       0x08
#define ACMP_CTL_WAKEUP_DISABLE      0x00
#define ACMP_CTL_HYSTERESIS_ENABLE   0x04
#define ACMP_CTL_HYSTERESIS_DISABLE  0x00
#define ACMP_CTL_INT_ENABLE       	 0x02
#define ACMP_CTL_INT_DISABLE       	 0x00

#define ACMP_CTL_CRV_VREF            0x02
#define ACMP_CTL_CRV_VDD             0x00
#define ACMP_CTL_CRV_ENABLE          0x01
#define ACMP_CTL_CRV_DISABLE         0x00

#define ACMP_CTL_ACMP0_OUTPUT_ENABLE          0x20
#define ACMP_CTL_ACMP0_OUTPUT_DISABLE					0x00
#define ACMP_CTL_ACMP1_OUTPUT_ENABLE					0x10
#define ACMP_CTL_ACMP0_OUTPUT_DISABLE					0x00
/*---------------------------------------------------------------------------------------------------------*/
/* ACMP_VREF constant definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define ACMP_VREF_CRVSSEL_VDDA       (0UL << 6)  /*!< ACMP_VREF setting for selecting analog supply voltage VDDA as the CRV source voltage \hideinitializer */
#define ACMP_VREF_CRVSSEL_INTVREF    (1UL << 6)  /*!< ACMP_VREF setting for selecting internal reference voltage as the CRV source voltage \hideinitializer */


void ACMP_Open(unsigned char u8ACMPNum, unsigned char u8PosSrc, unsigned char u8NegSrc, unsigned char u8CVRSource, unsigned char u8ACMPOut, unsigned char u8HysteresisEn);
void ACMP_INTEnable(unsigned char u8ACMPNum, unsigned char u8ACMPWakeEn,unsigned char u8ACMPINTEn);
void ACMP_CRVValue(unsigned char u8ACMPNum, unsigned char u8ACMPValue);