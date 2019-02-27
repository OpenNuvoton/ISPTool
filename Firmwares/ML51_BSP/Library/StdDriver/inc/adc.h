#define ADC_SINGLE	1
#define ADC_CONTINUOUS	2
#define VBG				8
#define VTEMP			9
#define VLDO			10

#define ADC_HWT_PWM0CH0		0
#define ADC_HWT_PWM0CH2		1
#define ADC_HWT_PWM0CH4		2
#define ADC_HWT_STADC		3

#define ADC_HWT_FALLINGEDGE	0
#define ADC_HWT_RISINGEDGE 		1
#define ADC_HWT_CENTRAL		2
#define ADC_HWT_END    		3

#define ADC_INT_HALFDONE 0
#define ADC_INT_CONTDONE 1
#define ADC_INT_SINGLE 3

#define ADCSlowSpeed    0
#define ADCHighSpeed    1
         
void ADC_Open(unsigned char u8ADCOpMode, unsigned char u8ADCChMask);
void ADC_Close(void);
void ADC_InitialContinous(unsigned int u16ADCRBase, unsigned char u8ADCRLength, unsigned char u8ADCSpeed);
void ADC_ConvertTime(unsigned char u8ADCDIV, unsigned char u8ADCAQT);
void ADC_EnableHWTrigger(unsigned char u8ADCSHWTource, unsigned char u8ADCHWTParam, unsigned char u8ADCHWTDelay);
void ADC_DisableHWTrigger(void);
void ADC_Interrupt(unsigned char u8ADCINT, unsigned char u8ADCIntSource);
unsigned int READ_BANDGAP();
void ADC_Calibration(void);