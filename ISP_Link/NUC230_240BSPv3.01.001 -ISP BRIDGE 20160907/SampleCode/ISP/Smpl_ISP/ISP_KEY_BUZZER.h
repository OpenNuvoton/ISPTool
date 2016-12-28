extern void Open_EINT0_KEY1(void);
extern unsigned char Get_Key_Input(void);
extern void Initial_Key_Input(void);
extern void Buzzer(void);
extern void Buzzer_FLASE_LED(void);

#define KEY1 PB14 //LOW IS PRESS
#define KEY2 PB9
#define KEY3 PB10
#define KEY4 PB11
#define EXTIO1 PD15
#define EXTIO2 PD14
#define LED_PASS_ON   PA13=0
#define LED_PASS_OFF  PA13=1
#define LED_FALSE_ON  PA12=0
#define LED_FALSE_OFF PA12=1
#define Buzzer_ON  PB8=0
#define Buzzer_OFF PB8=1
