extern uint32_t Get_ADC0_Value(void);
extern void Open_ADC0(void);
extern void Open_ACMP1(void);
extern uint32_t Get_ACMP1(void); //return 0, the tvcc voltage is big that vcc
extern void Poll_ADC0(void);
extern volatile double TVCC_voltage_old;
extern volatile double TVCC_voltage_new;


