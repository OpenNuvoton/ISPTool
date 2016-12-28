extern void Initial_UART0_RS485(void);
extern void SendChar_ToUART0(int ch);
extern unsigned char UART_package(void);
extern void auto_detect_command(void);
extern char GetChar_UART0(void);
extern unsigned  char UART_MasterSendDataT1(void);
extern unsigned char UART_MasterRcvDataT1(void);
extern unsigned char UART_MasterRcvDataT2(unsigned int time_out);
extern void Initial_UART2_Debug(void);
#define RS458_TX_ENEABLE PB2=1
#define RS458_TX_DISABLE PB2=0

