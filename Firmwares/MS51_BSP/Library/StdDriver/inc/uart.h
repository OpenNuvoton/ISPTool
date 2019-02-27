#define UART0_Timer1  0
#define UART0_Timer3  1
#define UART1_Timer3  2
#define UART0 0
#define UART1 1

void UART_Open(unsigned long u32SysClock, unsigned char u8UARTPort,unsigned long u32Baudrate);
void UART_Send_Data(UINT8 UARTPort, UINT8 c);
unsigned char Receive_Data(unsigned char UARTPort);