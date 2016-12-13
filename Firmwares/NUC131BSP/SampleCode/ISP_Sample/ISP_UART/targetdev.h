
#include "NUC131.h"

/* rename for uart_transfer.c */
#define UART_T					UART0
#define UART_T_IRQHandler		UART02_IRQHandler
#define UART_T_IRQn				UART02_IRQn

/*
// UART_T define option
#define UART_T					UART
#define UART_T					UART0
#define UART_T					UART1
*/

/*
// UART_T_IRQHandler define option
#define UART_T_IRQHandler		UART_IRQHandler
#define UART_T_IRQHandler		UART0_IRQHandler
#define UART_T_IRQHandler		UART1_IRQHandler

*/

/*
// UART_T_IRQn define option
#define UART_T_IRQn					UART_IRQn
#define UART_T_IRQn					UART0_IRQn
#define UART_T_IRQn					UART1_IRQn
*/
