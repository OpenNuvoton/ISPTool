
#include "Mini55Series.h"
#include "ISP_USER.h"
#include "uart_transfer.h"

/* rename for uart_transfer.c */
#define UART_N							UART0
#define UART_N_IRQHandler		UART0_IRQHandler
#define UART_N_IRQn					UART0_IRQn
