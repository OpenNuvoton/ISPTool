
#include "ISD9000.h"
#include "uart_transfer.h"
#include "ISP_USER.h"

/* rename for uart_transfer.c */
#define UART_N							UART0
#define UART_N_IRQHandler		UART0_IRQHandler
#define UART_N_IRQn					URT0_IRQn
