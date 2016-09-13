
#include "NUC200Series.h"
#include "uart_transfer.h"
#include "ISP_USER.h"

/* rename for uart_transfer.c */
#define UART_N							UART0
#define UART_N_IRQHandler		UART02_IRQHandler
#define UART_N_IRQn					UART02_IRQn
