
#include "NUC100Series.h"
#include "hid_transfer.h"
#include "ISP_USER.h"
#include <stdint.h>

/* rename for uart_transfer.c */
#define UART_N							UART1
#define UART_N_IRQHandler		UART1_IRQHandler
#define UART_N_IRQn					UART1_IRQn

/* 0: Disable, 1: Enable */
#define USE_ISP_HID					(1)
#define DetectPin   				PB15

#define USE_ISP_UART				(0)
