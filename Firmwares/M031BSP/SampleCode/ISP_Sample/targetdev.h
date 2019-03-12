
#include "NuMicro.h"
#include "ISP_HID\hid_transfer.h"
#include "ISP_UART\uart_transfer.h"
#include "isp_user.h"

#define DetectPin   				PB12

/* rename for uart_transfer.c */
#define UART_N							UART0
#define UART_N_IRQHandler		UART02_IRQHandler
#define UART_N_IRQn					UART02_IRQn

#define CONFIG_SIZE 8 // in bytes
