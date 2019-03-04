
#include "NUC1261.h"
#include "ISP_UART\uart_transfer.h"
#include "ISP_HID\hid_transfer.h"
#include "isp_user.h"

/* rename for uart_transfer.c */
#define UART_T					UART0
#define UART_T_IRQHandler		UART02_IRQHandler
#define UART_T_IRQn				UART02_IRQn

#define DetectPin   				PA3
