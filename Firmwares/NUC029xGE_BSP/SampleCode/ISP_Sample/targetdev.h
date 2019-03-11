
#include "NUC029xGE.h"
#include "isp_user.h"
#include "ISP_HID\hid_transfer.h"
#include "ISP_UART\uart_transfer.h"

#define DetectPin   			PA3

/* rename for uart_transfer.c */
#define UART_T					UART0
#define UART_T_IRQHandler		UART02_IRQHandler
#define UART_T_IRQn				UART02_IRQn

#define CONFIG_SIZE 8 // in bytes
