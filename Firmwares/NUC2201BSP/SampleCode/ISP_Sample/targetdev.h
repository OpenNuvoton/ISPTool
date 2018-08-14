
#include "NuMicro.h"
#include "ISP_UART\uart_transfer.h"
#include "ISP_HID\hid_transfer.h"
#include "isp_user.h"

#define DetectPin   				PA10

extern __align(4) uint8_t response_buff[64];
extern __align(4) uint8_t usb_rcvbuf[64];
extern uint8_t bUsbDataReady;


/* rename for uart_transfer.c */
#define UART_N							UART0
#define UART_N_IRQHandler		UART02_IRQHandler
#define UART_N_IRQn					UART02_IRQn
