
#include "Mini58Series.h"

/* rename for uart_transfer.c */
#define UART_T							UART0
#define UART_T_IRQHandler		UART0_IRQHandler
#define UART_T_IRQn					UART0_IRQn

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


/* MFP setting used in SYS_INIT */

/* Mini51 and Mini58 UART RX MFP Option */
#define SYS_MFP_P01_UART0_RXD        0x00000202UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P12_UART0_RXD        0x00000400UL     /*!< Data receiver input pin for UART0.               */
/* Mini58 Only UART RX MFP Option */
#define SYS_MFP_P14_UART1_RXD        0x00001000UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P24_UART1_RXD        0x00000010UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P46_UART1_RXD        0x00004000UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P51_UART0_RXD        0x00000202UL     /*!< Data receiver input pin for UART0.               */

/* Mini51 and Mini58 UART TX MFP Option */
#define SYS_MFP_P00_UART0_TXD        0x00000101UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P13_UART0_TXD        0x00000800UL     /*!< Data transmitter output pin for UART0.           */
/* Mini58 Only UART TX MFP Option */
#define SYS_MFP_P15_UART1_TXD        0x00002000UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P25_UART1_TXD        0x00000020UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P47_UART1_TXD        0x00008000UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P50_UART0_TXD        0x00000101UL     /*!< Data transmitter output pin for UART0.           */
