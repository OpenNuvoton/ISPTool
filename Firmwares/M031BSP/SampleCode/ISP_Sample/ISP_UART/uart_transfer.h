#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE        	64

/*-------------------------------------------------------------*/

extern uint8_t  uart_rcvbuf[];
extern uint8_t volatile bUartDataReady;
extern uint8_t volatile bufhead;

/*-------------------------------------------------------------*/
void UART_Init(void);
void UART0_IRQHandler(void);
void PutString(void);

#include "clk.h"
///*---------------------------------------------------------------------------------------------------------*/
///* UART_FUNCSEL constants definitions                                                                      */
///*---------------------------------------------------------------------------------------------------------*/
//#define UART_FUNCSEL_UART  (0x0ul << UART_FUNCSEL_FUNCSEL_Pos) /*!< UART_FUNCSEL setting to set UART Function  (Default) \hideinitializer */
//#define UART_FUNCSEL_LIN   (0x1ul << UART_FUNCSEL_FUNCSEL_Pos) /*!< UART_FUNCSEL setting to set LIN Function             \hideinitializer */
//#define UART_FUNCSEL_IrDA  (0x2ul << UART_FUNCSEL_FUNCSEL_Pos) /*!< UART_FUNCSEL setting to set IrDA Function            \hideinitializer */
//#define UART_FUNCSEL_RS485 (0x3ul << UART_FUNCSEL_FUNCSEL_Pos) /*!< UART_FUNCSEL setting to set RS485 Function           \hideinitializer */
//#define UART_FUNCSEL_SINGLE_WIRE (0x4ul << UART_FUNCSEL_FUNCSEL_Pos) /*!< UART_FUNCSEL setting to set Single Wire Function           \hideinitializer */
#define UART_FUNCSEL_MODE (UART_FUNCSEL_UART)


#endif  /* __UART_TRANS_H__ */
