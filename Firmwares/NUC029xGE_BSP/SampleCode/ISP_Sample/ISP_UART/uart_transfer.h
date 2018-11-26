#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>
#include "targetdev.h"

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE        	64

/*-------------------------------------------------------------*/

extern uint8_t  uart_rcvbuf[];
extern uint8_t volatile bUartDataReady;
extern uint8_t volatile bufhead;

/*-------------------------------------------------------------*/
void UART_Init(void);
void UART_T_IRQHandler(void);
void PutString(void);
uint32_t UART_IS_CONNECT(void);

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
