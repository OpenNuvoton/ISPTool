#ifndef __ISP_BSP_H__
#define __ISP_BSP_H__
extern void ISP_BSP_SYSTEM_CLOCK_INITIAL(void);
extern void RTC_SET(void);
extern void RTC_Init(void);

#define Read_RTC()    RTC_GetDateAndTime(&sReadRTC)

extern void TEST_PIN_INIT(void);
extern void IO_TEST_MODE(void);


#define PWC_C_ENABLE PA4=0 //TVCC is output 3.3V
#define PWC_C_DISABLE PA4=1 //TVCC no output


extern void Initial_USB(void);
#define T1   PE7  //CONNECT PB1
#define T2   PE8  //CONNECT PB0
#define T3   PA10 //CONNECT PA9
#define T4   PA11 //CONNECT PA8
#define T5_1 PB7  //MUST SET INPUT MODE
#define T6_1 PB6  //MUST SET INPUT MODE
#define T5   PD4  //CONNECT PD3
#define T6   PD5  //CONNECT PD2
#define T7   PC7  //CONNECT PD1
#define T8   PC6  //CONNECT PD0
#define T9   PB15 //RS485_A
#define T10  PB13 //RS485_B
#define Protocol_package 64
#define PACKET_SIZE	64
#include "ISP_UART0.h"
#include "ISP_KEY_BUZZER.h"
#include "ISP_ACMP_ADC.h"
#include "ISP_SPI_FLASH.h"
#include "ISP_SDCARD.h"
#include "ISP_LCD.h"
#include "ISP_SPI2.h"
#include "ISP_CAN0.H"
#include "ISP_I2C0.H"
#include "ISP_UART1WIRE.H"

#include "ISP_FILE_ACCESS.h"
extern __align(4) uint8_t rcvbuf[PACKET_SIZE];
extern __align(4) uint8_t sendbuf[PACKET_SIZE];
extern __align(4)  uint8_t response_buff[PACKET_SIZE];
#define dbg_printf printf
//#define dbg_printf(...)

extern volatile unsigned int file_totallen;
extern volatile unsigned int file_checksum;
//#define UART_BUS 1
//#define SPI_BUS 1
#define UART1WIRE 1

#endif

