#include <stdio.h>
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef EXPORT_LIB
#define printf(...)
#define EP_HID_IN  EPJ
#define EP_HID_OUT EPK
#else
#define __set_PRIMASK(...)
#define EP_HID_IN  EPA
#define EP_HID_OUT EPB

#endif

#define GPIO_SETMODE(port, pin, u32Mode) port->MODE = (port->MODE & ~(0x3ul << (pin << 1))) | (u32Mode << (pin << 1));

extern uint32_t _CyclesPerUs;         /*!< Cycles per micro second              */

// hal_hsusbd.c
//void _EP_HID_IN_Handler(uint32_t u32Ep, uint8_t *pu8Buf);  /* Interrupt IN handler */
void _EP_HID_IN_Handler(uint32_t u32Ep, uint8_t *pu8Buf, uint32_t u32rLen);  /* Interrupt IN handler */
void _EP_HID_OUT_Handler(uint32_t u32Ep, uint8_t *pu8Buf);  /* Interrupt OUT handler */

// hal_spi.c
void SPI1_Init(uint32_t Pclk0);
uint32_t SPI1_Write(uint32_t *buf, uint32_t len);
uint32_t SPI1_Read(uint32_t *buf, uint32_t len);

// hal_sys.c
void UART_Init(void); // for printf
void PrintfBufByte(uint8_t *Buf, uint32_t len);
void PrintfBufWord(uint32_t *Buf, uint32_t len);

// hal_usci_i2c.c
void UI2C0_Init(uint32_t Pclk0, uint32_t u32BusClock);
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *data, uint32_t u32wLen);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);

// hal_timer.c
void TIMER3_Init(void);

// hal_rs485.c
void RS485_Init(void);
void RS485_WriteMultiBytes(uint8_t *data);

// hal_can_isp.c
extern volatile uint8_t u8CAN_PackageFlag;
extern STR_CANMSG_T rrMsg;
void CAN_Init(void);
int32_t CAN_Package_Tx(CAN_T *tCAN, uint8_t *data);

#ifdef __cplusplus
}
#endif
