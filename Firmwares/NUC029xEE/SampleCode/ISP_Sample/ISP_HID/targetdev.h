
#include "NuMicro.h"
#include "hid_transfer.h"
#include "ISP_USER.h"

#define DetectPin   				PA10

extern __align(4) uint8_t response_buff[64];
extern __align(4) uint8_t usb_rcvbuf[64];
extern uint8_t bUsbDataReady;

