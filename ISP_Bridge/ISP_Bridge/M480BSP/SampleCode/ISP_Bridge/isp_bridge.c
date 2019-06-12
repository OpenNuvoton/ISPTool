#include <stdio.h>
#include "NuMicro.h"
#include "isp_bridge.h"
#include <string.h>
#include "hal_api.h"

#ifdef __cplusplus
extern "C"
{
#endif


static volatile uint8_t g_lib_CmdFromTool = 0ul;
static volatile uint8_t g_lib_CmdToTarget = 0ul;
static volatile uint8_t g_lib_IspModule = 0ul; // 3: SPI, 4: I2C, 5: RS485, 6: CAN
static volatile uint32_t g_lib_Delay = 0ul;

static uint8_t g_lib_CmdBuf[64] __attribute__((aligned(4)));
static uint8_t g_lib_AckBuf[64] __attribute__((aligned(4)));

void ISP_Bridge_UsbDataIn()
{
    _EP_HID_IN_Handler(EP_HID_IN, g_lib_AckBuf, 64);
}

void ISP_Bridge_UsbDataOut()
{
    _EP_HID_OUT_Handler(EP_HID_OUT, g_lib_CmdBuf);
    g_lib_CmdFromTool = TRUE;
}

void ISP_Bridge_Init(void)
{
    uint32_t Pclk0 = FREQ_192MHZ / 2;
    SPI1_Init(Pclk0);
    UI2C0_Init(Pclk0, 100000);
    RS485_Init();
    CAN_Init();
}

__STATIC_INLINE uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

void ISP_Bridge_Main(void)
{
    static uint32_t cks;
    static uint8_t cmd;

    // forward isp command to target device (I2C, SPI, RS485 or CAN)
    if (g_lib_CmdFromTool == TRUE) {
        g_lib_CmdFromTool = FALSE;
        cmd = g_lib_CmdBuf[0];
        g_lib_IspModule = g_lib_CmdBuf[1];
        g_lib_CmdBuf[1] = 0;
        // Checksum is the only way to verify correctness of ACK according to spec.
        cks = Checksum(g_lib_CmdBuf, 64);

        switch (g_lib_IspModule) {
            case 3:
                // add specific pattern "0x53504900" to word0 of g_lib_CmdBuf
                __set_PRIMASK(1);
                SPI1_Write((uint32_t *)g_lib_CmdBuf, 16);
                __set_PRIMASK(0);
                g_lib_CmdToTarget = 1;
                break;

            case 4:
                __set_PRIMASK(1);

                if (64 == UI2C_WriteMultiBytes(UI2C0, 0x60, g_lib_CmdBuf, 64)) {
                    g_lib_CmdToTarget = 1;
                } else {
                }

                __set_PRIMASK(0);
                break;

            case 5:
                __set_PRIMASK(1);
                RS485_WriteMultiBytes(g_lib_CmdBuf);
                __set_PRIMASK(0);
                g_lib_CmdToTarget = 0;
                return;

            case 6:
                __set_PRIMASK(1);
                CAN_Package_Tx(CAN1, g_lib_CmdBuf + 2);
                __set_PRIMASK(0);
                g_lib_CmdToTarget = 0;
                return;

            default:
                g_lib_CmdToTarget = 0;
                return;
        }

        // There's no ACK for these commands according to spec. (RESET command)
        // #define CMD_RUN_APROM         0x000000AB
        // #define CMD_RUN_LDROM         0x000000AC
        // #define CMD_RESET             0x000000AD
        if (g_lib_CmdToTarget && ((cmd == 0xAB) || (cmd == 0xAC) || (cmd == 0xAD))) {
            g_lib_CmdToTarget = 0;
        }

        return;
    }

    // checking response for CAN interface, this flag is set in CAN1_IRQHandler
    if (u8CAN_PackageFlag && (6 == g_lib_IspModule)) {
        u8CAN_PackageFlag = 0;
        _EP_HID_IN_Handler(EP_HID_IN, &rrMsg.Data[0], 64);
    }

    // polling response for SPI & I2C interface
    if (g_lib_CmdToTarget && (g_lib_CmdFromTool == FALSE)) {
        uint32_t delay = 349525UL, us = 0;

        if ((cmd == 0xA0) || (cmd == 0xA3) || (cmd == 0xC3)) {
            // #define CMD_UPDATE_APROM      0x000000A0
            // #define CMD_ERASE_ALL         0x000000A3
            // #define CMD_UPDATE_DATAFLASH  0x000000C3
            g_lib_Delay = 300000;
        } else if (cmd == 0xAE) {
            // #define CMD_CONNECT           0x000000AE
            g_lib_Delay = 20000;
        } else {
            g_lib_Delay = 50000;
        }

        us = g_lib_Delay;

        if (us > delay) {
            us -= delay;
        } else {
            delay = us;
            us = 0UL;
        }

        SysTick->LOAD = delay * _CyclesPerUs;
        SysTick->VAL  = 0x0UL;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        while (g_lib_CmdFromTool == FALSE) {
            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                uint32_t u32rxLen = 64;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;

                switch (g_lib_IspModule) {
                    case 3:
                        SPI1_Read((uint32_t *)g_lib_AckBuf, 16);
                        break;

                    case 4:
                    default:
                        u32rxLen = UI2C_ReadMultiBytes(UI2C0, 0x60, g_lib_AckBuf, 64);
                        break;
                }

                if ((u32rxLen == 64) && cks == inpw(g_lib_AckBuf)) {
                    ISP_Bridge_UsbDataIn();
                    g_lib_CmdToTarget = 0;
                    return;
                } else if (cmd == 0xAE) {
                    g_lib_CmdToTarget = 0;
                } else { // try again
                    SysTick->LOAD = delay * _CyclesPerUs;
                    SysTick->VAL  = 0x0UL;
                    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock
                }
            }
        }
    }
}

#ifdef __cplusplus
}
#endif
