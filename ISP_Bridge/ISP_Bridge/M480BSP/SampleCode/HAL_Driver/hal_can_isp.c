#include "NuMicro.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_api.h"

#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08

STR_CANMSG_T rrMsg;
volatile uint8_t u8CAN_PackageFlag = 0;
void CAN_Test(void);

void CAN_Init(void)
{
    /* Enable CAN1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN1CKEN_Msk;
    /* Set PC multi-function pins for CAN1 RXD(PC.9) and TXD(PC.10) */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC9MFP_Msk | SYS_GPC_MFPH_PC10MFP_Msk)) |
                    (SYS_GPC_MFPH_PC9MFP_CAN1_RXD | SYS_GPC_MFPH_PC10MFP_CAN1_TXD);
    /* Set CAN transceiver to high speed mode */
    GPIO_SETMODE(PC, 11, GPIO_MODE_OUTPUT);
    PC11 = 0;
    CAN_SetBaudRate(CAN1, CAN_BAUD_RATE);
    CAN_EnableInt(CAN1, (CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk));
    NVIC_SetPriority(CAN1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN1_IRQn);
    /* Set CAN reveive message */
    //can_setRxMsg(CAN1);
    CAN_SetRxMsg(CAN1, MSG(0), CAN_STD_ID, Device0_ISP_ID);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if (u32IIDR == 1) {
        CAN_Receive(tCAN, 0, &rrMsg);
        u8CAN_PackageFlag = 1;
        //_EP_HID_IN_Handler(EP_HID_IN, &rrMsg.Data[0], 64);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN1 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;
    u8IIDRstatus = CAN1->IIDR;

    if (u8IIDRstatus == 0x00008000) {     /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if (CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if (CAN1->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
        }
    } else if (u8IIDRstatus != 0) {
        CAN_MsgInterrupt(CAN1, u8IIDRstatus);
        CAN_CLR_INT_PENDING_BIT(CAN1, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
}

/*----------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                         */
/*----------------------------------------------------------------------------*/
int32_t CAN_Package_Tx(CAN_T *tCAN, uint8_t *data)
{
    STR_CANMSG_T tMsg;
    /* Send a 11-bit Standard Identifier message */
    tMsg.FrameType = CAN_DATA_FRAME;
    tMsg.IdType    = CAN_STD_ID;
    tMsg.Id        = Master_ISP_ID;
    tMsg.DLC       = CAN_ISP_DtatLength;
    memcpy(&tMsg.Data, data, 8);
    u8CAN_PackageFlag = 0;
    return CAN_Transmit(tCAN, MSG(5), &tMsg);
}

void CAN_Transmit_ISP(uint32_t cmd, uint32_t data)
{
    uint32_t buf[2];
    buf[0] = cmd;
    buf[1] = data;
    CAN_Package_Tx(CAN1, (uint8_t *)buf);
}

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

void CAN_Test()
{
    CAN_Init();
    CAN_Transmit_ISP(CMD_GET_DEVICEID, 0);

    while (0 == u8CAN_PackageFlag);

    CAN_Transmit_ISP(CMD_READ_CONFIG, FMC_USER_CONFIG_0);

    while (0 == u8CAN_PackageFlag);

    CAN_Transmit_ISP(CMD_READ_CONFIG, FMC_USER_CONFIG_1);

    while (0 == u8CAN_PackageFlag);

    CAN_Transmit_ISP(CMD_READ_CONFIG, FMC_USER_CONFIG_2);

    while (0 == u8CAN_PackageFlag);
}

#ifdef __cplusplus
}
#endif
