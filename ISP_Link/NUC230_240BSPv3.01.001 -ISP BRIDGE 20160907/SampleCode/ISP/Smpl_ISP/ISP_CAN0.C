#include "NUC230_240.h"
#include <string.h>
#include "ISP_BSP.h"
extern int32_t CAN_SetRxMsgObj(CAN_T  *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint8_t u8singleOrFifoLast);
#define CAN_ID_ISP_RX 0X100
#define CAN_ID_ISP_TX 0X102
void OPEN_CAN0(void)
{

	unsigned int i=0;
	CAN_T *tCAN;
	tCAN = (CAN_T *)CAN0;

	/* Enable CAN module clock */
	CLK_EnableModuleClock(CAN0_MODULE);
	/* Set PD multi-function pins for CANTX0, CANRX0 */
	SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
	SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;
	CAN_Open(tCAN, 1000000, CAN_NORMAL_MODE);
	
	for(i=8;i<15;i++)
	CAN_SetRxMsgObj(tCAN, MSG(i), CAN_STD_ID, CAN_ID_ISP_RX, FALSE);
	CAN_SetRxMsg(tCAN, MSG(15), CAN_STD_ID, CAN_ID_ISP_RX); //FOR LAST CAN PACKAGE
	
	GPIO_SetMode(PC, BIT4, GPIO_PMD_OUTPUT);
	GPIO_SetMode(PC, BIT5, GPIO_PMD_OUTPUT);
	PC4 = 0;
	PC5 = 0;
}
void CAN_MasterRcvData(void)
{

	unsigned int int_count;
	STR_CANMSG_T rrMsg;
			CAN_T *tCAN;
	tCAN = (CAN_T *)CAN0;

	int_count = 1;
  while (tCAN->IIDR == 0); 
	  CLK_SysTickDelay(2000);
	while (1) {
		                 /* Wait IDR is changed */
												  //printf("IDR = %x\n", tCAN->IIDR);
		CAN_Receive(tCAN, tCAN->IIDR - 1, &rrMsg); /* Get the message */
												   //CAN_ShowMsg(&rrMsg);                     /* Show the message object */
		memcpy(&response_buff[(int_count - 1) * 8], &rrMsg.Data[0], 8);
		int_count++;
		if (int_count>8)
			break;
	}

}

void CAN_MasterSendData(void)
{
	
	unsigned char i;
	STR_CANMSG_T tMsg;
	CAN_T *tCAN;
	tCAN = (CAN_T *)CAN0;

	for (i = 0; i<8; i++)
	{
		tMsg.FrameType = CAN_DATA_FRAME;
	  tMsg.IdType = CAN_STD_ID;
	  tMsg.DLC = 8;
	  tMsg.Id = CAN_ID_ISP_TX;
		memcpy(&tMsg.Data[0], &rcvbuf[i << 3], 8);
		CAN_Transmit(tCAN, MSG(i), &tMsg);
		CLK_SysTickDelay(200);
		CAN_CLR_INT_PENDING_BIT(tCAN,i);
	}
}

void CAN_package(void)
{

	CAN_MasterSendData();
	//CLK_SysTickDelay(20000); //delay for devices ack.
	CAN_MasterRcvData();
}

