#include "NUC230_240.h"
#include "ISP_BSP.h"
void Open_SPI2(void)
{
//GPIO_SetMode(PC,BIT10,GPIO_PMD_INPUT);
	/* Set PD0, PD1, PD2 and PD3 for SPI2 */
	SYS->GPD_MFP |= SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;
  SYS->ALT_MFP = SYS_ALT_MFP_PD0_SPI2_SS0 | SYS_ALT_MFP_PD1_SPI2_CLK | SYS_ALT_MFP_PD2_SPI2_MISO0 | SYS_ALT_MFP_PD3_SPI2_MOSI0;
	/* Enable SPI2 IP clock */
	CLK->APBCLK |= CLK_APBCLK_SPI2_EN_Msk;

	/* Configure SPI2 as a master, MSB first, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
	//SPI2->CNTRL = SPI_CNTRL_MASTER_MODE | SPI_CNTRL_MSB_FIRST | SPI_CNTRL_CLK_IDLE_LOW | SPI_CNTRL_TX_FALLING |
	//            SPI_CNTRL_RX_RISING | SPI_CNTRL_TX_BIT_LEN(32);

	SPI_Open(SPI2, SPI_MASTER, SPI_MODE_0, 8, 800000);
	/* Disable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
	//SPI_DisableAutoSS(SPI2);	
	SPI_EnableAutoSS(SPI2, SPI_SS0, SPI_SS_ACTIVE_LOW);
	//SPI2->CNTRL=0x05023840;
	//SPI_SET_SUSPEND_CYCLE(SPI2,6);
}
void SPI_MasterSendData(void)
{
	uint32_t temp_count;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		/* Write to TX register */
		SPI_WRITE_TX0(SPI2, rcvbuf[temp_count]);
		//SPI_WRITE_TX0(SPI2,temp_count);
		/* Trigger SPI data transfer */
		SPI_TRIGGER(SPI2);
		/* Check SPI0 busy status */
		while (SPI_IS_BUSY(SPI2));
		CLK_SysTickDelay(100);
	}
}

unsigned  char SPI_MasterSendDataT1(void)
{
	uint32_t temp_count;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		/* Write to TX register */
		SPI_WRITE_TX0(SPI2, sendbuf[temp_count]);
		//SPI_WRITE_TX0(SPI2,temp_count);
		/* Trigger SPI data transfer */
		SPI_TRIGGER(SPI2);
		/* Check SPI0 busy status */
		while (SPI_IS_BUSY(SPI2));
		CLK_SysTickDelay(100);
	}
	return TRUE;
}

unsigned char SPI_MasterRcvDataT1(void)
{
	uint32_t temp_count;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		/* Write to TX register */
		SPI_WRITE_TX0(SPI2, 0xff);
		/* Trigger SPI data transfer */
		SPI_TRIGGER(SPI2);
		/* Check SPI0 busy status */
		while (SPI_IS_BUSY(SPI2));
		rcvbuf[temp_count] = SPI_READ_RX0(SPI2);
		CLK_SysTickDelay(100);
	}
	return TRUE;
}

void SPI_MasterRcvData(void)
{
	uint32_t temp_count;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		/* Write to TX register */
		//SPI_WRITE_TX0(SPI2, 0xff);
		/* Trigger SPI data transfer */
		SPI_TRIGGER(SPI2);
		/* Check SPI0 busy status */
		while (SPI_IS_BUSY(SPI2));
		response_buff[temp_count]=SPI_READ_RX0(SPI2);
		CLK_SysTickDelay(100);
	}
}
void SPI_package(void)
{
	SPI_MasterSendData();
	CLK_SysTickDelay(50000); //delay for devices ack.
	SPI_MasterRcvData();
}

void SPI_package_erase(void)
{
 int i;
	SPI_MasterSendData();
	for(i=0;i<12000;i++)
	CLK_SysTickDelay(1000); //delay for devices ack.
	SPI_MasterRcvData();
}

