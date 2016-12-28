#include "NUC230_240.h"
#include "ISP_BSP.h"

//uart0, RS485 
void Initial_UART0_RS485(void)
{
	//for time out test
 CLK_EnableModuleClock(TMR0_MODULE); 
 CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, NULL);
 
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
	
	  /* Set PB.0 and PB.1 multi-function pins for UART0 RXD, UART0 TXD */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
	
	  /* Init UART to 115200-8n1 for print message */
	  UART_Open(UART0, 115200);
	
	    /* Configure PB.2 as Output mode for RS485 transfer*/
    GPIO_SetMode(PB, BIT2, GPIO_PMD_OUTPUT);
	  GPIO_SetMode(PA, BIT13, GPIO_PMD_OUTPUT);
	  GPIO_SetMode(PA, BIT14, GPIO_PMD_OUTPUT);
}

void SendChar_ToUART0(int ch)
{

    while(UART0->FSR & UART_FSR_TX_FULL_Msk);
    UART0->DATA = ch;
    if(ch == '\n')
    {
        while(UART0->FSR & UART_FSR_TX_FULL_Msk);
        UART0->DATA = '\r';
    }
}

char GetChar_UART0(void)
{
    while(1)
    {
        if((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
        {
            return (UART0->DATA);
        }
    }
}

unsigned char UART_MasterRcvDataT1(void)
{
	uint32_t temp_count;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		while (1)
		{
			if ((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
			{
				rcvbuf[temp_count] = UART0->DATA;
				break;
			}
		}
	}
	return TRUE;
}

unsigned  char UART_MasterSendDataT1(void)
{
	uint32_t temp_count;
	 
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		while (UART0->FSR & UART_FSR_TX_FULL_Msk);
		UART0->DATA = sendbuf[temp_count];
	}
	return TRUE;
}


unsigned char UART_MasterRcvDataT2(unsigned int time_out)
{
	uint32_t temp_count=0,time_count=0;
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
  TIMER_Start(TIMER0);
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		while (1)
		{
			if ((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
			{
				rcvbuf[temp_count] = UART0->DATA;
				break;
			}
			if(TIMER_GetIntFlag(TIMER0) == 1)
			 {
				// EXTIO2^=1;
		/* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
				time_count++;
				if(time_count>=time_out) 
				{
					TIMER0->TCSR |=TIMER_TCSR_CRST_Msk;
					return FALSE;
		   }
				 //EXTIO2^=1;
	    }
}
		}
	TIMER0->TCSR |=TIMER_TCSR_CRST_Msk;
	return TRUE;

}

#if 1
unsigned char UART_package(void)
{
	unsigned char  temp_count;
	for (temp_count = 0; temp_count < Protocol_package; temp_count++)
	{
		while (UART0->FSR & UART_FSR_TX_FULL_Msk);
		UART0->DATA = rcvbuf[temp_count];
	}
	for (temp_count = 0; temp_count < Protocol_package; temp_count++)
	{
		while (1)
		{
			if ((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
			{				
				response_buff[temp_count]=UART0->DATA;
				break;
			}
		}
	}
	return TRUE;
}



void auto_detect_command(void)
{
	unsigned char  temp_count;
	unsigned int time_count;
	while (1)
	{
	Detect_LOOP:
		for (temp_count = 0; temp_count < Protocol_package; temp_count++)
		{
			while (UART0->FSR & UART_FSR_TX_FULL_Msk);
			UART0->DATA = rcvbuf[temp_count];
		}

		for (temp_count = 0; temp_count < Protocol_package; temp_count++)
		{
			time_count = 0;
			while (1)
			{
				if (time_count>0x10000)
				{
					goto Detect_LOOP;
				}
				time_count++;
				if ((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
				{
					response_buff[temp_count] = UART0->DATA;
					break;
				}
			}
		}
		if (response_buff[4] == (rcvbuf[4] + 1))
			break;
	}

}

#endif


//uart0, RS485 
void Initial_UART2_Debug(void)
{
    /* Select IP clock source */
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART2_MODULE);
	
	  /* Set PB.0 and PB.1 multi-function pins for UART0 RXD, UART0 TXD */
    SYS->GPD_MFP |= SYS_GPD_MFP_PD14_UART2_RXD | SYS_GPD_MFP_PD15_UART2_TXD;
	
	  /* Init UART to 115200-8n1 for print message */
	  UART_Open(UART2, 115200);		  
}

