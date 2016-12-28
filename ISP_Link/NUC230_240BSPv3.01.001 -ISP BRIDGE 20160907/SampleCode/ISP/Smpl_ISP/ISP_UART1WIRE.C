 #include "NUC230_240.h"
#include "ISP_BSP.h"
 
 
 void INIT_UART_1WIRE(void)
 { 
	/* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, NULL);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, NULL);
	    /* Reset UART0 */
	SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
	GPIO_SetMode(PB, BIT0, GPIO_PMD_OUTPUT);
}

void SET_IO_UART_RX(void)
{	
   SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD;
	UART0->FCR |=UART_FCR_RFR_Msk;
}

void SET_IOMODE(void)
{	
   SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk);
		UART0->FCR |=UART_FCR_RFR_Msk;
}

volatile unsigned int data;
volatile unsigned int count;
volatile unsigned int time_out_flag;


void TMR0_IRQHandler(void)
{
    /* Clear Timer0 time-out interrupt flag */
     TIMER_ClearIntFlag(TIMER0);     
	   if(count!=0)
		 {
			if((data&0x01)==0x01)
	     PB0=1;
			 else
			 PB0=0;
		  data=data>>1;
			count--;
		 }
	
}

void UART_GPIO(unsigned char uart_data)
{
	data=0;
	data=(uart_data<<1)|(1<<9);
	count=10;
}


void UART1WIRTE_TX_64(unsigned char *buffer)
{
unsigned char temp;
	CLK_SysTickDelay(1000);	
SET_IOMODE();
/* Open Timer0 frequency 115200 in periodic mode, and enable interrupt */
TIMER0->TCMPR = (22118400/115200);
TIMER0->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
TIMER_SET_PRESCALE_VALUE(TIMER0, 0);	

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

for(temp =0;temp <64;temp++)
 {
  UART_GPIO(buffer[temp]);
	TIMER_Start(TIMER0);	
  while(count!=0);
	 TIMER_Stop(TIMER0);
	 		CLK_SysTickDelay(20);//need delay for time
 }
     /* Start Timer0 counting */
    	 
	 
	 /* Enable Timer0 NVIC */
    NVIC_DisableIRQ(TMR0_IRQn);
}

void TMR1_IRQHandler(void)
{
    /* Clear Timer0 time-out interrupt flag */
     TIMER_ClearIntFlag(TIMER1);     
	   time_out_flag=1;

}

unsigned int UART1WIRE_RX_64(unsigned char *buffer)
{
	unsigned char temp;
	SET_IO_UART_RX();
	TIMER1->TCMPR = (22118400/115200)*64*10*2; //11.1MS to time out
  TIMER1->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
  TIMER_SET_PRESCALE_VALUE(TIMER1, 0);	
	time_out_flag=0;
	NVIC_EnableIRQ(TMR1_IRQn);
	TIMER_Start(TIMER1);	

    /* Enable Timer0 NVIC */
    

	for(temp=0;temp<64;temp++)
	{
        while(UART0->FSR & UART_FSR_RX_EMPTY_Msk)
        {
			   if	(time_out_flag==1)
				  {
					 TIMER_Stop(TIMER1);				
					 NVIC_DisableIRQ(TMR1_IRQn);
	         return FALSE; //false
					}
				}               
        buffer[temp] = UART0->RBR;    /* Get Data from UART RX  */
	}
	TIMER_Stop(TIMER1);				
	NVIC_DisableIRQ(TMR1_IRQn);
	return TRUE; //pass
}


unsigned int UART1WIRE_RX_64_no_timeout(unsigned char *buffer)
{
	unsigned char temp;
	SET_IO_UART_RX();
	

    /* Enable Timer0 NVIC */
    

	for(temp=0;temp<64;temp++)
	{
        while(UART0->FSR & UART_FSR_RX_EMPTY_Msk);              
        buffer[temp] = UART0->RBR;    /* Get Data from UART RX  */
	}
	return TRUE; //pass
}

