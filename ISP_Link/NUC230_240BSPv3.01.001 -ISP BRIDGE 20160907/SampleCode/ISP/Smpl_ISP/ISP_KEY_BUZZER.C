#include "NUC230_240.h"
#include "ISP_BSP.h"
//KEY
void Initial_Key_Input(void)
{
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);//KEY1
    GPIO_SetMode(PB, BIT9,  GPIO_PMD_INPUT);//KEY2
    GPIO_SetMode(PB, BIT10, GPIO_PMD_INPUT);//KEY3
    GPIO_SetMode(PB, BIT11, GPIO_PMD_INPUT);//KEY4
}


unsigned char Get_Key_Input(void)
{
    unsigned char temp = 0;
    if(KEY1 == 0)
        temp |= 0x1;


    if(KEY2 == 0)
        temp |= 0x2;


    if(KEY3 == 0)
        temp |= 0x4;


    if(KEY4 == 0)
        temp |= 0x8;

    return   temp;
}


void Open_EINT0_KEY1(void)
{
      /* Set PB.14 as Input */
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);

    /* Set PB.14 multi-function pins for EINT0 */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB14_INT0;
    /* Enable interrupt by falling edge trigger */
    GPIO_EnableInt(PB, 14, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

}

void EINT0_IRQHandler(void)
{
    /* For PB.14, clear the INT flag */
    GPIO_CLR_INT_FLAG(PB, BIT14);

}

void Buzzer(void)
{
unsigned i;
	//600ms delay
    for(i=0;i<600;i++)
	{
		CLK_SysTickDelay(1000);
	}
	Buzzer_ON;
	
 //600ms delay	
	for(i=0;i<600;i++)
	{
		CLK_SysTickDelay(1000);
	}
	Buzzer_OFF;
}


void Buzzer_FLASE_LED(void)
{
unsigned i;
	//600ms delay
    for(i=0;i<600;i++)
	{
		CLK_SysTickDelay(1000);
	}
	//Buzzer_ON;
	LED_FALSE_ON;
	
 //600ms delay	
	for(i=0;i<600;i++)
	{
		CLK_SysTickDelay(1000);
	}
	//Buzzer_OFF;
	LED_FALSE_OFF;
}

