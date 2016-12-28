#include <stdio.h>
#include "NUC230_240.h"
#include "ISP_BSP.h"
#include "HID_Transfer_and_MSC.h"
#include "massstorage.h"
void TEST_PIN_INIT(void)
{
	GPIO_SetMode(PE, BIT7, GPIO_PMD_INPUT);
	GPIO_SetMode(PE, BIT8, GPIO_PMD_INPUT);
	GPIO_SetMode(PA, BIT10, GPIO_PMD_INPUT);
	GPIO_SetMode(PA, BIT11, GPIO_PMD_INPUT);
	GPIO_SetMode(PB, BIT7, GPIO_PMD_INPUT);
	GPIO_SetMode(PB, BIT6, GPIO_PMD_INPUT);
	GPIO_SetMode(PD, BIT4, GPIO_PMD_INPUT);
	GPIO_SetMode(PD, BIT5, GPIO_PMD_INPUT);
	GPIO_SetMode(PC, BIT7, GPIO_PMD_INPUT);
	GPIO_SetMode(PC, BIT6, GPIO_PMD_INPUT);
	GPIO_SetMode(PB, BIT15, GPIO_PMD_INPUT);
	GPIO_SetMode(PB, BIT13, GPIO_PMD_INPUT);
}

void IO_TEST_MODE(void)
{
if (T1!=1)
   goto FALSE_LED_FLUSH;
if (T2!=1)
   goto FALSE_LED_FLUSH;
if (T3!=1)
   goto FALSE_LED_FLUSH;
if (T4!=1)
   goto FALSE_LED_FLUSH;
if (T5!=1)
   goto FALSE_LED_FLUSH;
if (T6!=1)
   goto FALSE_LED_FLUSH;
if (T7!=1)
   goto FALSE_LED_FLUSH;
if (T8!=1)
   goto FALSE_LED_FLUSH;
if (T9!=1)
   goto FALSE_LED_FLUSH;
if (T10!=0)
   goto FALSE_LED_FLUSH;
PB0=0;
PB1=0;
PA9=0;
PA8=0;
PD3=0;
PD2=0;
PD1=0;
PD0=0;
if (T1!=0)
   goto FALSE_LED_FLUSH;
if (T2!=0)
   goto FALSE_LED_FLUSH;
if (T3!=0)
   goto FALSE_LED_FLUSH;
if (T4!=0)
   goto FALSE_LED_FLUSH;
if (T5!=0)
   goto FALSE_LED_FLUSH;
if (T6!=0)
   goto FALSE_LED_FLUSH;
if (T7!=0)
   goto FALSE_LED_FLUSH;
if (T8!=0)
   goto FALSE_LED_FLUSH;
if (T9!=0)
   goto FALSE_LED_FLUSH;
if (T10!=1)
   goto FALSE_LED_FLUSH;

PB0=1;
PB1=1;
PA9=1;
PA8=1;
PD3=1;
PD2=1;
PD1=1;
PD0=1;

return;

FALSE_LED_FLUSH:
   while(1)
	 {
	Buzzer_FLASE_LED();
	 
	 }
}

//SYSTEM CLOCK
void ISP_BSP_SYSTEM_CLOCK_INITIAL(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    GPIO_SetMode(PA, BIT4, GPIO_PMD_OUTPUT); //for tvcc power control
    GPIO_SetMode(PB, BIT8, GPIO_PMD_OUTPUT);//for buzzer
    /* Lock protected registers */
    SYS_LockReg();
}


//RTC
void RTC_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12 MHz XTAL, 32 kHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL32K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL32K_STB_Msk);

    /* Enable peripheral clock */
    CLK->APBCLK |= CLK_APBCLK_RTC_EN_Msk;

    /* lock protected registers */
    SYS_LockReg();
}

 
void RTC_SET(void)
{
	  S_RTC_TIME_DATA_T sWriteRTC;
 
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 2;
    sWriteRTC.u32Day        = 6;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 30;
    sWriteRTC.u32Second     = 55;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
	
    /* Unlock protected registers */
    SYS_UnlockReg();
    RTC_Open(&sWriteRTC);
	  RTC_SetTickPeriod(RTC_TICK_1_SEC);
	  NVIC_EnableIRQ(RTC_IRQn);
    RTC_EnableInt(RTC_RIER_TIER_Msk);
	  SYS_LockReg();
}


void Initial_USB(void)
{
    CLK_EnableModuleClock(USBD_MODULE);
	  CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));
	 USBD_Open(&gsInfo, HID_MSC_ClassRequest, NULL);

USBD_SetConfigCallback(MSC_SetConfig);

	
    /* Endpoint configuration */
    HID_MSC_Init();

    /* Start USB device */
    USBD_Start();

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);
}



