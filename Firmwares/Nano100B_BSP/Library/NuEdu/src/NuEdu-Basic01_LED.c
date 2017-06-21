/**************************************************************************//**
 * @file     NuEdu-NuEdu-Basic01_LED.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/12 11:15p $
 * @brief    NuEdu-Basic01_LED driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_LED.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Buzzer Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Porting Define                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define LED1_ON  PB0=0      //control LED1 on
#define LED2_ON  PB1=0      //control LED2 on
#define LED3_ON  PE9=0      //control LED3 on
#define LED4_ON  PE10=0     //control LED4 on
#define LED5_ON  PE11=0     //control LED5 on
#define LED6_ON  PD8=0      //control LED6 on
#define LED7_ON  PD9=0      //control LED7 on
#define LED8_ON  PC7=0      //control LED8 on

#define LED1_OFF  PB0=1     //control LED1 off
#define LED2_OFF  PB1=1     //control LED2 off
#define LED3_OFF  PE9=1     //control LED3 off
#define LED4_OFF  PE10=1    //control LED4 off
#define LED5_OFF  PE11=1    //control LED5 off
#define LED6_OFF  PD8=1     //control LED6 off
#define LED7_OFF  PD9=1     //control LED7 off
#define LED8_OFF  PC7=1     //control LED8 off


/**
 * @brief       Set LED GPIO to output mode
 * @return      None
 */
void initial_led(void)
{
    GPIO_SetMode(PB, BIT0, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PB, BIT1, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PE, BIT9, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PE, BIT10, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PE, BIT11, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT8, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT9, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT7, GPIO_PMD_OUTPUT);

}


/**
* @brief       This function turn on LED numbers to open or close
*
* @param[in]   temp        Turn on which LED open number
*
* @return      None
*/
void LED_on(unsigned int temp)
{
    if((temp&1)!=1)
        LED1_OFF;
    else
        LED1_ON;

    temp=temp>>1;

    if((temp&1)!=1)
        LED2_OFF;
    else
        LED2_ON;

    temp=temp>>1;
    if((temp&1)!=1)
        LED3_OFF;
    else
        LED3_ON;

    temp=temp>>1;
    if((temp&1)!=1)
        LED4_OFF;
    else
        LED4_ON;

    temp=temp>>1;
    if((temp&1)!=1)
        LED5_OFF;
    else
        LED5_ON;

    temp=temp>>1;
    if((temp&1)!=1)
        LED6_OFF;
    else
        LED6_ON;

    temp=temp>>1;
    if((temp&1)!=1)
        LED7_OFF;
    else
        LED7_ON;

    temp=temp>>1;
    if((temp&1)!=1)
        LED8_OFF;
    else
        LED8_ON;

}

/**
 * @brief       Set LED GPIO to input mode
 * @return      None
 */
void initial_close(void)
{

    GPIO_SetMode(PB, BIT0, GPIO_PMD_INPUT);  //LED1
    GPIO_SetMode(PB, BIT1, GPIO_PMD_INPUT);
    GPIO_SetMode(PE, BIT9, GPIO_PMD_INPUT);
    GPIO_SetMode(PE, BIT10, GPIO_PMD_INPUT);
    GPIO_SetMode(PE, BIT11, GPIO_PMD_INPUT);
    GPIO_SetMode(PD, BIT8, GPIO_PMD_INPUT);
    GPIO_SetMode(PD, BIT9, GPIO_PMD_INPUT);
    GPIO_SetMode(PC, BIT7, GPIO_PMD_INPUT);

}

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Buzzer Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */
/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
