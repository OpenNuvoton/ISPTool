/**************************************************************************//**
 * @file     NuEdu-Basic01_7_Segment.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 14/10/17 2:50p $
 * @brief    NuEdu-Basic01 7_Segment LED driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_7_Segment.h"
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Seven Segment LED Functions
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Porting Define                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define SEG_A_ON    PA2=0                                       /*!< Turn on segment A */
#define SEG_B_ON    PA3=0                                       /*!< Turn on segment B */
#define SEG_C_ON    PA4=0                                       /*!< Turn on segment C */
#define SEG_D_ON    PA5=0                                       /*!< Turn on segment D */
#define SEG_E_ON    PA6=0                                       /*!< Turn on segment E */
#define SEG_F_ON    PA7=0                                       /*!< Turn on segment F */
#define SEG_G_ON    PD6=0                                       /*!< Turn on segment G */
#define SEG_H_ON    PD7=0                                       /*!< Turn on segment H */
#define SEG_CONTROL1_ON    PD14=1                       /*!< Turn on 7_segment1 */
#define SEG_CONTROL2_ON    PD15=1                       /*!< Turn on 7_segment2 */

#define SEG_A_OFF   PA2=1                                       /*!< Turn off segment A */
#define SEG_B_OFF   PA3=1                                       /*!< Turn off segment B */
#define SEG_C_OFF   PA4=1                                       /*!< Turn off segment C */
#define SEG_D_OFF   PA5=1                                       /*!< Turn off segment D */
#define SEG_E_OFF   PA6=1                                       /*!< Turn off segment E */
#define SEG_F_OFF   PA7=1                                       /*!< Turn off segment F */
#define SEG_G_OFF   PD6=1                                       /*!< Turn off segment G */
#define SEG_H_OFF   PD7=1                                       /*!< Turn off segment H */
#define SEG_CONTROL1_OFF    PD14=0                  /*!< Turn off 7_segment1 */
#define SEG_CONTROL2_OFF    PD15=0                  /*!< Turn off 7_segment2 */

/**
 * @brief       Set 7_Segment LED GPIO to output mode
 * @return      None
 */
void Initial_SEG_GPIO(void)
{
    //io initail output mode

    GPIO_SetMode(PA, BIT2, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT4, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT6, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT7, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT7, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT14, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT15, GPIO_PMD_OUTPUT);
}

/**
 * @brief       This function turn on 7_Segment LED
 * @return      None
 */
void Open_Seven_Segment(void)
{
    Initial_SEG_GPIO();
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
}

/**
 * @brief       This function turn off 7_Segment LED
 * @return      None
 */
void Close_Seven_Segment(void)
{
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
    SEG_CONTROL1_OFF;
    SEG_CONTROL2_OFF;
}

/**
 * @brief       This function turn on 7_Segment LED to show numbers
 *
 * @param[in]   no        Turn on which 7_Segment LED
 * @param[in]   number    7_Segment LED show number 0~9
 *
 * @return      None
 */
void Show_Seven_Segment(unsigned char no, unsigned char number)
{
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
    SEG_CONTROL1_OFF;
    SEG_CONTROL2_OFF;

    switch(number) {
    case 1:
        SEG_CONTROL1_ON;
        break;

    case 2:
        SEG_CONTROL2_ON;
        break;
    }

    switch(no) {
    //show 0
    case 0:
        SEG_A_ON;
        SEG_B_ON;
        SEG_C_ON;
        SEG_D_ON;
        SEG_E_ON;
        SEG_F_ON;
        break;

    //show 1
    case 1:
        SEG_B_ON;
        SEG_C_ON;
        break;

    //show 2
    case 2:
        SEG_A_ON;
        SEG_B_ON;
        SEG_G_ON;
        SEG_E_ON;
        SEG_D_ON;
        break;

    //show 3
    case 3:
        SEG_A_ON;
        SEG_B_ON;
        SEG_G_ON;
        SEG_C_ON;
        SEG_D_ON;
        break;

    //show 4
    case 4:
        SEG_F_ON;
        SEG_B_ON;
        SEG_G_ON;
        SEG_C_ON;
        break;

    //show 5
    case 5:
        SEG_A_ON;
        SEG_F_ON;
        SEG_G_ON;
        SEG_C_ON;
        SEG_D_ON;
        break;

    //show 6
    case 6:
        SEG_A_ON;
        SEG_F_ON;
        SEG_E_ON;
        SEG_G_ON;
        SEG_C_ON;
        SEG_D_ON;
        break;

    //show 7
    case 7:
        SEG_A_ON;
        SEG_B_ON;
        SEG_C_ON;
        SEG_F_ON;
        break;

    //show 8
    case 8:
        SEG_A_ON;
        SEG_B_ON;
        SEG_C_ON;
        SEG_D_ON;
        SEG_E_ON;
        SEG_F_ON;
        SEG_G_ON;
        break;

    //show 9
    case 9:
        SEG_A_ON;
        SEG_B_ON;
        SEG_C_ON;
        SEG_F_ON;
        SEG_G_ON;
        break;
    }
}
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
