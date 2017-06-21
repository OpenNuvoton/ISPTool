/**************************************************************************//**
 * @file     NuEdu-NuEdu-Basic01_UART.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/15 5:11p $
 * @brief    NuEdu-Basic01_UART driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_UART.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS UART Functions
    @{
*/


/**
 * @brief       Initiate UART0 as 8-N-1 and 115200 BPS
 * @return      None
 */
void UART0_Init(void)
{
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HIRC,CLK_UART_CLK_DIVIDER(1));

    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_RX | SYS_PB_L_MFP_PB1_MFP_UART0_TX );

    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    //UART_Open(UART0, 115200);

    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_HXT;  /* Select 12 Mhz XTAL */

    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */

}

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
