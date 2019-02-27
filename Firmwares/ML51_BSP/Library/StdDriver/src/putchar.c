/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2018
//***********************************************************************************************************

#include "ML51.h"

/**
 * @brief       For printf function define  
 *
 * @param       none 
 *
 * @return      none
 *
 * @details     UART0 as printf use "#if 1", UART1 as printf output use "#if 0"
 */
 
 /**
* if use UART0 as printf source, enable following part
*/

#if 0
char putchar (char c)
{
		while (!TI);  /* wait until transmitter ready */
		TI = 0;
		SBUF = c;      /* output character */
		return (c);
}
#endif

/**
* if use UART1 as printf source, enable following part
*/
#if 1
char putchar (char c)
{
		while (!TI_1);  /* wait until transmitter ready */
		TI_1 = 0;
		SBUF1 = c;      /* output character */
		return (c);
}
#endif