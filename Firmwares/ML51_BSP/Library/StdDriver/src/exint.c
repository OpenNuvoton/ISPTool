/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


#include "ML51.h"

void ExternPinInterrupt_Enable( unsigned char u8EXNum, unsigned char u8EXTTRG, unsigned char u8EXINT)
{
		switch (u8EXNum)
		{
			case INT0:
				switch (u8EXTTRG)
				{
						case Level: clr_TCON_IT0; break;
						case Edge:  set_TCON_IT0; break;
				}
				switch (u8EXINT)
				{
						case Disable: clr_IE_EX0; break;
						case Enable:  set_IE_EX0;break;
				}
			break;
				
			case INT1:
				switch (u8EXTTRG)
				{
						case Level: clr_TCON_IT0; break;
						case Edge:  set_TCON_IT0; break;
				}
				switch (u8EXINT)
				{
						case Disable: clr_IE_EX1; break;
						case Enable:  set_IE_EX1;break;
				}
			break;
		}
}
