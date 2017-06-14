#ifndef __CLK_HXT_H__
#define __CLK_HXT_H__
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  PLLCON (PLLCTL) constant definitions can be found in CLK.h                                             */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(CLK_PLLCTL_192MHz_HXT)
    #define PLLCON_SETTING  		CLK_PLLCTL_192MHz_HXT
    #define PLL_CLOCK       		192000000
    #define HCLK_DIV 						2
    #define USBD_DIV 						4
#elif defined(CLK_PLLCON_144MHz_HXT)
    #define PLLCON_SETTING  		CLK_PLLCON_144MHz_HXT
    #define PLL_CLOCK       		144000000
    #define HCLK_DIV 						2
    #define USBD_DIV 						3
#elif defined(CLK_PLLCTL_144MHz_HXT)
    #define PLLCON_SETTING  		CLK_PLLCTL_144MHz_HXT
    #define PLL_CLOCK       		144000000
    #define HCLK_DIV 						2
    #define USBD_DIV 						3
#elif defined(CLK_PLLCON_48MHz_HXT)
    #define PLLCON_SETTING  		CLK_PLLCON_48MHz_HXT
    #define PLL_CLOCK       		48000000
    #define HCLK_DIV 						1
    #define USBD_DIV 						1
#elif defined(PLL_IN_12M_OUT_96M_HXT)
    #define PLLCON_SETTING  		PLL_IN_12M_OUT_96M_HXT
    #define PLL_CLOCK       		96000000
    #define HCLK_DIV 						3
    #define USBD_DIV 						2
#elif defined(CLK_PLLCON_48MHz_HXT)
    #define PLLCON_SETTING  		CLK_PLLCON_48MHz_HXT
    #define PLL_CLOCK       		48000000
    #define HCLK_DIV 						1
    #define USBD_DIV 						1
#endif




#endif  /* __CLK_HXT_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
