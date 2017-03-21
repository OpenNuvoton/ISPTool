#ifndef __CLK_HIRC_H__
#define __CLK_HIRC_H__
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  PLLCON (PLLCTL) constant definitions can be found in CLK.h                                             */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(CLK_PLLCON_72MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCON_72MHz_HIRC
    #define PLL_CLOCK       		72000000
#elif defined(CLK_PLLCON_60MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCON_60MHz_HIRC
    #define PLL_CLOCK       		60000000
#elif defined(CLK_PLLCON_50MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCON_50MHz_HIRC
    #define PLL_CLOCK       		50000000
#elif defined(CLK_PLLCTL_50MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCTL_50MHz_HIRC
    #define PLL_CLOCK       		50000000
#elif defined(CLK_PLLCTL_100MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCTL_100MHz_HIRC
    #define PLL_CLOCK       		100000000
    #define HCLK_DIV 						2
#elif defined(CLK_PLLCTL_42MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCTL_42MHz_HIRC
    #define PLL_CLOCK       		42000000
#elif defined(CLK_PLLCTL_32MHz_HIRC)
    #define PLLCON_SETTING  		CLK_PLLCTL_32MHz_HIRC
    #define PLL_CLOCK       		32000000
#elif defined(PLL_IN_12M_OUT_96M_HIRC)
    #define PLLCON_SETTING  		PLL_IN_12M_OUT_96M_HIRC
    #define PLL_CLOCK       		96000000
    #define HCLK_DIV 						3
#endif

#ifndef HCLK_DIV
    #define HCLK_DIV 						1
#endif

#endif  /* __CLK_HIRC_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
