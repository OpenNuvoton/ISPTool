/******************************************************************************
 *                                                                            *
 *        Copyright (c) Nuvoton Technology Corp. All rights reserved.         *
 *                                                                            * 
 ******************************************************************************/

// ############################################################################
// Description:		Chip-definition of N571 chip series
//					1. N571P032
// 
// Author:			PatrickHsieh
// Date:			2012/04/20
// ############################################################################
#ifndef __CHIPDEFS_N571_H__
#define __CHIPDEFS_N571_H__

//-----------------------------------------------------------------------------
// [NOTE] System Manager Control Registers
// 
// ----------------------------------------------------------------------------
#define N571_GCR_BA				0x50000000
#define N571_REG_PID			N571_GCR_BA+0x0000                             // (-) [R]	MAP2 (Part_Number) Data Image Register
#define N571_REG_RSTSRC			N571_GCR_BA+0x0004                             // (-) [R/W]	System Reset Source Register
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_IPRST1			N571_GCR_BA+0x0008                             // (-) [R/W] IP Reset Control Register1
#define N571_REG_IPRST1_CHIPRST	(1<<0)
#define N571_REG_IPRST1_CPURST	(1<<1)
#if 0
#define N571_REG_IPRST2			N571_GCR_BA+0x000C                             // (-) [R/W] IP Reset Control Register2
#define N571_REG_IPRST2_ADC		(1<<28)
#define N571_REG_IPRST2_PWM		(1<<20)
#define N571_REG_IPRST2_SPI0	(1<<12)
#define N571_REG_IPRST2_TMRF	(1<<6)
#define N571_REG_IPRST2_APU		(1<<5)
#define N571_REG_IPRST2_TMR2	(1<<4)
#define N571_REG_IPRST2_TMR1	(1<<3)
#define N571_REG_IPRST2_TMR0	(1<<2)
#define N571_REG_IPRST2_GPIO	(1<<1)
#endif
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_LOCKADDR		N571_GCR_BA+0x0100                             // (-) [R/W]	Register Lock Key Address
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// [NOTE] Clock Control Register
// 
// ----------------------------------------------------------------------------
#define N571_CLK_BA				0x50000200
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_PWRCON			N571_CLK_BA+0x0000                             // (-) [R/W]	System Power Down Control Register
#define N571_REG_PWRCON_OSC46M_EN	(1<<2)                                     // (+) Internal 46MHz RC Oscillator Control
#define N571_REG_PWRCON_OSC32K_EN	(1<<1)                                     // (+) External 32.768KHz Crystal Control
#define N571_REG_PWRCON_SLEEP		(1<<0)                                     // (+) OSC32 sleep mode
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_AHBCLK			N571_CLK_BA+0x0004                             // (-) [R/W]	AHB Devices Clock Enable Control Register
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// [NOTE] Device-ID Register
// ----------------------------------------------------------------------------
#define N571_REG_DID			0x500000F4
// ----------------------------------------------------------------------------
// [NOTE] OTP Memory Block
// ----------------------------------------------------------------------------
#define N571_OTP_ADDR			0x00000000

#define N571_CHECKER_ADDR		0x00007FF0
#define N571_CONFIG_ADDR		0x00007FF4
#define N571_MAP1_ADDR			0x00007FF8
#define N571_MAP2_ADDR			0x00007FFC

// $$------------------------------------------------------------------------$$
// [NOTE] Special Handling for [N571E000]
// $$------------------------------------------------------------------------$$
#define N571E000_CONFIG_ADDR	0x5000C100
#define N571E000_MAP1_ADDR		0x5000C104
#define N571E000_MAP2_ADDR		0x5000C108
// $$------------------------------------------------------------------------$$


#define N571_CONFIG_SIZE		4
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_CONFIG_CWDTEN				(1<<31)
#define N571_CONFIG_OTP_SEL				(1<<8)
#define N571_CONFIG_OTP_PCE_MASK		(1<<2)
#define N571_CONFIG_LOCK				(1<<1)
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ----------------------------------------------------------------------------
// [NOTE] BOOT Register
// 
// Description:		For [N571E000], BOOT register is used to configure 
//					external boot.
// 
// Author:			PatrickHsieh
// Date:			2013/02/05
// ----------------------------------------------------------------------------
#define N571_REG_BOOTREG_LOCK	N571_GCR_BA+0x144
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_BOOTREG_LOCK_CHIPSELUNLOCK		(1<<8)
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_BOOTCFG		N571_GCR_BA+0x148
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define N571_REG_BOOTCFG_BOOTSEL	(1<<4)
#define N571_REG_BOOTCFG_BOOTSRC	(1<<3)
#define N571_REG_BOOTCFG_32K_32K	(0x00000007)
#define N571_REG_BOOTCFG_48K_16K	(0x00000006)
#define N571_REG_BOOTCFG_56K_08K	(0x00000005)
#define N571_REG_BOOTCFG_60K_04K	(0x00000004)
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#if 0
#define N571_OTP_PAGESIZE		512                                            // (?) Need to confirm OTP programming
#endif
#define N571_RAM_STARTADDR		0x20000000
#define N571_RAM_MINSIZE		4096
// ----------------------------------------------------------------------------

// ############################################################################
// Description:		Chip-definition about N572F072/P072
// 
// Author:			PatrickHsieh
// Date:			2012/04/20
// ############################################################################
#define N572F072_MAP0_MOTPB				(1<<5)
#define N572F072_MAP0_SOTPB				(1<<4)

#define N572F072_FLASH_CONFIG_CWDTEN	0x80000000UL
#define N572F072_FLASH_CONFIG_CVDEN		0x00800000UL
#define N572F072_FLASH_CONFIG_CVDTV		0x00200000UL
#define N572F072_FLASH_CONFIG_CSPI0_CT	0x000000C0UL
#define N572F072_FLASH_CONFIG_CSPI0_FT	0x00000030UL
#define N572F072_FLASH_CONFIG_PRTB		0x00000004UL
#define N572F072_FLASH_CONFIG_LOCK		0x00000002UL
// ############################################################################

#endif
