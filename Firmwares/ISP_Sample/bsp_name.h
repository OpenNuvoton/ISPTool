/**************************************************************************//**
 * @file     bsp_name.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/12/21 5:27p $
 * @brief    Solve Naming Conflicts in peripheral access layer header files and driver header files.
 *
 * @note
 * Copyright (C) 2016~2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
 
#ifndef __BSP_NAME_H__
#define __BSP_NAME_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Nano103 used MIRC as hclk clock source, the others chips used HIRC as clock source.                     */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(TARGET_NANO103)
    // There are three IRC options for Nano103 series, which are HIRC0, HIRC1, MIRC
    #define CLK_PWRCON_OSC22M_EN_Msk     CLK_PWRCTL_MIRCEN_Msk
    #define CLK_CLKSTATUS_HIRC_STB_Msk					CLK_STATUS_MIRCSTB_Msk
    #define PLL_CLOCK       							36000000
    #define CLK_CLKSEL1_UART_S_HIRC						CLK_CLKSEL1_UART0SEL_MIRC
#endif

#if defined(__HIRC)
    // No Conflicts
#elif defined(__IRC22M)
    #define __HIRC		__IRC22M
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* System Clock Controller(CLK)                                                                            */
/*     -->  CLK->PWRCON    == CLK->PWRCTL                                                                  */
/*     -->  CLK->CLKSTATUS == CLK->STATUS                                                                  */
/*     -->  CLK->CLKDIV    == CLK->CLKDIV0                                                                 */
/*     -->  CLK->PLLCON    == CLK->PLLCTL                                                                  */
/*     -->  CLK->APBCLK    == CLK->APBCLK0                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(CLK_PWRCTL_HXTEN_Msk) || defined(CLK_PWRCTL_XTLEN_Msk) || defined(CLK_PWRCTL_HXT_EN_Msk)
    #define PWRCON 										PWRCTL
#endif

#if defined(CLK_STATUS_HIRCSTB_Msk) || defined(CLK_STATUS_MIRCSTB_Msk) || defined(CLK_STATUS_PLLSTB_Msk)
    #define CLKSTATUS 									STATUS
#endif

#if defined(CLK_CLKDIV0_HCLK_N_Msk) || defined(CLK_CLKDIV0_HCLKDIV_Msk)
    #define CLKDIV										CLKDIV0
#endif


#if defined(CLK_PLLCTL_100MHz_HIRC) || defined(CLK_PLLCTL_42MHz_HIRC) || defined(CLK_PLLCTL_32MHz_HIRC)
    #define PLLCON 										PLLCTL
#elif defined(CLK_PLLCTL_PLL_SRC_HIRC) || defined(CLK_PLLCTL_PLL_SRC_HXT)
    #define PLLCON 										PLLCTL
#elif defined(CLK_PLLCTL_144MHz_HXT)
    #define PLLCON 										PLLCTL
#endif


#if defined(CLK_APBCLK0_UART0CKEN_Msk) || defined(CLK_APBCLK0_USBDCKEN_Msk)
    #define APBCLK 										APBCLK0
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* CLK Bit Field Definition                                                                                */
/*     -->     CLK_PWRCON_XTL12M_EN_Msk                                                                    */
/*          == CLK_PWRCTL_HXTEN_Msk                                                                        */
/*          == CLK_PWRCON_XTLCLK_EN_Msk                                                                    */
/*          == CLK_PWRCTL_XTLEN_Msk                                                                        */
/*          == CLK_PWRCTL_HXT_EN_Msk                                                                       */
/*                                                                                                         */
/*     -->     CLK_PWRCON_OSC22M_EN_Msk                                                                    */
/*          == CLK_PWRCTL_HIRCEN_Msk                                                                       */
/*          == CLK_PWRCTL_HIRC_EN_Msk                                                                      */
/*                                                                                                         */
/*     -->     CLK_CLKSTATUS_HIRC_STB_Msk                                                                  */
/*          == CLK_CLKSTATUS_IRC22M_STB_Msk                                                                */
/*          == CLK_STATUS_HIRCSTB_Msk                                                                      */
/*                                                                                                         */
/*     -->     CLK_CLKSTATUS_PLL_STB_Msk                                                                   */
/*          == CLK_STATUS_PLLSTB_Msk                                                                       */
/*                                                                                                         */
/*     -->     CLK_CLKSEL0_HCLK_S_Msk                                                                      */
/*          == CLK_CLKSEL0_HCLKSEL_Msk                                                                     */
/*                                                                                                         */
/*     -->     CLK_CLKSEL0_HCLK_S_PLL                                                                      */
/*          == CLK_CLKSEL0_HCLKSEL_PLL                                                                     */
/*                                                                                                         */
/*     -->     CLK_CLKDIV_HCLK_N_Msk                                                                       */
/*          == CLK_CLKDIV_HCLKDIV_Msk                                                                      */
/*          == CLK_CLKDIV0_HCLK_N_Msk                                                                      */
/*                                                                                                         */
/*     -->     CLK_CLKDIV_HCLK                                                                             */
/*          == CLK_HCLK_CLK_DIVIDER                                                                        */
/*                                                                                                         */
/*     -->     CLK_APBCLK_UART0_EN_Msk                                                                     */
/*          == CLK_APBCLK_UART_EN_Msk                                                                      */
/*          == CLK_APBCLK_UART0CKEN_Msk                                                                    */
/*          == CLK_APBCLK0_UART0CKEN_Msk                                                                   */
/*                                                                                                         */
/*     -->     CLK_CLKSEL1_UART_S_Msk                                                                      */
/*          == CLK_CLKSEL1_UARTSEL_Msk                                                                     */
/*          == CLK_CLKSEL1_UART0SEL_Msk                                                                    */
/*                                                                                                         */
/*     -->     CLK_CLKSEL1_UART_S_HIRC                                                                     */
/*          == CLK_CLKSEL1_UARTSEL_HIRC                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

// HXT: Bit0 (or Bit0~1 for Mini51 and Mini58)
#if defined(CLK_PWRCON_XTL12M_EN_Msk)
    // No Conflicts
#elif defined(CLK_PWRCTL_HXTEN_Msk)
    #define CLK_PWRCON_XTL12M_EN_Msk		CLK_PWRCTL_HXTEN_Msk
#elif defined(CLK_PWRCON_XTLCLK_EN_Msk)
    #define CLK_PWRCON_XTL12M_EN_Msk		CLK_PWRCON_XTLCLK_EN_Msk
#elif defined(CLK_PWRCTL_XTLEN_Msk)
    #define CLK_PWRCON_XTL12M_EN_Msk		CLK_PWRCTL_XTLEN_Msk
#elif defined(CLK_PWRCTL_HXT_EN_Msk)
    #define CLK_PWRCON_XTL12M_EN_Msk		CLK_PWRCTL_HXT_EN_Msk
#else
    //# error "Fail to find external clock enable bit field definition."
    # error "CLK_PWRCON_XTL12M_EN_Msk is not defined."
#endif

// HIRC: Bit2
#if defined(CLK_PWRCON_OSC22M_EN_Msk)
    // No Conflicts
#elif defined(CLK_PWRCTL_HIRCEN_Msk)
    #define CLK_PWRCON_OSC22M_EN_Msk			CLK_PWRCTL_HIRCEN_Msk
#elif defined(CLK_PWRCTL_HIRC_EN_Msk)
    #define CLK_PWRCON_OSC22M_EN_Msk			CLK_PWRCTL_HIRC_EN_Msk
#else
    // # error "Fail to find internal clock enable bit field definition."
    # error "CLK_PWRCON_OSC22M_EN_Msk is not defined."
#endif

#if defined(CLK_PWRCON_XTL12M_EN_Msk) && defined(CLK_PWRCON_OSC22M_EN_Msk)
    #define PWRCON_SETTING (CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk)
#endif



// Bit0
#if defined(CLK_CLKSTATUS_HXT_STB_Msk)
    // No Conflicts
#elif defined(CLK_STATUS_HXTSTB_Msk)
    #define CLK_CLKSTATUS_HXT_STB_Msk			CLK_STATUS_HXTSTB_Msk
#elif defined(CLK_CLKSTATUS_XTL12M_STB_Msk)
    #define CLK_CLKSTATUS_HXT_STB_Msk			CLK_CLKSTATUS_XTL12M_STB_Msk
#elif defined(CLK_STATUS_XTLSTB_Msk)
    #define CLK_CLKSTATUS_HXT_STB_Msk			CLK_STATUS_XTLSTB_Msk
#else
    # error "Fail to find external clock stable bit field definition."
#endif


// Bit4
#if defined(CLK_CLKSTATUS_HIRC_STB_Msk)
    // No Conflicts
#elif defined(CLK_CLKSTATUS_IRC22M_STB_Msk)
    #define CLK_CLKSTATUS_HIRC_STB_Msk		CLK_CLKSTATUS_IRC22M_STB_Msk
#elif defined(CLK_STATUS_HIRCSTB_Msk)
    #define CLK_CLKSTATUS_HIRC_STB_Msk		CLK_STATUS_HIRCSTB_Msk
#else
    # error "Fail to find internal clock stable bit field definition."
#endif

#if defined(CLK_CLKSTATUS_PLL_STB_Msk)
    // No Conflicts
#elif defined(CLK_STATUS_PLLSTB_Msk)
    #define CLK_CLKSTATUS_PLL_STB_Msk		CLK_STATUS_PLLSTB_Msk
#endif

#if defined(CLK_CLKSEL0_HCLK_S_Msk)
    // No Conflicts
#elif defined(CLK_CLKSEL0_HCLKSEL_Msk)
    #define CLK_CLKSEL0_HCLK_S_Msk		CLK_CLKSEL0_HCLKSEL_Msk
#endif

#if defined(CLK_CLKSEL0_HCLK_S_PLL)
    // No Conflicts
#elif defined(CLK_CLKSEL0_HCLKSEL_PLL)
    #define CLK_CLKSEL0_HCLK_S_PLL		CLK_CLKSEL0_HCLKSEL_PLL
#endif

#if defined(CLK_CLKDIV_HCLK_N_Msk)
    // No Conflicts
#elif defined(CLK_CLKDIV_HCLKDIV_Msk)
    #define CLK_CLKDIV_HCLK_N_Msk		CLK_CLKDIV_HCLKDIV_Msk
#elif defined(CLK_CLKDIV0_HCLK_N_Msk)
    #define CLK_CLKDIV_HCLK_N_Msk		CLK_CLKDIV0_HCLK_N_Msk
		#elif defined(CLK_CLKDIV0_HCLKDIV_Msk)
    #define CLK_CLKDIV_HCLK_N_Msk		CLK_CLKDIV0_HCLKDIV_Msk
#endif

#if defined(CLK_CLKDIV_USB_N_Msk)
    // No Conflicts
#elif defined(CLK_CLKDIV0_USB_N_Msk)
    #define CLK_CLKDIV_USB_N_Msk		CLK_CLKDIV0_USB_N_Msk
#elif defined(CLK_CLKDIV0_USBDIV_Msk)
    #define CLK_CLKDIV_USB_N_Msk		CLK_CLKDIV0_USBDIV_Msk
#endif

#if defined(CLK_CLKDIV_USB)
    // No Conflicts
#elif defined(CLK_USB_CLK_DIVIDER)
    #define CLK_CLKDIV_USB		CLK_USB_CLK_DIVIDER
#elif defined(CLK_CLKDIV0_USB)
    #define CLK_CLKDIV_USB		CLK_CLKDIV0_USB
#endif


#if defined(CLK_CLKDIV_HCLK)
    // No Conflicts
#elif defined(CLK_HCLK_CLK_DIVIDER)
    #define CLK_CLKDIV_HCLK		CLK_HCLK_CLK_DIVIDER
#elif defined(CLK_CLKDIV0_HCLK)
    #define CLK_CLKDIV_HCLK		CLK_CLKDIV0_HCLK
#endif

#if defined CLK_APBCLK_UART0_EN_Msk
    // No Conflicts
#elif defined(CLK_APBCLK_UART_EN_Msk)
    #define CLK_APBCLK_UART0_EN_Msk			CLK_APBCLK_UART_EN_Msk
#elif defined(CLK_APBCLK_UART0CKEN_Msk)
    #define CLK_APBCLK_UART0_EN_Msk			CLK_APBCLK_UART0CKEN_Msk
#elif defined(CLK_APBCLK0_UART0CKEN_Msk)
    #define CLK_APBCLK_UART0_EN_Msk			CLK_APBCLK0_UART0CKEN_Msk
#endif

#if defined(CLK_CLKSEL1_UART_S_Msk)
    // No Conflicts
#elif defined(CLK_CLKSEL1_UARTSEL_Msk)
    #define CLK_CLKSEL1_UART_S_Msk			CLK_CLKSEL1_UARTSEL_Msk
#elif defined(CLK_CLKSEL1_UART0SEL_Msk)
    #define CLK_CLKSEL1_UART_S_Msk			CLK_CLKSEL1_UART0SEL_Msk
#endif

#if defined(CLK_CLKSEL1_UART_S_HIRC)
    // No Conflicts
#elif defined(CLK_CLKSEL1_UARTSEL_HIRC)
    #define CLK_CLKSEL1_UART_S_HIRC			CLK_CLKSEL1_UARTSEL_HIRC
#elif defined(CLK_CLKSEL1_UART0SEL_HIRC)
    #define CLK_CLKSEL1_UART_S_HIRC			CLK_CLKSEL1_UART0SEL_HIRC
#endif

// Bit2
#ifdef CLK_AHBCLK_ISP_EN_Msk
    // No Conflicts
#elif defined(CLK_AHBCLK_ISPCKEN_Msk)
    #define CLK_AHBCLK_ISP_EN_Msk				CLK_AHBCLK_ISPCKEN_Msk
#endif

// Bit27
#ifdef CLK_APBCLK_USBD_EN_Msk
    // No Conflicts
#elif defined(CLK_APBCLK0_USBDCKEN_Msk)
    #define CLK_APBCLK_USBD_EN_Msk			CLK_APBCLK0_USBDCKEN_Msk
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* System Global Control Registers(SYS)                                                                    */
/*     -->  SYS->RSTSRC    == SYS->RSTSTS                                                                  */
/*                         == SYS->RST_SRC                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(SYS_RSTSTS_PORF_Msk) || defined(SYS_RSTSTS_PINRF_Msk)
    #define RSTSRC 							RSTSTS
#elif defined(SYS_RST_SRC_RSTS_POR_Msk) || defined(SYS_RST_SRC_RSTS_PAD_Msk)
    #define RSTSRC 							RST_SRC
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* SYS Bit Field Definition                                                                                */
/*     -->     SYS_RSTSRC_RSTS_POR_Msk                                                                     */
/*          == SYS_RSTSTS_PORF_Msk                                                                         */
/*          == SYS_RST_SRC_RSTS_POR_Msk                                                                    */
/*                                                                                                         */
/*     -->     SYS_RSTSRC_RSTS_RESET_Msk                                                                   */
/*          == SYS_RSTSTS_PINRF_Msk                                                                        */
/*          == SYS_RST_SRC_RSTS_PAD_Msk                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(SYS_RSTSTS_PORF_Msk) && defined(SYS_RSTSTS_PINRF_Msk)
    #define SYS_RSTSRC_RSTS_POR_Msk			SYS_RSTSTS_PORF_Msk
    #define SYS_RSTSRC_RSTS_RESET_Msk		SYS_RSTSTS_PINRF_Msk
#elif defined(SYS_RST_SRC_RSTS_POR_Msk) && defined(SYS_RST_SRC_RSTS_PAD_Msk)
    #define SYS_RSTSRC_RSTS_POR_Msk			SYS_RST_SRC_RSTS_POR_Msk
    #define SYS_RSTSRC_RSTS_RESET_Msk		SYS_RST_SRC_RSTS_PAD_Msk
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Flash Memory Controller(FMC)                                                                            */
/*     -->  FMC->ISPCON    == FMC->ISPCTL                                                                  */
/*     -->  FMC->ISPADR    == FMC->ISPADDR                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(FMC_ISPCTL_ISPEN_Msk) || defined(FMC_ISPCTL_ISPFF_Msk) || defined(FMC_ISPCTL_BS_Msk)
    #define ISPCON 							ISPCTL
#endif

#if defined(FMC_ISPADDR_ISPADDR_Msk)
    #define ISPADR 							ISPADDR
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* FMC Bit Field Definition                                                                                */
/*     -->     FMC_ISPCON_ISPEN_Msk                                                                        */
/*          == FMC_ISPCTL_ISPEN_Msk                                                                        */
/*                                                                                                         */
/*     -->     FMC_ISPCON_ISPFF_Msk                                                                        */
/*          == FMC_ISPCTL_ISPFF_Msk                                                                        */
/*                                                                                                         */
/*     -->     FMC_ISPCON_BS_Msk                                                                           */
/*          == FMC_ISPCTL_BS_Msk                                                                           */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


#if defined(FMC_ISPCTL_ISPEN_Msk) && defined(FMC_ISPCTL_ISPFF_Msk) && defined(FMC_ISPCTL_BS_Msk)
    #define FMC_ISPCON_ISPEN_Msk			FMC_ISPCTL_ISPEN_Msk
    #define FMC_ISPCON_ISPFF_Msk			FMC_ISPCTL_ISPFF_Msk
    #define FMC_ISPCON_BS_Msk				FMC_ISPCTL_BS_Msk
#endif

#if defined(FMC_ISPCMD_WRITE)
    #define FMC_ISPCMD_PROGRAM				FMC_ISPCMD_WRITE
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Universal Asynchronous Receiver/Transmitter Controller(UART)                                            */
/*     -->  UART->IER       == UART->INTEN                                                                 */
/*     -->  UART->ISPADR    == UART->ISPADDR                                                               */
/*     -->  UART->ISR       == UART->INTSTS                                                                */
/*     -->  UART->FCR       == UART->FIFO                                                                  */
/*     -->  UART->FSR       == UART->FIFOSTS                                                               */
/*     -->  UART->RBR       == UART->DAT                                                                   */
/*     -->  UART->THR       == UART->DAT                                                                   */
/*     -->  UART->LCR       == UART->LINE                                                                  */
/*     -->  UART->TLCTL     == UART->LINE   (for Nano103 Only)                                             */
/*     -->  UART->FUN_SEL   == UART->FUNCSEL                                                               */
/*                          == UART->FUNSEL                                                                */
/*     -->  UART->TOR       == UART->TOUT                                                                  */
/*---------------------------------------------------------------------------------------------------------*/


#if defined(UART_INTEN_RDAIEN_Msk) || defined(UART_INTEN_RXTOIEN_Msk) || defined(UART_INTEN_TOCNTEN_Msk)
    #define IER 							INTEN
#endif

#if defined(FMC_ISPADDR_ISPADDR_Msk)
    #define ISPADR 							ISPADDR
#endif

#if defined(UART_INTSTS_RDAIF_Msk) || defined(UART_INTSTS_RXTOIF_Msk)
    #define ISR								INTSTS
#endif

#if defined(UART_FIFO_RFITL_14BYTES) || defined(UART_FIFO_RTSTRGLV_14BYTES)
    #define FCR								FIFO
#endif

#if defined(UART_FIFOSTS_RXEMPTY_Msk) || defined(UART_FIFOSTS_TXFULL_Msk)
    #define FSR								FIFOSTS
#endif

#if defined(UART_DAT_DAT_Msk) && !defined(TARGET_NANO1X2) && !defined(TARGET_NANO100A) && !defined(TARGET_NANO100B)
    #define RBR								DAT
    #define THR								DAT
#endif

#if defined(UART_LINE_WLS_Msk)
    #define LCR								LINE
#endif

#if defined(TARGET_NANO103)
    #define TLCTL							LINE
#endif

#if defined(UART_FUNCSEL_UART)
    #define FUN_SEL							FUNCSEL
#elif defined(UART_FUNSEL_FUN_SEL_Pos)
    #define FUN_SEL							FUNSEL
#endif

#if defined(UART_TOUT_TOIC_Msk)
    #define TOR								TOUT
#elif defined(UART_TMCTL_TOIC_Msk)
    #define TOR								TMCTL
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* UART Bit Field Definition                                                                               */
/*     -->     UART_IER_RTO_IEN_Msk                                                                        */
/*          == UART_IER_TOUT_IEN_Msk                                                                       */
/*          == UART_INTEN_RXTOIEN_Msk                                                                      */
/*                                                                                                         */
/*     -->     UART_IER_RDA_IEN_Msk                                                                        */
/*          == UART_INTEN_RDAIEN_Msk                                                                       */
/*                                                                                                         */
/*     -->     UART_IER_TIME_OUT_EN_Msk                                                                    */
/*          == UART_INTEN_TOCNTEN_Msk                                                                      */
/*                                                                                                         */
/*     -->     UART_ISR_RDA_IF_Msk                                                                         */
/*          == UART_INTSTS_RDAIF_Msk                                                                       */
/*                                                                                                         */
/*     -->     UART_ISR_TOUT_IF_Msk                                                                        */
/*          == UART_INTSTS_RXTOIF_Msk                                                                      */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#if defined(UART_IER_RTO_IEN_Msk)
    // No Conflicts
#elif defined(UART_IER_TOUT_IEN_Msk)
    #define UART_IER_RTO_IEN_Msk	UART_IER_TOUT_IEN_Msk
#elif defined(UART_INTEN_RXTOIEN_Msk)
    #define UART_IER_RTO_IEN_Msk	UART_INTEN_RXTOIEN_Msk
#endif

#if defined(UART_IER_RDA_IEN_Msk)
    // No Conflicts
#elif defined(UART_INTEN_RDAIEN_Msk)
    #define UART_IER_RDA_IEN_Msk	UART_INTEN_RDAIEN_Msk
#endif

#if defined(UART_INTEN_TOCNTEN_Msk)
    #define UART_IER_TIME_OUT_EN_Msk	UART_INTEN_TOCNTEN_Msk
#elif defined(TARGET_NANO1X2) || defined(TARGET_NANO100A) || defined(TARGET_NANO100B) || defined(TARGET_NANO103)
    // No euevilaent bit filed
    #define UART_IER_TIME_OUT_EN_Msk		0
#endif

#if defined(UART_INTSTS_RDAIF_Msk) && defined(UART_INTSTS_RXTOIF_Msk)
    #define UART_ISR_RDA_IF_Msk				UART_INTSTS_RDAIF_Msk
    #define UART_ISR_TOUT_IF_Msk			UART_INTSTS_RXTOIF_Msk
#endif

#if defined(UART_FIFO_RFITL_14BYTES) && defined(UART_FIFO_RTSTRGLV_14BYTES)
    #define UART_FCR_RFITL_14BYTES			UART_FIFO_RFITL_14BYTES
    #define UART_FCR_RTS_TRI_LEV_14BYTES	UART_FIFO_RTSTRGLV_14BYTES
#endif

#if defined(UART_FIFOSTS_RXEMPTY_Msk) && defined(UART_FIFOSTS_TXFULL_Msk)
    #define UART_FSR_RX_EMPTY_Msk			UART_FIFOSTS_RXEMPTY_Msk
    #define UART_FSR_TX_FULL_Msk			UART_FIFOSTS_TXFULL_Msk
#endif


// UART->FUN_SEL == UART->FUNCSEL == UART->FUNSEL
#if defined(UART_FUNCSEL_UART)
    #define UART_FUNC_SEL_UART				UART_FUNCSEL_UART
#endif

#if defined(UART_FSR_RX_EMPTY_F_Msk)
    #define UART_FSR_RX_EMPTY_Msk			UART_FSR_RX_EMPTY_F_Msk
#endif

#if defined(UART_FSR_TX_FULL_F_Msk)
    #define UART_FSR_TX_FULL_Msk			UART_FSR_TX_FULL_F_Msk
#endif

#if defined(UART_IER_RTO_IE_Msk)
    #define UART_IER_RTO_IEN_Msk			UART_IER_RTO_IE_Msk
#endif

#if defined(UART_IER_RDA_IE_Msk)
    #define UART_IER_RDA_IEN_Msk			UART_IER_RDA_IE_Msk
#endif

#if defined(UART_TOUT_TOIC_Msk)
    #define UART_TOR_TOIC_Msk					UART_TOUT_TOIC_Msk
#endif

#if defined(UART_LINE_RFITL_14BYTES) && defined(UART_LINE_RTS_TRI_LEV_14BYTES)
    #define UART_TLCTL_RFITL_14BYTES			UART_LINE_RFITL_14BYTES
    #define UART_TLCTL_RTS_TRI_LEV_14BYTES		UART_LINE_RTS_TRI_LEV_14BYTES
#endif

#if defined(TARGET_MINI51) || defined(TARGET_NUC029FAE)
/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
// This function is missing in sys.h
static __INLINE void SYS_UnlockReg(void)
{
    while(SYS->RegLockAddr != 1) {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }
}
#elif defined(TARGET_NANO100A) || defined(TARGET_NANO100B) || defined(TARGET_NANO1X2) || defined(TARGET_NUC472_442)
// This variable is missing in system_Nano100Series.c
extern uint32_t PllClock;               /*!< PLL Output Clock Frequency          */
#endif

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/





#endif  /* __BSP_NAME_H__ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
