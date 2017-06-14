/******************************************************************************
 * @file     uart_transfer.h
 * @brief    General UART ISP slave header file
 * @version  1.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE        	64

/*-------------------------------------------------------------*/

extern uint8_t  uart_rcvbuf[];
extern uint8_t volatile bUartDataReady;
extern uint8_t volatile bufhead;

/*-------------------------------------------------------------*/
void UART_Init(void);
void UART0_IRQHandler(void);
void PutString(void);
uint32_t UART_IS_CONNECT(void);


#if defined(TARGET_M051)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);
}
#elif defined(TARGET_M058)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD | SYS_MFP_P31_TXD);
}
#elif defined(TARGET_M451)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
}


#elif defined(TARGET_M0518)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
}

#define UART_T							UART0
#define UART_T_IRQHandler		UART02_IRQHandler
#define UART_T_IRQn					UART02_IRQn

#elif defined(TARGET_M0519)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk );
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD );
}

#elif defined(TARGET_MINI51)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD | SYS_MFP_P13_TXD);
}
#define UART_T							UART
#define UART_T_IRQHandler		UART_IRQHandler
#define UART_T_IRQn					UART_IRQn

#elif defined(TARGET_MINI58)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);
}

#elif defined(TARGET_NANO1X2)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk|SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |=  (SYS_PB_L_MFP_PB0_MFP_UART0_TX|SYS_PB_L_MFP_PB1_MFP_UART0_RX);
}
#elif defined(TARGET_NANO100A) || defined(TARGET_NANO100B)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk | SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA14_MFP_UART0_RX | SYS_PA_H_MFP_PA15_MFP_UART0_TX);
}
#elif defined(TARGET_NANO103)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk|SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |=  (SYS_GPB_MFPL_PB0MFP_UART0_RXD|SYS_GPB_MFPL_PB1MFP_UART0_TXD);
}
#elif defined(TARGET_NM1530)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);
}
#elif defined(TARGET_NUC029FAE)
#include "NUC029FAE.h"
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD | SYS_MFP_P13_TXD);
}

#define UART_T							UART
#define UART_T_IRQHandler		UART_IRQHandler
#define UART_T_IRQn					UART_IRQn

#elif defined(TARGET_NUC029XAN)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);
}

#elif defined(TARGET_NUC100) || defined(TARGET_NUC122) || defined(TARGET_NUC123)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);
}




#define UART_T							UART1
#define UART_T_IRQHandler		UART1_IRQHandler
#define UART_T_IRQn					UART1_IRQn







#elif defined(TARGET_NUC131) || defined(TARGET_NUC200) || defined(TARGET_NUC230_240)
static __INLINE void UART_MFP_Setting(void)
{
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
}


#define UART_T							UART0
#define UART_T_IRQHandler		UART02_IRQHandler
#define UART_T_IRQn					UART02_IRQn



#elif defined(TARGET_NUC472_442)
static __INLINE void UART_MFP_Setting(void)
{
//    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;
    SYS->GPA_MFPH = SYS_GPA_MFPH_PA13MFP_UART0_RXD | SYS_GPA_MFPH_PA14MFP_UART0_TXD ;

}

#elif defined(TARGET_M480)

static __INLINE void UART_MFP_Setting(void)
{
    /* Set PD multi-function pins for UART0 RXD(PD.2) and TXD(PD.3) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD2MFP_UART0_RXD | SYS_GPD_MFPL_PD3MFP_UART0_TXD);
}

#else
#error "UART Multiple Function Pin is not set correcly in uart_transfer.h"
#endif


/* rename for uart_transfer.c */
#ifndef UART_T
#define UART_T							UART0
#define UART_T_IRQHandler		UART0_IRQHandler
#define UART_T_IRQn					UART0_IRQn
#endif

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
