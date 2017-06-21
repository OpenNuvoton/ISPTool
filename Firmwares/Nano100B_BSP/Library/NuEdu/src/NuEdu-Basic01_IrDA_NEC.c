/**************************************************************************//**
 * @file     NuEdu-Basic01_IrDA_NEC.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/15 2:05p $
 * @brief    NuEdu-Basic01 IrDA NEC driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_IrDA_NEC.h"


/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS IrDA NEC Exported Functions
  @{
*/


/// @cond HIDDEN_SYMBOLS
volatile    uint8_t             IR_State = 0;       // IR State
volatile    uint8_t             IR_LDC_Ready = 0;   // LeaDer Code is Ready
volatile    uint8_t             IR_CTC_Ready = 0;   // CusTomer Code is Ready
volatile    uint8_t             IR_CTC0 = 0;        // Received CusTomer Code 0
volatile    uint8_t             IR_CTC1 = 0;        // Received CusTomer Code 1
volatile    uint8_t             IR_DAC = 0;         // Received Data Code
volatile    uint8_t             IR_DAB = 0;         // Received Data Bar code
volatile    uint8_t             IR_cnt = 0;
volatile    uint8_t             IR_CODE[4]  =   {0x00, 0x00, 0x00, 0x00};
IrDA_Code_Exe g_pfnIrDA_Code_Exe;
/// @endcond

/**
  * @brief This function is used to detect NEC IR procotol
  * @param[in] u32Time is the time length of received bit
  * @return None
  */
void IrDa_NEC_Rx(uint32_t u32Time)
{
    if(IR_State == 0) {
        IR_LDC_Ready = 0;           // Clear LeaDer Code Ready
        IR_CTC_Ready = 0;           // Clear CusTomer Code Ready
        IR_State++;
    }
    // Leader or Repeater code
    else if(IR_State == 1) {
        // Leader code
        if((u32Time >= IR_LDC_MIN) && (u32Time <= IR_LDC_MAX)) {
            IR_LDC_Ready = 1;       // Set LeaDer Code Ready
            IR_State++;
        } else {
            IR_State = 1;
            IR_LDC_Ready = 0;           // Clear LeaDer Code Ready
            IR_CTC_Ready = 0;           // Clear CusTomer Code Ready
        }
    }
    // Customer code 0
    else if((IR_State >= 2 && IR_State < 10) && (IR_LDC_Ready == 1)) {
        IR_State++;
        IR_CTC0 = IR_CTC0 >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_CTC0 &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX)) // 2.25ms = 1
            IR_CTC0 |= 0x80;
        else
            IR_State = 0;
    }
    // Customer code 1
    else if((IR_State >= 10 && IR_State < 18) && (IR_LDC_Ready == 1)) {
        IR_State++;
        IR_CTC1 = IR_CTC1 >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_CTC1 &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX)) // 2.25ms = 1
            IR_CTC1 |= 0x80;
        else
            IR_State = 0;
    }
    // Data
    else if((IR_State >= 18 && IR_State < 26) && (IR_LDC_Ready == 1)) {
        IR_State++;
        IR_DAC = IR_DAC >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_DAC &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_DAC |= 0x80;
        else
            IR_State = 0;

    }
    // Data bar
    else if((IR_State >= 26 && IR_State < 34) && (IR_LDC_Ready == 1)) {
        IR_State++;
        IR_DAB = IR_DAB >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))      // 1.12ms = 0
            IR_DAB &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_DAB |= 0x80;
        else
            IR_State = 0;

        if(IR_State == 34) {
            if((IR_DAC ^ IR_DAB) == 0xff) {
                IR_LDC_Ready = 0;   // Clear LeaDer Code Ready
                IR_CODE[0] = IR_CTC0;
                IR_CODE[1] = IR_CTC1;
                IR_CODE[2] = IR_DAC;
                IR_CODE[3] = IR_DAB;
                IR_cnt++;
                g_pfnIrDA_Code_Exe(IR_CODE);
                printf("IR_cnt=%d, CTC0=%02x, CTC1=%02x, DAC=%02x, DAB=%02x\n", IR_cnt, IR_CTC0, IR_CTC1, IR_DAC, IR_DAB);
            }
            IR_State = 0;
        }
    }
}

#define     NEC_LDC_MARK        16      // 16 x 560us = 8960us =   9ms
#define     NEC_LDC_SPACE       8       //  8 x 560us = 4480us = 4.5ms
#define     NEC_BIT_MARK        1       // 560us
#define     NEC_ONE_SPACE       3       //  3 x 560us = 1680us = 1690us
#define     NEC_ZERO_SPACE      1       // 560us
#define     NEC_BYTES           4

/**
  * @brief This function is used to transmit MASK waveform
  *        Pulse = 1/3 duty @38KHz frequency
  * @param[in] N is time length of MASK
  * @return None
  */
void Mark(uint8_t N)
{
    /* Switch to PWM output waveform */
    PWM_EnableOutput(PWM1,PWM_CH_2_MASK);
    CLK_SysTickDelay(560*N);
    PWM_DisableOutput(PWM1,PWM_CH_2_MASK);
}
/**
  * @brief This function is used to transmit SPACE waveform
  * @param[in] N is time length of SPACE
  * @return None
  */
void SPACE(uint8_t N)
{
    CLK_SysTickDelay(560*N);
}

/**
  * @brief This function is used to transmit IrDA NEC waveform through PC 15 (PWM1_CH3)
  * @param[in] data is pointer of trasnmitted data
  * @return None
  */
void SendNEC(uint8_t* data)
{
    uint8_t nbyte;
    uint8_t nbit;

    /* Send out Leader code */
    Mark(NEC_LDC_MARK);
    SPACE(NEC_LDC_SPACE);

    /* Send out Customer code and Data code */
    for (nbyte=0; nbyte < NEC_BYTES; nbyte++) {
        for (nbit=0; nbit < 8; nbit++) {
            Mark(NEC_BIT_MARK);
            if (data[nbyte] & (1 << nbit))      // LSB first
                SPACE(NEC_ONE_SPACE);
            else
                SPACE(NEC_ZERO_SPACE);
        }
    }

    /* Send out Stop bit */
    Mark(NEC_BIT_MARK);

}

/**
  * @brief This function is used to initiate PWM for IrDA NEC
  * @param[in] pfnIrDA_Code_Exe is function pointer that will be executed after received IrDA NEC command
  * @return None
  */
void IrDA_NEC_TxRx_Init(IrDA_Code_Exe pfnIrDA_Code_Exe)
{
    g_pfnIrDA_Code_Exe = pfnIrDA_Code_Exe;

    CLK_SetModuleClock(PWM1_CH23_MODULE, CLK_CLKSEL2_PWM1_CH23_S_HIRC, 0);
    CLK_EnableModuleClock(PWM1_CH23_MODULE);

    SYS->PC_H_MFP |= ( SYS_PC_H_MFP_PC14_MFP_PWM1_CH3 | SYS_PC_H_MFP_PC15_MFP_PWM1_CH2);

    PWM1->CTL |= (PWM_CTL_CH3MOD_Msk|PWM_CTL_CH2MOD_Msk);               //Set PWM1_CH3 is Continuous Mode
    PWM_SET_DIVIDER(PWM1, 2, PWM_CLK_DIV_1);                            //Set PWM1_CH2 Clock Source Divider
    PWM_SET_DIVIDER(PWM1, 3, PWM_CLK_DIV_4);                            //Set PWM1_CH3 Clock Source Divider
    PWM_SET_PRESCALER(PWM1, 3, 2);                                      //Set PWM1_CH2 and PWM1_CH3 Prescaler
    PWM_SET_CNR(PWM1, 3, MaxValue);
    PWM_SET_CNR(PWM1, 2, ((120000000/3/38000+5)/10));
    PWM_SET_CMR(PWM1, 2, ((120000000/3/38000+5)/10)/2);

    //Start Capture
    PWM_EnableCapture(PWM1, PWM_CH_3_MASK);
    PWM1->CAPCTL |= PWM_CAPCTL_CAPRELOADFEN3_Msk;
    PWM_EnableCaptureInt(PWM1, 3,PWM_FALLING_LATCH_INT_ENABLE);
    NVIC_EnableIRQ(PWM1_IRQn);
    PWM_Start(PWM1, PWM_CH_2_MASK|PWM_CH_3_MASK);
}

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS IrDA NEC Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
