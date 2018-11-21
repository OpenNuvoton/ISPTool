/**************************************************************************//**
 * @file     acmp.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 2018/10/03 14:41 $
 * @brief    NM1230 ACMP Driver Header File
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __ACMP_H__
#define __ACMP_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_ACMP_Driver ACMP Driver
  @{
*/


/** @addtogroup NM1230_ACMP_EXPORTED_CONSTANTS ACMP Exported Constants
  @{
*/



/*---------------------------------------------------------------------------------------------------------*/
/* ACMP_CTL constant definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define ACMP_CTL_NFCLKS_PCLKDIV_1    (0UL << ACMP_CTL_NFCLKS_Pos)     /*!< ACMP_CTL setting for noise filter frequency clock is PCLK/1. */
#define ACMP_CTL_NFCLKS_PCLKDIV_2    (1UL << ACMP_CTL_NFCLKS_Pos)     /*!< ACMP_CTL setting for noise filter frequency clock is PCLK/2. */
#define ACMP_CTL_NFCLKS_PCLKDIV_4    (2UL << ACMP_CTL_NFCLKS_Pos)     /*!< ACMP_CTL setting for noise filter frequency clock is PCLK/4. */
#define ACMP_CTL_NFCLKS_PCLKDIV_16   (3UL << ACMP_CTL_NFCLKS_Pos)     /*!< ACMP_CTL setting for noise filter frequency clock is PCLK/16. */
#define ACMP_CTL_INTPOL_R            (1UL << ACMP_CTL_EDGESEL_Pos)    /*!< ACMP_CTL setting for selecting rising edge as interrupt condition. */
#define ACMP_CTL_INTPOL_F            (2UL << ACMP_CTL_EDGESEL_Pos)    /*!< ACMP_CTL setting for selecting falling edge as interrupt condition. */
#define ACMP_CTL_INTPOL_RF           (3UL << ACMP_CTL_EDGESEL_Pos)    /*!< ACMP_CTL setting for selecting rising edge and falling edge as interrupt condition. */
#define ACMP_CTL_POSSEL_P0           (0UL << ACMP_CTL_CPPSEL_Pos)     /*!< ACMP_CTL setting for selecting ACMPx_P0 pin as the source of ACMP V+. */
#define ACMP_CTL_POSSEL_P1           (1UL << ACMP_CTL_CPPSEL_Pos)     /*!< ACMP_CTL setting for selecting ACMPx_P1 pin as the source of ACMP V+. */
#define ACMP_CTL_POSSEL_P2           (2UL << ACMP_CTL_CPPSEL_Pos)     /*!< ACMP_CTL setting for selecting ACMPx_P2 pin as the source of ACMP V+. */
#define ACMP_CTL_POSSEL_P3           (3UL << ACMP_CTL_CPPSEL_Pos)     /*!< ACMP_CTL setting for selecting ACMPx_P3 pin as the source of ACMP V+. */
#define ACMP_CTL_POSSEL_PGA          (4UL << ACMP_CTL_CPPSEL_Pos)     /*!< ACMP_CTL setting for selecting PGA_CMP as the source of ACMP V+. */
#define ACMP_CTL_NEGSEL_PIN          (0UL << ACMP_CTL_CPNSEL_Pos)     /*!< ACMP_CTL setting for selecting the voltage of ACMP negative input pin as the source of ACMP V-. */
#define ACMP_CTL_NEGSEL_VBG          (1UL << ACMP_CTL_CPNSEL_Pos)     /*!< ACMP_CTL setting for selecting internal Band-gap voltage as the source of ACMP V-. */
#define ACMP_CTL_NEGSEL_CRV          (2UL << ACMP_CTL_CPNSEL_Pos)     /*!< ACMP_CTL setting for selecting internal comparator reference voltage as the source of ACMP V-. */
#define ACMP_CTL_NEGSEL_DAC0         (3UL << ACMP_CTL_CPNSEL_Pos)     /*!< ACMP_CTL setting for selecting DAC0 as the source of ACMP V-. */
#define ACMP_CTL_NEGSEL_DAC1         (3UL << ACMP_CTL_CPNSEL_Pos)     /*!< ACMP_CTL setting for selecting DAC0 as the source of ACMP V-. */
#define ACMP_CTL_HYSTERESIS_ENABLE   (1UL << ACMP_CTL_ACMPHYSEN_Pos)  /*!< ACMP_CTL setting for enabling the hysteresis function. */
#define ACMP_CTL_HYSTERESIS_DISABLE  (0UL << ACMP_CTL_ACMPHYSEN_Pos)  /*!< ACMP_CTL setting for disabling the hysteresis function. */
#define ACMP_CTL_DLYTREGSEL_R        (1UL << ACMP_CTL_DLYTRGSEL_Pos)  /*!< ACMP_CTL setting for selecting rising edge as delay trigger condition.*/
#define ACMP_CTL_DLYTREGSEL_F        (2UL << ACMP_CTL_DLYTRGSEL_Pos)  /*!< ACMP_CTL setting for selecting falling edge as delay trigger condition. */
#define ACMP_CTL_DLYTREGSEL_RF       (3UL << ACMP_CTL_DLYTRGSEL_Pos)  /*!< ACMP_CTL setting for selecting rising edge and falling edge as delay trigger condition. */
#define ACMP_CTL_DLYTRGSOR_PWM0      (0UL << ACMP_CTL_DLYTRGSOR_Pos)  /*!< ACMP_CTL setting for selecting PWM0 pin as the source of ACMP delay trigger. */
#define ACMP_CTL_DLYTRGSOR_PWM2      (1UL << ACMP_CTL_DLYTRGSOR_Pos)  /*!< ACMP_CTL setting for selecting PWM2 pin as the source of ACMP delay trigger. */
#define ACMP_CTL_DLYTRGSOR_PWM4      (2UL << ACMP_CTL_DLYTRGSOR_Pos)  /*!< ACMP_CTL setting for selecting PWM3 pin as the source of ACMP delay trigger. */ 
#define ACMP_CTL_DLYTRGSOR_TMR4      (3UL << ACMP_CTL_DLYTRGSOR_Pos)  /*!< ACMP_CTL setting for selecting TIMER4 pin as the source of ACMP delay trigger. */
#define ACMP_CTL_PRESET_L            (0UL << ACMP_CTL_PRESET_Pos)     /*!< ACMP_CTL setting for selecting low as the preset value. */
#define ACMP_CTL_PRESET_H            (1UL << ACMP_CTL_PRESET_Pos)     /*!< ACMP_CTL setting for selecting high as the preset value. */
#define ACMP_CTL_DLYEDGE_R           (1UL << ACMP_CTL_DLYEDGESEL_Pos) /*!< ACMP_CTL setting for selecting rising edge as delay edge position.*/
#define ACMP_CTL_DLYEDGE_F           (2UL << ACMP_CTL_DLYEDGESEL_Pos) /*!< ACMP_CTL setting for selecting falling edge as delay edge position. */
#define ACMP_CTL_DLYEDGE_RF          (3UL << ACMP_CTL_DLYEDGESEL_Pos) /*!< ACMP_CTL setting for selecting rising edge and falling edge as delay edge position. */



/*@}*/ /* end of group NM1230_ACMP_EXPORTED_CONSTANTS */


/** @addtogroup NM1230_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Define Macros and functions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/


/**
  * @brief This macro is used to select comparator reference voltage using DAC0
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32Level  The comparator reference voltage setting.
  *             The formula is:
  *                       comparator reference voltage = AVDD x (u32Level/4096)
  *             The range of u32Level is 0 ~ 4095.
  * @return   None
  * @details  When DAC0 is selected as ACMP negative input source, the DAC0 level is determined by DAC0 (ACMP_DAC[15:4]).
  * \hideinitializer  
  */
#define ACMP_DAC0_SEL(acmp, u32Level) ((acmp)->DAC = ((acmp)->DAC & ~ACMP_DAC_DAC0_Msk) | ((u32Level)<<ACMP_DAC_DAC0_Pos))

/**
  * @brief This macro is used to select comparator reference voltage using DAC1
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32Level  The comparator reference voltage setting.
  *             The formula is:
  *                       comparator reference voltage = AVDD x (u32Level/4096)
  *             The range of u32Level is 0 ~ 4095.
  * @return   None
  * @details  When DAC1 is selected as ACMP negative input source, the DAC1 level is determined by DAC1 (ACMP_DAC[31:20]).
  * \hideinitializer  
  */
#define ACMP_DAC1_SEL(acmp, u32Level) ((acmp)->DAC = ((acmp)->DAC & ~ACMP_DAC_DAC1_Msk) | ((u32Level)<<ACMP_DAC_DAC1_Pos))

/**
  * @brief This macro is used to select comparator reference voltage using DAC2
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32Level  The comparator reference voltage setting.
  *             The formula is:
  *                       comparator reference voltage = AVDD x (u32Level/4096)
  *             The range of u32Level is 0 ~ 4095.
  * @return   None
  * @details  When DAC2 is selected as ACMP negative input source, the DAC2 level is determined by DAC2 (ACMP_VREF[15:4]).
  * \hideinitializer  
  */
#define ACMP_DAC2_SEL(acmp, u32Level) ((acmp)->VREF = (((acmp)->VREF & ~ACMP_VREF_DAC2_Msk) | ((u32Level)<<ACMP_VREF_DAC2_Pos)) | ACMP_VREF_DACMODE_Msk)

/**
  * @brief This macro is used to select comparator reference voltage
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32Level  The comparator reference voltage setting.
  *             The formula is:
  *                       comparator reference voltage = AVDD x (1/6 + u32Level/24)
  *             The range of u32Level is 0 ~ 15.
  * @return   None
  * @details  When CRV is selected as ACMP negative input source, the CRV level is determined by CRVCTL (ACMP_VREF[3:0]).
  * \hideinitializer  
  */
#define ACMP_CRV_SEL(acmp, u32Level) ((acmp)->VREF = (((acmp)->VREF & ~ACMP_VREF_CRVCTL_Msk) | (((u32Level)<<ACMP_VREF_CRVCTL_Pos) & ACMP_VREF_CRVCTL_Msk)) & ~ACMP_VREF_DACMODE_Msk)

/**
  * @brief This macro is used to set ACMP delay trigger cycle
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32Level  The ACMP delay trigger cycle setting.
  *             The formula is:
  *                       delay cycle = u32Level x ACMP clock cycle
  * @return   None
  * @details  This macro will set ACMP delay trigger cycle.
  * \hideinitializer  
  */
  #define ACMP_TRGDLY_SEL(acmp, u32Level) ((acmp)->TRGDLY = (((acmp)->TRGDLY & ~ACMP_TRGDLY_DELAY_Msk) | ((u32Level == 0)?(1):(u32Level))<<ACMP_TRGDLY_DELAY_Pos))

/**
  * @brief This macro is used to get ACMP delay trigger output value
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return  ACMP output value
  * @details This macro will return the ACMP delay trigger output value.
  * \hideinitializer  
  */
#define ACMP_GET_DLYTRG_OUTPUT(acmp, u32ChNum) (((acmp)->STATUS & (ACMP_STATUS_DLYTRGO0_Msk<<(u32ChNum)))?1:0)

/**
  * @brief This macro is used to get ACMP delay trigger flag
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return   ACMP delay trigger occurred (1) or not (0)
  * @details This macro will return the ACMP delay trigger flag.
  * \hideinitializer  
  */
#define ACMP_GET_DLYTRG_FLAG(acmp, u32ChNum) (((acmp)->STATUS & (ACMP_STATUS_DLYTRGF0_Msk<<(u32ChNum)))?1:0)

/**
  * @brief This macro is used to clear ACMP delay trigger flag
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return   None
  * @details This macro will write 1 to DLYTRGFn bit of ACMP_STATUS register to clear delay trigger flag.
  * \hideinitializer  
  */
#define ACMP_CLR_DLYTRG_FLAG(acmp, u32ChNum) ((acmp)->STATUS = (ACMP_STATUS_DLYTRGF0_Msk<<(u32ChNum)))

/**
  * @brief This macro is used to set comparator result preset value
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32val is comparator result preset value. Including:
  *                  - \ref ACMP_CTL_PRESET_L
  *                  - \ref ACMP_CTL_PRESET_H
  * @return None
  * @details This macro will set PRESET (ACMP_CTL[31]) to determine the comparator result preset value.
  * \hideinitializer  
  */
#define ACMP_SET_PRESET(acmp, u32ChNum, u32val) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_PRESET_Msk) | (u32val))

/**
  * @brief This macro is used to select delay edge position of PWM for ACMP delay trigger 
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Src is delay trigger edge type. Including:
  *                  - \ref ACMP_CTL_DLYEDGE_R
  *                  - \ref ACMP_CTL_DLYEDGE_F
  *                  - \ref ACMP_CTL_DLYEDGE_RF
  * @return None
  * @details This macro will set DLYTRGSOR (ACMP_CTL[15:14]) to determine the delay edge position of PWM for ACMP delay trigger.
  * \hideinitializer  
  */
#define ACMP_SELECT_DLYEDGE_SEL(acmp, u32ChNum, u32Src) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_DLYEDGESEL_Msk) | (u32Src))

/**
  * @brief This macro is used to enable ACMP delay trigger interrupt
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set DLYTRGIE bit of ACMP_CTL register to enable ACMP delay trigger interrupt.
  * \hideinitializer  
  */
#define ACMP_ENABLE_DLYTRG_INT(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_DLYTRGIE_Msk)

/**
  * @brief This macro is used to disable ACMP delay trigger interrupt
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear DLYTRGIE bit of ACMP_CTL register to disable ACMP delay trigger interrupt.
  * \hideinitializer  
  */
#define ACMP_DISABLE_DLYTRG_INT(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_DLYTRGIE_Msk)

/**
  * @brief This macro is used to enable ACMP delay trigger
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set DLYTRGEN bit of ACMP_CTL register to enable ACMP delay trigger.
  * \hideinitializer  
  */
#define ACMP_ENABLE_DLYTRG(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_DLYTRGEN_Msk)

/**
  * @brief This macro is used to disable ACMP delay trigger
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear DLYTRGEN bit of ACMP_CTL register to disable ACMP delay trigger.
  * \hideinitializer  
  */
#define ACMP_DISABLE_DLYTRG(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_DLYTRGEN_Msk)

/**
  * @brief This macro is used to select ACMP delay trigger source
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Src is delay trigger source selection. Including:
  *                  - \ref ACMP_CTL_DLYTRGSOR_PWM0
  *                  - \ref ACMP_CTL_DLYTRGSOR_PWM2
  *                  - \ref ACMP_CTL_DLYTRGSOR_PWM4
  *                  - \ref ACMP_CTL_DLYTRGSOR_TMR4
  * @return None
  * @details This macro will set DLYTRGSOR (ACMP_CTL[11:10]) to determine the source of delay trigger.
  * \hideinitializer  
  */
#define ACMP_SELECT_DLYTRG_SRC(acmp, u32ChNum, u32Src) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_DLYTRGSOR_Msk) | (u32Src))

/**
  * @brief This macro is used to select ACMP dealy trigger mode level
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Mode dealy trigger mode selection. Including: 
  *                  - \ref ACMP_CTL_DLYTREGSEL_R
  *                  - \ref ACMP_CTL_DLYTREGSEL_F
  *                  - \ref ACMP_CTL_DLYTREGSEL_RF
  * @return None
  * @details The ACMP dealy trigger mode can be rising edge, falling edge or any edge.
  * \hideinitializer  
  */
#define ACMP_SELECT_DLYTREG_MODE(acmp, u32ChNum, u32Mode) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_DLYTRGSEL_Msk) | (u32Mode))

/**
  * @brief This macro is used to enable brake output delay
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set PBRKSEL bit of ACMP_CTL register to enable brake output delay.
  * \hideinitializer  
  */
#define ACMP_ENABLE_BRAKE_DELAY(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_PBRKSEL_Msk)

/**
  * @brief This macro is used to disable brake output delay
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear PBRKSEL bit of ACMP_CTL register to disable brake output delay.
  * \hideinitializer  
  */
#define ACMP_DISABLE_BRAKE_DELAY(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_PBRKSEL_Msk)

/**
  * @brief This macro is used to enable output inverse function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set POLARITY bit of ACMP_CTL register to enable output inverse function.
  * \hideinitializer  
  */
#define ACMP_ENABLE_OUTPUT_INVERSE(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_POLARITY_Msk)

/**
  * @brief This macro is used to disable output inverse function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear POLARITY bit of ACMP_CTL register to disable output inverse function.
  * \hideinitializer  
  */
#define ACMP_DISABLE_OUTPUT_INVERSE(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_POLARITY_Msk)

/**
  * @brief This macro is used to select ACMP negative input source
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Src is comparator negative input selection. Including:
  *                  - \ref ACMP_CTL_NEGSEL_PIN  
  *                  - \ref ACMP_CTL_NEGSEL_VBG
  *                  - \ref ACMP_CTL_NEGSEL_CRV
  *                  - \ref ACMP_CTL_NEGSEL_DAC
  * @return None
  * @details This macro will set CPNSEL (ACMP_CTL[25:24]) to determine the source of negative input.
  * \hideinitializer  
  */
#define ACMP_SET_NEG_SRC(acmp, u32ChNum, u32Src) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_CPNSEL_Msk) | (u32Src))

/**
  * @brief This macro is used to enable hysteresis function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMPHYSEN (ACMP_CTL[3:2]) to enable hysteresis function.
  * \hideinitializer  
  */
#define ACMP_ENABLE_HYSTERESIS(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= (1 << ACMP_CTL_ACMPHYSEN_Pos))

/**
  * @brief This macro is used to disable hysteresis function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMPHYSEN (ACMP_CTL[3:2]) to disable hysteresis function.
  * \hideinitializer  
  */
#define ACMP_DISABLE_HYSTERESIS(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_ACMPHYSEN_Msk)

/**
  * @brief This macro is used to enable interrupt
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMPIE bit of ACMP_CTL register to enable interrupt function.
  * \hideinitializer  
  */
#define ACMP_ENABLE_INT(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_ACMPIE_Msk)

/**
  * @brief This macro is used to disable interrupt
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMPIE bit of ACMP_CTL register to disable interrupt function.
  * \hideinitializer  
  */
#define ACMP_DISABLE_INT(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_ACMPIE_Msk)

/**
  * @brief This macro is used to enable ACMP
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set ACMPEN bit of ACMP_CTL register to enable analog comparator.
  * \hideinitializer  
  */
#define ACMP_ENABLE(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_ACMPEN_Msk)

/**
  * @brief This macro is used to disable ACMP
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear ACMPEN bit of ACMP_CTL register to disable analog comparator.
  * \hideinitializer  
  */
#define ACMP_DISABLE(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_ACMPEN_Msk)

/**
  * @brief This macro is used to get ACMP output value
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return  ACMP output value
  * @details This macro will return the ACMP output value.
  * \hideinitializer  
  */
#define ACMP_GET_OUTPUT(acmp, u32ChNum) (((acmp)->STATUS & (ACMP_STATUS_ACMPO0_Msk<<(u32ChNum)))?1:0)

/**
  * @brief This macro is used to get ACMP interrupt flag
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return   ACMP interrupt occurred (1) or not (0)
  * @details This macro will return the ACMP interrupt flag.
  * \hideinitializer  
  */
#define ACMP_GET_INT_FLAG(acmp, u32ChNum) (((acmp)->STATUS & (ACMP_STATUS_ACMPF0_Msk<<(u32ChNum)))?1:0)

/**
  * @brief This macro is used to clear ACMP interrupt flag
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return   None
  * @details This macro will write 1 to ACMPFn bit of ACMP_STATUS register to clear interrupt flag.
  * \hideinitializer  
  */
#define ACMP_CLR_INT_FLAG(acmp, u32ChNum) ((acmp)->STATUS = (ACMP_STATUS_ACMPF0_Msk<<(u32ChNum)))

/**
  * @brief This macro is used to select ACMP positive input pin
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Pin Comparator positive pin selection. Including:
  *                  - \ref ACMP_CTL_POSSEL_P0
  *                  - \ref ACMP_CTL_POSSEL_P1
  *                  - \ref ACMP_CTL_POSSEL_P2
  *                  - \ref ACMP_CTL_POSSEL_P3
  *                  - \ref ACMP_CTL_POSSEL_PGA
  * @return None
  * @details This macro will set CPPSEL (ACMP_CTL[30:28]) to determine the comparator positive input pin.
  * \hideinitializer  
  */
#define ACMP_SELECT_P(acmp, u32ChNum, u32Pin) \
	do{ \
	        if((u32ChNum == 1) && (u32Pin == ACMP_CTL_POSSEL_PGA)) \
			(acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_CPPSEL_Msk) | (3 << ACMP_CTL_CPPSEL_Pos); \
		else \
			(acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_CPPSEL_Msk) | (u32Pin); \
    }while(0)

/**
  * @brief This macro is used to enable ACMP filter function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will clear NFDIS (ACMP_CTL[23]) to enable output filter function.
  * \hideinitializer  
  */
#define ACMP_ENABLE_FILTER(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] &= ~ACMP_CTL_NFDIS_Msk)

/**
  * @brief This macro is used to disable ACMP filter function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * @details This macro will set NFDIS (ACMP_CTL[12]) to disable output filter function.
  * \hideinitializer  
  */
#define ACMP_DISABLE_FILTER(acmp, u32ChNum) ((acmp)->CTL[u32ChNum] |= ACMP_CTL_NFDIS_Msk)

/**
  * @brief This macro is used to set ACMP filter function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Clkdiv is Noise Filter clock pre-divided.
  *                  - \ref ACMP_CTL_NFCLKS_PCLKDIV_1
  *                  - \ref ACMP_CTL_NFCLKS_PCLKDIV_2
  *                  - \ref ACMP_CTL_NFCLKS_PCLKDIV_4
  *                  - \ref ACMP_CTL_NFCLKS_PCLKDIV_16
  * @return None
  * @details When ACMP output filter function is enabled, the sampling frequency of the Noise Filter clock is determined by NFCLKS (ACMP_CTL[21:20]).
  * \hideinitializer  
  */
#define ACMP_SET_FILTER(acmp, u32ChNum, u32Clkdiv) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_NFCLKS_Msk) | (u32Clkdiv))

/**
  * @brief This macro is used to select ACMP interrupt condition
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Cond Comparator interrupt condition selection. Including:  
  *                  - \ref ACMP_CTL_INTPOL_R
  *                  - \ref ACMP_CTL_INTPOL_F
  *                  - \ref ACMP_CTL_INTPOL_RF
  * @return None
  * @details The ACMP output interrupt condition can be rising edge, falling edge or any edge.
  * \hideinitializer  
  */
#define ACMP_SELECT_INT_COND(acmp, u32ChNum, u32Cond) ((acmp)->CTL[u32ChNum] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_EDGESEL_Msk) | (u32Cond))



/* Function prototype declaration */
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);



/*@}*/ /* end of group NM1230_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_ACMP_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

#ifdef __cplusplus
}
#endif


#endif 

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
