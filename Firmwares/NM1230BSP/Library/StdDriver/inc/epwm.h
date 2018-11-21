/**************************************************************************//**
 * @file     epwm.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 2018/10/23 14:18 $
 * @brief    NM1230 EPWM driver header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __EPWM_H__
#define __EPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_EPWM_Driver EPWM Driver
  @{
*/

/** @addtogroup NM1230_EPWM_EXPORTED_CONSTANTS EPWM Exported Constants
  @{
*/
#define EPWM_CHANNEL_NUM                     (6)        /*!< EPWM channel number */
#define EPWM_CH_0_MASK                       (0x1UL)    /*!< EPWM channel 0 mask */
#define EPWM_CH_1_MASK                       (0x2UL)    /*!< EPWM channel 1 mask */
#define EPWM_CH_2_MASK                       (0x4UL)    /*!< EPWM channel 2 mask */
#define EPWM_CH_3_MASK                       (0x8UL)    /*!< EPWM channel 3 mask */
#define EPWM_CH_4_MASK                       (0x10UL)   /*!< EPWM channel 4 mask */
#define EPWM_CH_5_MASK                       (0x20UL)   /*!< EPWM channel 5 mask */

/*---------------------------------------------------------------------------------------------------------*/
/*  Clock Divider Constant Definitions                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_CLK_DIV_1                       (0UL)    /*!< EPWM clock divide by 1 */
#define EPWM_CLK_DIV_2                       (1UL)    /*!< EPWM clock divide by 2 */
#define EPWM_CLK_DIV_4                       (2UL)    /*!< EPWM clock divide by 4 */
#define EPWM_CLK_DIV_8                       (3UL)    /*!< EPWM clock divide by 8 */
#define EPWM_CLK_DIV_16                      (4UL)    /*!< EPWM clock divide by 16 */
#define EPWM_CLK_DIV_32                      (5UL)    /*!< EPWM clock divide by 32 */
#define EPWM_CLK_DIV_64                      (6UL)    /*!< EPWM clock divide by 64 */
#define EPWM_CLK_DIV_128                     (7UL)    /*!< EPWM clock divide by 128 */
#define EPWM_CLK_DIV_256                     (8UL)    /*!< EPWM clock divide by 256 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Aligned Type Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_EDGE_ALIGNED                    (0UL)                       /*!< EPWM working in edge aligned type(down count) */
#define EPWM_CENTER_ALIGNED                  (EPWM_CTL_CNTTYPE_Msk)      /*!< EPWM working in center aligned type */

/*---------------------------------------------------------------------------------------------------------*/
/*  Fail brake Control Constant Definitions                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_BRK0_BRKP0                      (EPWM_BRKCTL_BRK0PEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)       /*!< Brake0 signal source from external pin */
#define EPWM_BRK0_ACMP0                      (EPWM_BRKCTL_BRK0A0EN_Msk | EPWM_BRKCTL_BRK0EN_Msk)      /*!< Brake0 signal source from analog comparator 0 output */
#define EPWM_BRK0_ACMP1                      (EPWM_BRKCTL_BRK0A1EN_Msk | EPWM_BRKCTL_BRK0EN_Msk)      /*!< Brake0 signal source from analog comparator 1 output */
#define EPWM_BRK0_EADC                       (EPWM_BRKCTL_BK0ADCEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)      /*!< Brake0 signal source from analog EADC output */
#define EPWM_BRK1_BRKP1                      (EPWM_BRKCTL_BRK1PEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)       /*!< Brake1 signal source from external pin */
#define EPWM_BRK1_ACMP0                      (EPWM_BRKCTL_BRK1A0EN_Msk | EPWM_BRKCTL_BRK1EN_Msk)      /*!< Brake1 signal source from analog comparator 0 output */
#define EPWM_BRK1_ACMP1                      (EPWM_BRKCTL_BRK1A1EN_Msk | EPWM_BRKCTL_BRK1EN_Msk)      /*!< Brake1 signal source from analog comparator 1 output */
#define EPWM_BRK1_EADC                       (EPWM_BRKCTL_BK1ADCEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)      /*!< Brake1 signal source from analog EADC output */
#define EPWM_BRK1_LVDBK                      (EPWM_BRKCTL_LVDBKEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)       /*!< Brake1 signal source from level detect */

#define EPWM_BRK0_PIN0                       (EPWM_BRKCTL_BRKP0EN_Msk | EPWM_BRKCTL_BRK0PEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)   /*!< Brake0 signal source from external pin0 */
#define EPWM_BRK0_PIN1                       (EPWM_BRKCTL_BRKP1EN_Msk | EPWM_BRKCTL_BRK0PEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)   /*!< Brake0 signal source from external pin1 */
#define EPWM_BRK0_PIN2                       (EPWM_BRKCTL_BRKP2EN_Msk | EPWM_BRKCTL_BRK0PEN_Msk | EPWM_BRKCTL_BRK0EN_Msk)   /*!< Brake0 signal source from external pin2 */
#define EPWM_BRK1_PIN0                       (EPWM_BRKCTL_BRKP0EN_Msk | EPWM_BRKCTL_BRK1PEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)   /*!< Brake1 signal source from external pin0 */
#define EPWM_BRK1_PIN1                       (EPWM_BRKCTL_BRKP1EN_Msk | EPWM_BRKCTL_BRK1PEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)   /*!< Brake1 signal source from external pin1 */
#define EPWM_BRK1_PIN2                       (EPWM_BRKCTL_BRKP2EN_Msk | EPWM_BRKCTL_BRK1PEN_Msk | EPWM_BRKCTL_BRK1EN_Msk)   /*!< Brake1 signal source from external pin2 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Duty Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_DUTY_INT_DOWN_COUNT_MATCH_CMP   (EPWM_INTEN_CMPDIEN0_Msk)   /*!< EPWM duty interrupt triggered if down count match comparator */
#define EPWM_DUTY_INT_UP_COUNT_MATCH_CMP     (EPWM_INTEN_CMPUIEN0_Msk)   /*!< EPWM duty interrupt triggered if up down match comparator */

/*---------------------------------------------------------------------------------------------------------*/
/*  Period and Counter Load Mode Constant Definitions                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_HCUPDT_CNTER_PERIOD             (0UL)  /*!< EPWM load new period and counter when counter value equals period */
#define EPWM_HCUPDT_CNTER_ZERO               (1UL)  /*!< EPWM load new period and counter when counter value equals zero */
#define EPWM_HCUPDT_CNTER_HALF               (2UL)  /*!< EPWM load new period and counter when counter value equals both period and zero */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Brake Pin Definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_BRKPIN_P0                       (0UL)  /*!< EPWM external brake pin0 */
#define EPWM_BRKPIN_P1                       (1UL)  /*!< EPWM external brake pin1 */
#define EPWM_BRKPIN_P2                       (2UL)  /*!< EPWM external brake pin2 */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Noise Filter Clock Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_NFCLKSEL_PCLK                   (0UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/1. */
#define EPWM_NFCLKSEL_PCLKDIV2               (1UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/2. */
#define EPWM_NFCLKSEL_PCLKDIV4               (2UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/4. */
#define EPWM_NFCLKSEL_PCLKDIV8               (3UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/8. */
#define EPWM_NFCLKSEL_PCLKDIV16              (4UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/16. */
#define EPWM_NFCLKSEL_PCLKDIV32              (5UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/32. */
#define EPWM_NFCLKSEL_PCLKDIV64              (6UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/64. */
#define EPWM_NFCLKSEL_PCLKDIV128             (7UL << EPWM_BRKCTL_NFCLKSEL_Pos)  /*!< EPWM_BRKCTL setting for noise filter frequency clock is PCLK/128. */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Trigger Source of Phase Change Definitions                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_TRGSEL_TIMER0                   (0UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by Timer0 */
#define EPWM_TRGSEL_TIMER1                   (1UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by Timer1 */
#define EPWM_TRGSEL_ECAP                     (2UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by EACP */
#define EPWM_TRGSEL_HALLSTS                  (3UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by HALLSTS */
#define EPWM_TRGSEL_ACMP0                    (4UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by ACMP0 */
#define EPWM_TRGSEL_ACMP1                    (5UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by ACMP1 */
#define EPWM_TRGSEL_TIMER2                   (6UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change triggered by Timer2 */
#define EPWM_TRGSEL_DISABLE                  (7UL << EPWM_PHCHG_TRGSEL_Pos)  /*!< EPWM Phase change trigger disable */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Phase Chnage Trigger Source ACMP0 Positive Input Definitions                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_A0POSSEL_ACMP0_P0               (0UL << EPWM_PHCHG_A0POSSEL_Pos)  /*!< EPWM Phase change trigger source from ACMP0_P0 */
#define EPWM_A0POSSEL_ACMP0_P1               (1UL << EPWM_PHCHG_A0POSSEL_Pos)  /*!< EPWM Phase change trigger source from ACMP0_P1 */
#define EPWM_A0POSSEL_ACMP0_P2               (2UL << EPWM_PHCHG_A0POSSEL_Pos)  /*!< EPWM Phase change trigger source from ACMP0_P2 */

/*---------------------------------------------------------------------------------------------------------*/
/*  EPWM Phase Chnage Trigger Source ACMP1 Positive Input Definitions                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define EPWM_A1POSSEL_ACMP1_P0               (0UL << EPWM_PHCHG_A1POSSEL_Pos)  /*!< EPWM Phase change trigger source from ACMP1_P0 */
#define EPWM_A1POSSEL_ACMP1_P1               (1UL << EPWM_PHCHG_A1POSSEL_Pos)  /*!< EPWM Phase change trigger source from ACMP1_P1 */
#define EPWM_A1POSSEL_ACMP1_P2               (2UL << EPWM_PHCHG_A1POSSEL_Pos)  /*!< EPWM Phase change trigger source from ACMP1_P2 */


/*@}*/ /* end of group NM1230_EPWM_EXPORTED_CONSTANTS */


/** @addtogroup NM1230_EPWM_EXPORTED_FUNCTIONS EPWM Exported Functions
  @{
*/

/**
 * @brief This macro get the current value of EPWM perios interrupt down-counter data
 * @param[in] epwm The pointer of the specified EPWM module
 * @return Current value of EPWM perios interrupt down-counter data
 * @details This macro is used to get the current value EPWM perios interrupt down-counter data.
 * \hideinitializer
 */
#define EPWM_GET_IFDCDAT(epwm) ((epwm->IFA & EPWM_IFA_IFDAT_Msk) >> EPWM_IFA_IFDAT_Pos)

/**
 * @brief This macro disable the EPWM perios interrupt accumulation function
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable the EPWM perios interrupt accumulation function.
 * \hideinitializer
 */
#define EPWM_DISABLE_IFACC(epwm) (epwm->IFA &= ~EPWM_IFA_IFAEN_Msk)

/**
 * @brief This macro enable the EPWM perios interrupt accumulation function
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Cnt is period interrupt accumulation counter
 * @return None
 * @details This macro is used to enable the EPWM perios interrupt accumulation function.
 * \hideinitializer
 */
#define EPWM_ENABLE_IFACC(epwm, u32Cnt) (epwm->IFA = (EPWM_IFA_IFAEN_Msk | ((u32Cnt) << EPWM_IFA_IFCNT_Pos)))

/**
 * @brief This macro set the ACMP1 positive input of phase change source control by A1POSSEL in PHG
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to set the ACMP1 positive input of phase change source control by ACMP_CTL1.
 * \hideinitializer
 */
#define EPWM_SET_POTCTL1_A1POSSEL(epwm) (epwm->PHCHGALT |= EPWM_PHCHGALT_POSCTL1_Msk)

/**
 * @brief This macro set the ACMP1 positive input of phase change source control by ACMP_CTL1
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to set the ACMP1 positive input of phase change source control by ACMP_CTL1.
 * \hideinitializer
 */
#define EPWM_SET_POSCTL1_ACMPCTL1(epwm) (epwm->PHCHGALT &= ~EPWM_PHCHGALT_POSCTL1_Msk)

/**
 * @brief This macro set the ACMP0 positive input of phase change source control by A0POSSEL in PHG
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to set the ACMP0 positive input of phase change source control by ACMP_CTL0.
 * \hideinitializer
 */
#define EPWM_SET_POTCTL0_A0POSSEL(epwm) (epwm->PHCHGALT |= EPWM_PHCHGALT_POSCTL0_Msk)

/**
 * @brief This macro set the ACMP0 positive input of phase change source control by ACMPCTL0
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to set the ACMP0 positive input of phase change source control by ACMP_CTL0.
 * \hideinitializer
 */
#define EPWM_SET_POSCTL0_ACMPCTL0(epwm) (epwm->PHCHGALT &= ~EPWM_PHCHGALT_POSCTL0_Msk)

/**
 * @brief This macro enable ACMP1 as the next phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable ACMP1 as the next phase change source.
 * \hideinitializer
 */
#define EPWM_ENABLE_ACMP1_PHCHGNXTTRG(epwm) (epwm->PHCHGNXT |= EPWM_PHCHGNXT_ACMP1TEN_Msk)

/**
 * @brief This macro disable ACMP1 as the next phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable ACMP1 as the next phase change source.
 * \hideinitializer
 */
#define EPWM_DISABLE_ACMP1_PHCHGNXTTRG(epwm) (epwm->PHCHGNXT &= ~EPWM_PHCHGNXT_ACMP1TEN_Msk)

/**
 * @brief This macro enable ACMP0 as the next phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable ACMP0 as the next phase change source.
 * \hideinitializer
 */
#define EPWM_ENABLE_ACMP0_PHCHGNXTTRG(epwm) (epwm->PHCHGNXT |= EPWM_PHCHGNXT_ACMP0TEN_Msk)

/**
 * @brief This macro disable ACMP0 as the next phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable ACMP0 as the next phase change source.
 * \hideinitializer
 */
#define EPWM_DISABLE_ACMP0_PHCHGNXTTRG(epwm) (epwm->PHCHGNXT &= ~EPWM_PHCHGNXT_ACMP0TEN_Msk)

/**
 * @brief This macro set the positive input of next phase change trigger source ACMP1
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Sel is positive input of next phase change trigger source ACMP1. Valid values are
 *              - \ref EPWM_A1POSSEL_P0
 *              - \ref EPWM_A1POSSEL_P1
 *              - \ref EPWM_A1POSSEL_P2
 * @return None
 * @details This macro is used to set the positive input of next phase change trigger source ACMP1.
 * \hideinitializer
 */
#define EPWM_PHCHGNXT_A1POSSEL(epwm, u32Sel) ((epwm)->PHCHGNXT = ((epwm)->PHCHGNXT & ~EPWM_PHCHGNXT_A1POSSEL_Msk) | (u32Sel));

/**
 * @brief This macro set the positive input of next phase change trigger source ACMP0
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Sel is positive input of next phase change trigger source ACMP0. Valid values are
 *              - \ref EPWM_A0POSSEL_ACMP0_P0
 *              - \ref EPWM_A0POSSEL_P1
 *              - \ref EPWM_A0POSSEL_P2
 * @return None
 * @details This macro is used to set the positive input of next phase change trigger source ACMP0.
 * \hideinitializer
 */
#define EPWM_PHCHGNXT_A0POSSEL(epwm, u32Sel) ((epwm)->PHCHGNXT = ((epwm)->PHCHGNXT & ~EPWM_PHCHGNXT_A0POSSEL_Msk) | (u32Sel));

/**
 * @brief This macro set the trigger source of next phase change
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Sel is trigger source of next phase change. Valid values are
 *              - \ref EPWM_TRGSEL_TIMER0
 *              - \ref EPWM_TRGSEL_TIMER1
 *              - \ref EPWM_TRGSEL_ECAP
 *              - \ref EPWM_TRGSEL_HALLSTS
 *              - \ref EPWM_TRGSEL_ACMP0 
 *              - \ref EPWM_TRGSEL_ACMP1
 *              - \ref EPWM_TRGSEL_DISABLE
 * @return None
 * @details This macro is used to set the trigger source of next phase change.
 * \hideinitializer
 */
#define EPWM_PHCHGNXT_TRGSEL(epwm, u32Sel) ((epwm)->PHCHGNXT = ((epwm)->PHCHGNXT & ~EPWM_PHCHGNXT_TRGSEL_Msk) | (u32Sel));

/**
 * @brief This macro set predicted next HALL state
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32HS0 is predicted value of HALL state0 and will compatre with ECAP0.
 * @param[in] u32HS1 is predicted value of HALL state1 and will compatre with ECAP1.
 * @param[in] u32HS2 is predicted value of HALL state2 and will compatre with ECAP2.
 * @return None
 * @details This macro is used to set the predicted next HALL state.
 * \hideinitializer
 */
#define EPWM_PHCHGNXT_HALLSTS(epwm, u32HS0, u32HS1, u32HS2) \
    {\
        (epwm)->PHCHGNXT &= ~EPWM_PHCHGNXT_HALLSTS_Msk; \
        (epwm)->PHCHGNXT |= (u32HS0 & 0x01) << (EPWM_PHCHGNXT_HALLSTS_Pos);\
        (epwm)->PHCHGNXT |= (u32HS1 & 0x01) << (EPWM_PHCHGNXT_HALLSTS_Pos + 1);\
        (epwm)->PHCHGNXT |= (u32HS2 & 0x01) << (EPWM_PHCHGNXT_HALLSTS_Pos + 2);\
    }

/**
 * @brief This macro enable ACMP1 as the phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable ACMP1 as the phase change source.
 * \hideinitializer
 */
#define EPWM_ENABLE_ACMP1_PHCHGTRG(epwm) (epwm->PHCHG |= EPWM_PHCHG_ACMP1TEN_Msk)

/**
 * @brief This macro disable ACMP1 as the phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable ACMP1 as the phase change source.
 * \hideinitializer
 */
#define EPWM_DISABLE_ACMP1_PHCHGTRG(epwm) (epwm->PHCHG &= ~EPWM_PHCHG_ACMP1TEN_Msk)

/**
 * @brief This macro enable ACMP0 as the phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable ACMP0 as the phase change source.
 * \hideinitializer
 */
#define EPWM_ENABLE_ACMP0_PHCHGTRG(epwm) (epwm->PHCHG |= EPWM_PHCHG_ACMP0TEN_Msk)

/**
 * @brief This macro disable ACMP0 as the phase change source
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable ACMP0 as the phase change source.
 * \hideinitializer
 */
#define EPWM_DISABLE_ACMP0_PHCHGTRG(epwm) (epwm->PHCHG &= ~EPWM_PHCHG_ACMP0TEN_Msk)

/**
 * @brief This macro set the positive input of phase change trigger source ACMP1
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Sel is positive input of phase change trigger source ACMP1. Valid values are
 *              - \ref EPWM_A1POSSEL_ACMP1_P0
 *              - \ref EPWM_A1POSSEL_ACMP1_P1
 *              - \ref EPWM_A1POSSEL_ACMP1_P2
 * @return None
 * @details This macro is used to set the positive input of phase change trigger source ACMP1.
 * \hideinitializer
 */
#define EPWM_PHCHG_A1POSSEL(epwm, u32Sel) ((epwm)->PHCHG = ((epwm)->PHCHG & ~EPWM_PHCHG_A1POSSEL_Msk) | (u32Sel));

/**
 * @brief This macro set the positive input of phase change trigger source ACMP0
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Sel is positive input of phase change trigger source ACMP0. Valid values are
 *              - \ref EPWM_A0POSSEL_ACMP0_P0
 *              - \ref EPWM_A0POSSEL_ACMP0_P1
 *              - \ref EPWM_A0POSSEL_ACMP0_P2
 * @return None
 * @details This macro is used to set the positive input of phase change trigger source ACMP0.
 * \hideinitializer
 */
#define EPWM_PHCHG_A0POSSEL(epwm, u32Sel) ((epwm)->PHCHG = ((epwm)->PHCHG & ~EPWM_PHCHG_A0POSSEL_Msk) | (u32Sel));

/**
 * @brief This macro set the trigger source of phase change
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Sel is trigger source of phase change. Valid values are
 *              - \ref EPWM_TRGSEL_TIMER0
 *              - \ref EPWM_TRGSEL_TIMER1
 *              - \ref EPWM_TRGSEL_ECAP
 *              - \ref EPWM_TRGSEL_HALLSTS
 *              - \ref EPWM_TRGSEL_ACMP0 
 *              - \ref EPWM_TRGSEL_ACMP1
 *              - \ref EPWM_TRGSEL_DISABLE
 * @return None
 * @details This macro is used to set the trigger source of phase change.
 * \hideinitializer
 */
#define EPWM_PHCHG_TRGSEL(epwm, u32Sel) ((epwm)->PHCHG = ((epwm)->PHCHG & ~EPWM_PHCHG_TRGSEL_Msk) | (u32Sel));

/**
 * @brief This macro is used to set EPWM brake pin filter function
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Clkdiv is Noise Filter clock pre-divided. Valid values are
 *              - \ref EPWM_NFCLKSEL_PCLK      
 *              - \ref EPWM_NFCLKSEL_PCLKDIV2  
 *              - \ref EPWM_NFCLKSEL_PCLKDIV4  
 *              - \ref EPWM_NFCLKSEL_PCLKDIV8  
 *              - \ref EPWM_NFCLKSEL_PCLKDIV16 
 *              - \ref EPWM_NFCLKSEL_PCLKDIV32 
 *              - \ref EPWM_NFCLKSEL_PCLKDIV64 
 *              - \ref EPWM_NFCLKSEL_PCLKDIV128
 * @return None
 * @details When EPWM break pin is enabled, the sampling frequency of the Noise Filter clock is determined by BRKCTL (BRKCTL[22:20]).
 * \hideinitializer  
 */
#define EPWM_BRKCTL_NFCLKSEL(epwm, u32Clkdiv) \
                ((epwm)->BRKCTL = (((epwm)->BRKCTL & ~EPWM_BRKCTL_NFCLKSEL_Msk) | (u32Clkdiv)) | EPWM_BRKCTL_NFPEN_Msk)

/**
 * @brief This macro set the resume delay counter of when low voltage detect 
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Cnt is a 12-bit down-counter.
 * @return None
 * @details This macro is used to set the resume delay counter of when low voltage detect.
 * \hideinitializer  
 */
#define EPWM_SET_RESDLY(epwm, u32Cnt) (epwm->RESDLY = ((u32Cnt << EPWM_RESDLY_DELAY_Pos) & EPWM_RESDLY_DELAY_Msk))

/**
 * @brief This macro is used to get EPWM break0 lock PWM output status
 * @param[in] epwm The pointer of the specified EPWM module 
 * @return EPWM brake0 lock occurred (1) or not (0)
 * @details This macro will return EPWM break0 lock PWM output status.
 * \hideinitializer  
 */
#define EPWM_GET_BRK0LOCK(epwm) ((epwm->EPWM_INSTS & EPWM_INTSTS_BRK0LOCK_Msk)?(1):(0))

/**
 * @brief This macro is used to get EPWM break pin trigger status
 * @param[in] epwm The pointer of the specified EPWM module  
 * @param[in] u32Sel pin select of brake function. Valid values are
 *              - \ref EPWM_BRKPIN_P0
 *              - \ref EPWM_BRKPIN_P1
 *              - \ref EPWM_BRKPIN_P2
 * @return EPWM brake pin trigger occurred (1) or not (0)
 * @details This macro will return EPWM break pin trigger status.
 * \hideinitializer  
 */
#define EPWM_GET_BRKPINIF(epwm, u32Sel) ((epwm->EPWM_INSTS & (EPWM_INTSTS_BRKP0IF_Msk << u32Sel))?(1):(0))

/**
 * @brief This macro is used to get EPWM current counter value
 * @param[in] epwm The pointer of the specified EPWM module
 * @return EPWM current counter value
 * @details This macro will return EPWM current counter value.
 * \hideinitializer  
 */
#define EPWM_GET_CNT(epwm) (epwm->CNT & EPWM_CNT_CNT_Msk)

/**
 * @brief This macro enable asymmetric mode in Center-aligned mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable asymmetric mode in Center-aligned mode.
 * \hideinitializer
 */
#define EPWM_ENABLE_ASYMMETRIC_MODE(epwm) (epwm->CTL |= EPWM_CTL_ASYMEN_Msk)

/**
 * @brief This macro disable asymmetric mode in Center-aligned mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable asymmetric mode in Center-aligned mode.
 * \hideinitializer
 */
#define EPWM_DISABLE_ASYMMETRIC_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_ASYMEN_Msk)

/**
 * @brief This macro set the load mode of period and counter in Center-aligned mode. 
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32Mode load mode of period and counter. Valid values are
 *              - \ref EPWM_HCUPDT_CNTER_PERIOD
 *              - \ref EPWM_HCUPDT_CNTER_ZERO
 *              - \ref EPWM_HCUPDT_CNTER_HALF 
 * @return None
 * @details This macro is used to set the load mode of period and counter in Center-aligned mode.
 * \hideinitializer
 */
#define EPWM_SET_HCUPDT(epwm, u32Mode) \
    (epwm->CTL = (epwm->CLKDIV & ~(EPWM_CTL_HCUPDT_Msk)) | (u32Mode))

/**
 * @brief This macro enable complementary mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable complementary mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_ENABLE_COMPLEMENTARY_MODE(epwm) (epwm->CTL = (epwm->CTL & ~EPWM_CTL_MODE_Msk) |(1UL << EPWM_CTL_MODE_Pos))

/**
 * @brief This macro disable complementary mode, and enable independent mode.
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable complementary mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_DISABLE_COMPLEMENTARY_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_MODE_Msk)

/**
 * @brief This macro enable group mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable group mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_ENABLE_GROUP_MODE(epwm) (epwm->CTL |= EPWM_CTL_GROUPEN_Msk)

/**
 * @brief This macro disable group mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable group mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_DISABLE_GROUP_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_GROUPEN_Msk)

/**
 * @brief This macro enable synchronous mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to enable synchronous mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_ENABLE_SYNC_MODE(epwm) (epwm->CTL = (epwm->CTL & ~EPWM_CTL_MODE_Msk) |(2UL << EPWM_CTL_MODE_Pos))

/**
 * @brief This macro disable synchronous mode, and enable independent mode.
 * @param[in] epwm The pointer of the specified EPWM module
 * @return None
 * @details This macro is used to disable synchronous mode of EPWM module.
 * \hideinitializer
 */
#define EPWM_DISABLE_SYNC_MODE(epwm) (epwm->CTL &= ~EPWM_CTL_MODE_Msk)

/**
 * @brief This macro enable output inverter of specified channel(s)
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * @details This macro is used to enable output inverter of specified channel(s).
 * \hideinitializer
 */
#define EPWM_ENABLE_OUTPUT_INVERTER(epwm, u32ChannelMask) ((epwm)->NPCTL = (u32ChannelMask))

/**
 * @brief This macro mask output logic to high or low
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @param[in] u32LevelMask Output logic to high or low
 * @return None
 * @details This macro is used to mask output logic to high or low of specified channel(s).
 * @note If u32ChannelMask parameter is 0, then mask function will be disabled.
 * \hideinitializer
 */
#define EPWM_MASK_OUTPUT(epwm, u32ChannelMask, u32LevelMask) \
    { \
        (epwm)->PHCHG = ((epwm)->PHCHG & ~(0x3F << EPWM_PHCHG_MSKEN0_Pos)) | (u32ChannelMask << EPWM_PHCHG_MSKEN0_Pos); \
        (epwm)->PHCHG = ((epwm)->PHCHG & ~(0x3F << EPWM_PHCHG_MSKDAT0_Pos)) | (u32LevelMask << EPWM_PHCHG_MSKDAT0_Pos); \
    }

/**
 * @brief This macro set the divider of the EPWM channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref EPWM_CLK_DIV_1
 *              - \ref EPWM_CLK_DIV_2
 *              - \ref EPWM_CLK_DIV_4
 *              - \ref EPWM_CLK_DIV_8
 *              - \ref EPWM_CLK_DIV_16
 *              - \ref EPWM_CLK_DIV_32
 *              - \ref EPWM_CLK_DIV_64
 *              - \ref EPWM_CLK_DIV_128
 *              - \ref EPWM_CLK_DIV_256
 * @return None
 * @details This macro is used to set the divider of the EPWM channel.
 * \hideinitializer
 */
#define EPWM_SET_DIVIDER(epwm, u32ChannelNum, u32Divider) \
    (epwm->CLKDIV = (epwm->CLKDIV & ~(EPWM_CLKDIV_CLKDIV_Msk)) | (u32Divider))

/**
 * @brief This macro set the comparator for up counter in Center-aligned mode
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @param[in] u32CMUR Comparator for up counter in Center-aligned mode.
              Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set the comparator for up counter in Center-aligned mode
 * @note This new setting will take effect on next EPWM period.
 * \hideinitializer
 */
#define EPWM_SET_CMUR(epwm, u32ChannelNum, u32CMUR) ((epwm)->CMPDAT[(u32ChannelNum)]= (EPWM_SET_CMUR)<<EPWM_CMPDAT0_CMPU_Pos)

/**
 * @brief This macro set the comparator of the selected channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum EPWM channel number. Valid values are between 0~5
 * @param[in] u32CMR Comparator of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set the comparator of specified channel.
 * @note This new setting will take effect on next EPWM period.
 * \hideinitializer
 */
#define EPWM_SET_CMR(epwm, u32ChannelNum, u32CMR) ((epwm)->CMPDAT[(u32ChannelNum)]= (u32CMR))

/**
 * @brief This macro set the period of the EPWM channel
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelNum This parameter is not used
 * @param[in] u32CNR Period of EPWM channel. Valid values are between 0~0xFFFF
 * @return None
 * @details This macro is used to set the period of EPWM channel.
 * @note This new setting will take effect on next EPWM period.
 * @note EPWM counter will stop if period length set to 0.
 * \hideinitializer
 */
#define EPWM_SET_CNR(epwm, u32ChannelNum, u32CNR)  ((epwm)->PERIOD = (u32CNR))

/**
 * @brief This macro set the EPWM aligned type
 * @param[in] epwm The pointer of the specified EPWM module
 * @param[in] u32ChannelMask This parameter is not used
 * @param[in] u32AlignedType EPWM aligned type, valid values are:
 *              - \ref EPWM_EDGE_ALIGNED
 *              - \ref EPWM_CENTER_ALIGNED
 * @return None
 * @details This macro is used to set the EPWM aligned type.
 * \hideinitializer
 */
#define EPWM_SET_ALIGNED_TYPE(epwm, u32ChannelMask, u32AlignedType) \
   (epwm->CTL = (epwm->CTL & ~EPWM_CTL_CNTTYPE_Msk) | (u32AlignedType))


/*---------------------------------------------------------------------------------------------------------*/
/* Define EPWM functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void EPWM_DisableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);


/*@}*/ /* end of group NM1230_EPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_EPWM_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
