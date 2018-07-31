/**************************************************************************//**
 * @file     wwdt_reg.h
 * @version  V1.00
 * @brief    WWDT register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __WWDT_REG_H__
#define __WWDT_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*---------------------- Window Watchdog Timer -------------------------*/
/**
    @addtogroup WWDT Window Watchdog Timer (WWDT)
    Memory Mapped Structure for WWDT Controller
@{ */


typedef struct
{


/**
 * @var WWDT_T::WWDTRLD
 * Offset: 0x00  Window Watchdog Timer Reload Counter Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |WWDTRLD   |WWDT Reload Counter Register
 * |        |          |Writing 0x00005AA5 to this register will reload the WWDT counter value to 0x3F.
 * |        |          |Note: User can only write WWDTRLD to reload WWDT counter value when current WWDT
 * |        |          | counter value between 0 and WINCMP. If user writes WWDTRLD when current WWDT
 * |        |          | counter value is larger than WINCMP, WWDT reset signal will generate immediately.
 * @var WWDT_T::WWDTCR
 * Offset: 0x04  Window Watchdog Timer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WWDTEN    |WWDT Enable Control
 * |        |          |0 = WWDT counter is stopped.
 * |        |          |1 = WWDT counter is starting counting.
 * |[1]     |WWDTIE    |WWDT Interrupt Enable Control
 * |        |          |If this bit is enabled, the WWDT counter compare match interrupt signal is generated and inform
 * |        |          |to CPU.
 * |        |          |0 = WWDT counter compare match interrupt Disabled.
 * |        |          |1 = WWDT counter compare match interrupt Enabled.
 * |[11:8]  |PERIODSEL |WWDT Counter Prescale Period Selection
 * |        |          |0000 = Pre-scale is 1; Max time-out period is 1 * 64 * TWWDT.
 * |        |          |0001 = Pre-scale is 2; Max time-out period is 2 * 64 * TWWDT.
 * |        |          |0010 = Pre-scale is 4; Max time-out period is 4 * 64 * TWWDT.
 * |        |          |0011 = Pre-scale is 8; Max time-out period is 8 * 64 * TWWDT.
 * |        |          |0100 = Pre-scale is 16; Max time-out period is 16 * 64 * TWWDT.
 * |        |          |0101 = Pre-scale is 32; Max time-out period is 32 * 64 * TWWDT.
 * |        |          |0110 = Pre-scale is 64; Max time-out period is 64 * 64 * TWWDT.
 * |        |          |0111 = Pre-scale is 128; Max time-out period is 128 * 64 * TWWDT.
 * |        |          |1000 = Pre-scale is 192; Max time-out period is 192 * 64 * TWWDT.
 * |        |          |1001 = Pre-scale is 256; Max time-out period is 256 * 64 * TWWDT.
 * |        |          |1010 = Pre-scale is 384; Max time-out period is 384 * 64 * TWWDT.
 * |        |          |1011 = Pre-scale is 512; Max time-out period is 512 * 64 * TWWDT.
 * |        |          |1100 = Pre-scale is 768; Max time-out period is 768 * 64 * TWWDT.
 * |        |          |1101 = Pre-scale is 1024; Max time-out period is 1024 * 64 * TWWDT.
 * |        |          |1110 = Pre-scale is 1536; Max time-out period is 1536 * 64 * TWWDT.
 * |        |          |1111 = Pre-scale is 2048; Max time-out period is 2048 * 64 * TWWDT.
 * |[21:16] |WINCMP    |WWDT Window Compare Register
 * |        |          |Set this register to adjust the valid reload window.
 * |        |          |Note: User can only write WWDTRLD to reload WWDT counter value when current WWDT counter value
 * |        |          |between 0 and WINCMP.
 * |        |          |If user writes WWDTRLD when current WWDT counter value larger than WINCMP, WWDT reset signal
 * |        |          |will generate immediately.
 * |[31]    |DBGACK_WWDT|ICE Debug Mode Acknowledge Disable Control
 * |        |          |0 = ICE debug mode acknowledgement effects WWDT counting.
 * |        |          |WWDT down counter will be held while CPU is held by ICE.
 * |        |          |1 = ICE debug mode acknowledgement Disabled.
 * |        |          |WWDT down counter will keep going no matter CPU is held by ICE or not.
 * @var WWDT_T::WWDTSR
 * Offset: 0x08  Window Watchdog Timer Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WWDTIF    |WWDT Compare Match Interrupt Flag
 * |        |          |This bit indicates the interrupt flag status of WWDT while WWDT counter value matches WINCMP
 * |        |          |value.
 * |        |          |0 = No effect.
 * |        |          |1 = WWDT counter value matches WINCMP value.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * |[1]     |WWDTRF    |WWDT Time-out Reset Flag
 * |        |          |This bit indicates the system has been reset by WWDT time-out reset or not.
 * |        |          |0 = WWDT time-out reset did not occur.
 * |        |          |1 = WWDT time-out reset occurred.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * @var WWDT_T::WWDTCVR
 * Offset: 0x0C  Window Watchdog Timer Counter Value Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |WWDTCVAL  |WWDT Counter Value
 * |        |          |WWDTCVAL will be updated continuously to monitor 6-bit down counter value.
 */

    __IO uint32_t WWDTRLD;       /* Offset: 0x00  Window Watchdog Timer Reload Counter Register                      */
    __IO uint32_t WWDTCR;        /* Offset: 0x04  Window Watchdog Timer Control Register                             */
    __IO uint32_t WWDTSR;        /* Offset: 0x08  Window Watchdog Timer Status Register                              */
    __I  uint32_t WWDTCVR;       /* Offset: 0x0C  Window Watchdog Timer Counter Value Register                       */

} WWDT_T;


/**
    @addtogroup WWDT_CONST WWDT Bit Field Definition
    Constant Definitions for WWDT Controller
@{ */



/* WWDT WWDTRLD Bit Field Definitions */
#define WWDT_WWDTRLD_WWDTRLD_Pos    0                                           /*!< WWDT_T::WWDTRLD: WWDTRLD Position */
#define WWDT_WWDTRLD_WWDTRLD_Msk    (0xFFFFFFFFul << WWDT_WWDTRLD_WWDTRLD_Pos)  /*!< WWDT_T::WWDTRLD: WWDTRLD Mask */

/* WWDT WWDTCR Bit Field Definitions */
#define WWDT_WWDTCR_DBGACK_WWDT_Pos 31                                          /*!< WWDT_T::WWDTCR: DBGACK_WWDT Position */
#define WWDT_WWDTCR_DBGACK_WWDT_Msk (1ul << WWDT_WWDTCR_DBGACK_WWDT_Pos)        /*!< WWDT_T::WWDTCR: DBGACK_WWDT Mask */

#define WWDT_WWDTCR_WINCMP_Pos      16                                          /*!< WWDT_T::WWDTCR: WINCMP Position */
#define WWDT_WWDTCR_WINCMP_Msk      (0x3Ful << WWDT_WWDTCR_WINCMP_Pos)          /*!< WWDT_T::WWDTCR: WINCMP Mask */

#define WWDT_WWDTCR_PERIODSEL_Pos   8                                           /*!< WWDT_T::WWDTCR: PERIODSEL Position */
#define WWDT_WWDTCR_PERIODSEL_Msk   (0xFul << WWDT_WWDTCR_PERIODSEL_Pos)        /*!< WWDT_T::WWDTCR: PERIODSEL Mask */

#define WWDT_WWDTCR_WWDTIE_Pos      1                                           /*!< WWDT_T::WWDTCR: WWDTIE Position */
#define WWDT_WWDTCR_WWDTIE_Msk      (1ul << WWDT_WWDTCR_WWDTIE_Pos)             /*!< WWDT_T::WWDTCR: WWDTIE Mask */

#define WWDT_WWDTCR_WWDTEN_Pos      0                                           /*!< WWDT_T::WWDTCR: WWDTEN Position */
#define WWDT_WWDTCR_WWDTEN_Msk      (1ul << WWDT_WWDTCR_WWDTEN_Pos)             /*!< WWDT_T::WWDTCR: WWDTEN Mask */

/* WWDT WWDTSR Bit Field Definitions */
#define WWDT_WWDTSR_WWDTRF_Pos      1                                           /*!< WWDT_T::WWDTSR: WWDTRF Position */
#define WWDT_WWDTSR_WWDTRF_Msk      (1ul << WWDT_WWDTSR_WWDTRF_Pos)             /*!< WWDT_T::WWDTSR: WWDTRF Mask */

#define WWDT_WWDTSR_WWDTIF_Pos      0                                           /*!< WWDT_T::WWDTSR: WWDTIF Position */
#define WWDT_WWDTSR_WWDTIF_Msk      (1ul << WWDT_WWDTSR_WWDTIF_Pos)             /*!< WWDT_T::WWDTSR: WWDTIF Mask */

/* WWDT WWDTCVR Bit Field Definitions */
#define WWDT_WWDTCVR_WWDTCVAL_Pos   0                                           /*!< WWDT_T::WWDTCVR: WWDTCVAL Position */
#define WWDT_WWDTCVR_WWDTCVAL_Msk   (0x3Ful << WWDT_WWDTCVR_WWDTCVAL_Pos)       /*!< WWDT_T::WWDTCVR: WWDTCVAL Mask */
/*@}*/ /* end of group WWDT_CONST */
/*@}*/ /* end of group WWDT */
/**@}*/ /* end of REGISTER group */


#endif /* __WWDT_REG_H__ */
