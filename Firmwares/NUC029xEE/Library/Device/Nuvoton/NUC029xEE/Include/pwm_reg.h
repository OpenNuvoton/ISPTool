/**************************************************************************//**
 * @file     pwm_reg.h
 * @version  V1.00
 * @brief    PWM register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PWM_REG_H__
#define __PWM_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */




/*---------------------- Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup PWM Pulse Width Modulation Controller (PWM)
    Memory Mapped Structure for PWM Controller
@{ */



typedef struct
{


/**
 * @var PWM_T::PPR
 * Offset: 0x00  PWM Prescaler Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits     |Field     |Descriptions
 * | :----:  | :----:   | :---- |
 * |[7:0]    |CP01      |Clock Prescaler 0 (PWM-Timer 0 / 1 For Group A And PWM-Timer 4 / 5 For Group B)
 * |         |          |Clock input is divided by (CP01 + 1) before it is fed to the corresponding PWM-timer
 * |         |          |If CP01=0, then the clock prescaler 0 output clock will be stopped.
 * |         |          |So corresponding PWM-timer will also be stopped.
 * |[15:8]   |CP23      |Clock Prescaler 2 (PWM-Timer2 / 3 For Group A And PWM-Timer 6 / 7 For Group B)
 * |         |          |Clock input is divided by (CP23 + 1) before it is fed to the corresponding PWM-timer
 * |         |          |If CP23=0, then the clock prescaler 2 output clock will be stopped.
 * |         |          |So corresponding PWM-timer will also be stopped.
 * |[23:16]  |DZI01     |Dead-Zone Interval For Pair Of Channel 0 And Channel 1 (PWM0 And PWM1 Pair For PWM Group A, PWM4 And PWM5 Pair For PWM Group B)
 * |         |          |These 8-bit determine the Dead-zone length.
 * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWMxy_CLK (where xy
 * |         |          |could be 01 or 45, depends on selected PWM channel.).
 * |[31:24]  |DZI23     |Dead-Zone Interval For Pair Of Channel2 And Channel3 (PWM2 And PWM3 Pair For PWM Group A, PWM6 And PWM7 Pair For PWM Group B)
 * |         |          |These 8-bit determine the Dead-zone length.
 * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWMxy_CLK (where xy
 * |         |          |could be 23 or 67, depends on selected PWM channel.).
 * @var PWM_T::CSR
 * Offset: 0x04  PWM Clock Source Divider Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits      |Field     |Descriptions
 * | :----:   | :----:   | :---- |
 * |[2:0]     |CSR0      |Timer 0 Clock Source Selection(PWM timer 0 for group A and PWM timer 4 for group B)
 * |          |          |Select clock input for timer.
 * |          |          |(Table is the same as CSR3)
 * |[6:4]     |CSR1      |Timer 1 Clock Source Selection(PWM timer 1 for group A and PWM timer 5 for group B)
 * |          |          |Select clock input for timer.
 * |          |          |(Table is the same as CSR3)
 * |[10:8]    |CSR2      |Timer 2 Clock Source Selection(PWM timer 2 for group A and PWM timer 6 for group B)
 * |          |          |Select clock input for timer.
 * |          |          |(Table is the same as CSR3)
 * |[14:12]   |CSR3      |Timer 3 Clock Source Selection (PWM timer 3 for group A and PWM timer 7 for group B)
 * |          |          |Select clock input for timer.
 * |          |          |CSRx[2:0] = Input clock divider
 * |          |          |100 = 1
 * |          |          |011 = 16
 * |          |          |010 = 8
 * |          |          |001 = 4
 * |          |          |000 = 2
 * @var PWM_T::PCR
 * Offset: 0x08  PWM Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits     |Field         |Descriptions
 * | :----:  | :----:       | :---- |
 * |[0]      |CH0EN         |PWM-Timer 0 Enable (PWM Timer 0 For Group A And PWM Timer 4 For Group B)
 * |         |              |0 = The corresponding PWM-Timer stops running.
 * |         |              |1 = The corresponding PWM-Timer starts running.
 * |[1]      |CH0PINV       |PWM-Timer 0 Output Polar Inverse Enable (PWM Timer 0 For Group A And PWM Timer 4 For Group B)
 * |         |              |0 = PWM0 output polar inverse Disabled.
 * |         |              |1 = PWM0 output polar inverse Enabled.
 * |[2]      |CH0INV        |PWM-Timer 0 Output Inverter Enable (PWM Timer 0 For Group A And PWM Timer 4 For Group B)
 * |         |              |0 = Inverter Disabled.
 * |         |              |1 = Inverter Enabled.
 * |[3]      |CH0MOD        |PWM-Timer 0 Auto-Reload/One-Shot Mode (PWM Timer 0 For Group A And PWM Timer 4 For Group B)
 * |         |              |0 = One-shot mode.
 * |         |              |1 = Auto-reload mode.
 * |         |              |Note: If there is a transition at this bit, it will cause CNR0 and CMR0 be cleared.
 * |[4]      |DZEN01        |Dead-Zone 0 Generator Enable (PWM0 And PWM1 Pair For PWM Group A, PWM4 And PWM5 Pair For PWM Group B)
 * |         |              |0 = Disabled.
 * |         |              |1 = Enabled.
 * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary
 * |         |              |pair for PWM group A and the pair of PWM4 and PWM5 becomes a complementary pair for PWM group B.
 * |[5]      |DZEN23        |Dead-Zone 2 Generator Enable (PWM2 And PWM3 Pair For PWM Group A, PWM6 And PWM7 Pair For PWM Group B)
 * |         |              |0 = Disabled.
 * |         |              |1 = Enabled.
 * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM2 and PWM3 becomes a complementary
 * |         |              |pair for PWM group A and the pair of PWM6 and PWM7 becomes a complementary pair for PWM group B.
 * |[8]      |CH1EN         |PWM-Timer 1 Enable (PWM Timer 1 For Group A And PWM Timer 5 For Group B)
 * |         |              |0 = Corresponding PWM-Timer Stopped.
 * |         |              |1 = Corresponding PWM-Timer Start Running.
 * |[9]      |CH1PINV       |PWM-Timer 1 Output Polar Inverse Enable (PWM Timer 1 For Group A And PWM Timer 5 For Group B)
 * |         |              |0 = PWM1 output polar inverse Disabled.
 * |         |              |1 = PWM1 output polar inverse Enabled.
 * |[10]     |CH1INV        |PWM-Timer 1 Output Inverter Enable (PWM Timer 1 For Group A And PWM Timer 5 For Group B)
 * |         |              |0 = Inverter Disable.
 * |         |              |1 = Inverter Enable.
 * |[11]     |CH1MOD        |PWM-Timer 1 Auto-Reload/One-Shot Mode (PWM Timer 1 For Group A And PWM Timer 5 For Group B)
 * |         |              |0 = One-shot mode.
 * |         |              |1 = Auto-reload mode.
 * |         |              |Note: If there is a transition at this bit, it will cause CNR1 and CMR1 be cleared.
 * |[16]     |CH2EN         |PWM-Timer 2 Enable (PWM Timer 2 For Group A And PWM Timer 6 For Group B)
 * |         |              |0 = Corresponding PWM-Timer Stopped.
 * |         |              |1 = Corresponding PWM-Timer Start Running.
 * |[17]     |CH2PINV       |PWM-Timer 2 Output Polar Inverse Enable (PWM Timer 2 For Group A And PWM Timer 6 For Group B)
 * |         |              |0 = PWM2 output polar inverse Disabled.
 * |         |              |1 = PWM2 output polar inverse Enabled.
 * |[18]     |CH2INV        |PWM-Timer 2 Output Inverter Enable (PWM Timer 2 For Group A And PWM Timer 6 For Group B)
 * |         |              |0 = Inverter Disabled.
 * |         |              |1 = Inverter Enabled.
 * |[19]     |CH2MOD        |PWM-Timer 2 Auto-Reload/One-Shot Mode (PWM Timer 2 For Group A And PWM Timer 6 For Group B)
 * |         |              |0 = One-shot mode.
 * |         |              |1 = Auto-reload mode.
 * |         |              |Note: If there is a transition at this bit, it will cause CNR2 and CMR2 be cleared.
 * |[24]     |CH3EN         |PWM-Timer 3 Enable (PWM Timer 3 For Group A And PWM Timer 7 For Group B)
 * |         |              |0 = Corresponding PWM-Timer Stopped.
 * |         |              |1 = Corresponding PWM-Timer Start Running.
 * |[25]     |CH3PINV       |PWM-Timer 3 Output Polar Inverse Enable (PWM Timer 3 For Group A And PWM Timer 7 For Group B)
 * |         |              |0 = PWM3 output polar inverse Disable.
 * |         |              |1 = PWM3 output polar inverse Enable.
 * |[26]     |CH3INV        |PWM-Timer 3 Output Inverter Enable (PWM Timer 3 For Group A And PWM Timer 7 For Group B)
 * |         |              |0 = Inverter Disabled.
 * |         |              |1 = Inverter Enabled.
 * |[27]     |CH3MOD        |PWM-Timer 3 Auto-Reload/One-Shot Mode (PWM Timer 3 For Group A And PWM Timer 7 For Group B)
 * |         |              |0 = One-shot mode.
 * |         |              |1 = Auto-reload mode.
 * |         |              |Note: If there is a transition at this bit, it will cause CNR3 and CMR3 be cleared.
 * |[30]     |PWM01TYPE     |PWM01 Aligned Type Selection Bit (PWM0 And PWM1 Pair For PWM Group A, PWM4 And PWM5 Pair For PWM Group B)
 * |         |              |0 = Edge-aligned type.
 * |         |              |1 = Center-aligned type.
 * |[31]     |PWM23TYPE     |PWM23 Aligned Type Selection Bit (PWM2 And PWM3 Pair For PWM Group A, PWM6 And PWM7 Pair For PWM Group B)
 * |         |              |0 = Edge-aligned type.
 * |         |              |1 = Center-aligned type.
 * @var PWM_T::CNR0
 * Offset: 0x0C  PWM Counter Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNRx      |PWM Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to
 * |        |          |0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * |        |          |Note: When CNR value is set to 0, PWM output is always high.
 * @var PWM_T::CMR0
 * Offset: 0x0C  PWM Counter Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNRx      |PWM Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to
 * |        |          |0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * |        |          |Note: When CNR value is set to 0, PWM output is always high.
 * @var PWM_T::PDR0
 * Offset: 0x14  PWM Data Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDRx      |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit counter.
 * @var PWM_T::CNR1
 * Offset: 0x18  PWM Counter Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNRx      |PWM Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to
 * |        |          |0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * |        |          |Note: When CNR value is set to 0, PWM output is always high.
 * @var PWM_T::CMR1
 * Offset: 0x1C  PWM Comparator Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMRx      |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * @var PWM_T::PDR1
 * Offset: 0x20  PWM Data Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDRx      |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit counter.
 * @var PWM_T::CNR2
 * Offset: 0x24  PWM Counter Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNRx      |PWM Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to
 * |        |          |0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * |        |          |Note: When CNR value is set to 0, PWM output is always high.
 * @var PWM_T::CMR2
 * Offset: 0x28  PWM Comparator Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMRx      |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * @var PWM_T::PDR2
 * Offset: 0x2C  PWM Data Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDRx      |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit counter.
 * @var PWM_T::CNR3
 * Offset: 0x30  PWM Counter Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNRx      |PWM Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to
 * |        |          |0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * |        |          |Note: When CNR value is set to 0, PWM output is always high.
 * @var PWM_T::CMR3
 * Offset: 0x34  PWM Comparator Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMRx      |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01, 23, 45
 * |        |          |or 67, depends on selected PWM channel.
 * |        |          |For Edge-aligned type:
 * |        |          | Duty ratio = (CMR+1)/(CNR+1).
 * |        |          | CMR >= CNR: PWM output is always high.
 * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
 * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
 * |        |          |For Center-aligned type:
 * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
 * |        |          | CMR > CNR: PWM output is always high.
 * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
 * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
 * @var PWM_T::PDR3
 * Offset: 0x38  PWM Data Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDRx      |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit counter.
 * @var PWM_T::PBCR
 * Offset: 0x3C  PWM Backward Compatible Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BCn       |PWM Backward Compatible Register
 * |        |          |0 = Configure write 0 to clear CFLRI0~3 and CRLRI0~3.
 * |        |          |1 = Configure write 1 to clear CFLRI0~3 and CRLRI0~3.
 * |        |          |Refer to the CCR0/CCR2 register bit 6, 7, 22, 23 description
 * |        |          |Note: It is recommended that this bit be set to 1 to prevent CFLRIx and CRLRIx from being
 * |        |          |cleared when writing CCR0/CCR2.
 * @var PWM_T::PIER
 * Offset: 0x40  PWM Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWMIE0    |PWM Channel 0 Period Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[1]     |PWMIE1    |PWM Channel 1 Period Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[2]     |PWMIE2    |PWM Channel 2 Period Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[3]     |PWMIE3    |PWM Channel 3 Period Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[8]     |PWMDIE0   |PWM Channel 0 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[9]     |PWMDIE1   |PWM Channel 1 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[10]    |PWMDIE2   |PWM Channel 2 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[11]    |PWMDIE3   |PWM Channel 3 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[16]    |INT01TYPE |PWM01 Interrupt Period Type Selection Bit (PWM0 And PWM1 Pair For PWM Group A, PWM4 And PWM5 Pair For PWM Group B)
 * |        |          |0 = PWMIFn will be set if PWM counter underflow.
 * |        |          |1 = PWMIFn will be set if PWM counter matches CNRn register.
 * |        |          |Note: This bit is effective when PWM in Center-aligned type only.
 * |[17]    |INT23TYPE |PWM23 Interrupt Period Type Selection Bit (PWM2 And PWM3 Pair For PWM Group A, PWM6 And PWM7 Pair For PWM Group B)
 * |        |          |0 = PWMIFn will be set if PWM counter underflow.
 * |        |          |1 = PWMIFn will be set if PWM counter matches CNRn register.
 * |        |          |Note: This bit is effective when PWM in Center-aligned type only.
 * @var PWM_T::PIIR
 * Offset: 0x44  PWM Interrupt Indication Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWMIF0    |PWM Channel 0 Period Interrupt Status
 * |        |          |This bit is set by hardware when PWM0 counter reaches the requirement of interrupt (depend on
 * |        |          |INT01TYPE bit of PIER register), software can write 1 to clear this bit to 0.
 * |[1]     |PWMIF1    |PWM Channel 1 Period Interrupt Status
 * |        |          |This bit is set by hardware when PWM1 counter reaches the requirement of interrupt (depend on
 * |        |          |INT01TYPE bit of PIER register), software can write 1 to clear this bit to 0.
 * |[2]     |PWMIF2    |PWM Channel 2 Period Interrupt Status
 * |        |          |This bit is set by hardware when PWM2 counter reaches the requirement of interrupt (depend on
 * |        |          |INT23TYPE bit of PIER register), software can write 1 to clear this bit to 0.
 * |[3]     |PWMIF3    |PWM Channel 3 Period Interrupt Status
 * |        |          |This bit is set by hardware when PWM3 counter reaches the requirement of interrupt (depend on
 * |        |          |INT23TYPE bit of PIER register), software can write 1 to clear this bit to 0.
 * |[8]     |PWMDIF0   |PWM Channel 0 Duty Interrupt Flag
 * |        |          |Flag is set by hardware when channel 0 PWM counter down count and reaches CMR0, software can
 * |        |          |clear this bit by writing a one to it.
 * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
 * |[9]     |PWMDIF1   |PWM Channel 1 Duty Interrupt Flag
 * |        |          |Flag is set by hardware when channel 1 PWM counter down count and reaches CMR1, software can
 * |        |          |clear this bit by writing a one to it.
 * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
 * |[10]    |PWMDIF2   |PWM Channel 2 Duty Interrupt Flag
 * |        |          |Flag is set by hardware when channel 2 PWM counter down count and reaches CMR2, software can
 * |        |          |clear this bit by writing a one to it.
 * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
 * |[11]    |PWMDIF3   |PWM Channel 3 Duty Interrupt Flag
 * |        |          |Flag is set by hardware when channel 3 PWM counter down count and reaches CMR3, software can
 * |        |          |clear this bit by writing a one to it.
 * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
 * @var PWM_T::CCR0
 * Offset: 0x50  PWM Capture Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INV0      |Channel 0 Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
 * |[1]     |CRL_IE0   |Channel 0 Rising Latch Interrupt Enable
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 0 has rising transition, Capture will issue
 * |        |          |an Interrupt.
 * |[2]     |CFL_IE0   |Channel 0 Falling Latch Interrupt Enable
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 0 has falling transition, Capture will issue
 * |        |          |an Interrupt.
 * |[3]     |CAPCH0EN  |Channel 0 Capture Function Enable
 * |        |          |0 = Capture function on PWM group channel 0 Disabled.
 * |        |          |1 = Capture function on PWM group channel 0 Enabled.
 * |        |          |When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR
 * |        |          |(Falling latch).
 * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 0 Interrupt.
 * |[4]     |CAPIF0    |Channel 0 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 0 rising latch interrupt is enabled (CRL_IE0 = 1), a rising transition
 * |        |          |occurs at PWM group channel 0 will result in CAPIF0 to high; Similarly, a falling transition
 * |        |          |will cause CAPIF0 to be set high if PWM group channel 0 falling latch interrupt is enabled
 * |        |          |(CFL_IE0 = 1).
 * |        |          |Write 1 to clear this bit to 0.
 * |[6]     |CRLRI0    |CRLR0 Latched Indicator Bit
 * |        |          |When PWM group input channel 0 has a rising transition, CRLR0 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if the BCn bit is 0, and can write 1 to clear this
 * |        |          |bit to 0 if the BCn bit is 1.
 * |[7]     |CFLRI0    |CFLR0 Latched Indicator Bit
 * |        |          |When PWM group input channel 0 has a falling transition, CFLR0 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if the BCn bit is 0, and can write 1 to clear this
 * |        |          |bit to0 if BCn bit is 1.
 * |[16]    |INV1      |Channel 1 Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
 * |[17]    |CRL_IE1   |Channel 1 Rising Latch Interrupt Enable
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 1 has rising transition, Capture will issue
 * |        |          |an Interrupt.
 * |[18]    |CFL_IE1   |Channel 1 Falling Latch Interrupt Enable
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 1 has falling transition, Capture will issue
 * |        |          |an Interrupt.
 * |[19]    |CAPCH1EN  |Channel 1 Capture Function Enable
 * |        |          |0 = Capture function on PWM group channel 1 Disabled.
 * |        |          |1 = Capture function on PWM group channel 1 Enabled.
 * |        |          |When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling
 * |        |          |latch).
 * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 1 Interrupt.
 * |[20]    |CAPIF1    |Channel 1 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 1 rising latch interrupt is enabled (CRL_IE1 = 1), a rising transition
 * |        |          |occurs at PWM group channel 1 will result in CAPIF1 to high; Similarly, a falling transition
 * |        |          |will cause CAPIF1 to be set high if PWM group channel 1 falling latch interrupt is enabled
 * |        |          |(CFL_IE1 = 1).
 * |        |          |Write 1 to clear this bit to 0.
 * |[22]    |CRLRI1    |CRLR1 Latched Indicator Bit
 * |        |          |When PWM group input channel 1 has a rising transition, CRLR1 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if BCn bit is 0, and can write 1 to clear this bit
 * |        |          |to0 if BCn bit is 1.
 * |[23]    |CFLRI1    |CFLR1 Latched Indicator Bit
 * |        |          |When PWM group input channel 1 has a falling transition, CFLR1 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if BCn bit is 0, and can write 1 to clear this bit
 * |        |          |to 0 if BCn bit is 1.
 * @var PWM_T::CCR2
 * Offset: 0x54  PWM Capture Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INV2      |Channel 2 Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
 * |[1]     |CRL_IE2   |Channel 2 Rising Latch Interrupt Enable
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 2 has rising transition, Capture will issue
 * |        |          |an Interrupt.
 * |[2]     |CFL_IE2   |Channel 2 Falling Latch Interrupt Enable
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 2 has falling transition, Capture will issue
 * |        |          |an Interrupt.
 * |[3]     |CAPCH2EN  |Channel 2 Capture Function Enable
 * |        |          |0 = Capture function on PWM group channel 2 Disabled.
 * |        |          |1 = Capture function on PWM group channel 2 Enabled.
 * |        |          |When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR
 * |        |          |(Falling latch).
 * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 2 Interrupt.
 * |[4]     |CAPIF2    |Channel 2 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 2 rising latch interrupt is enabled (CRL_IE2=1), a rising transition occurs
 * |        |          |at PWM group channel 2 will result in CAPIF2 to high; Similarly, a falling transition will cause
 * |        |          |CAPIF2 to be set high if PWM group channel 2 falling latch interrupt is enabled (CFL_IE2=1).
 * |        |          |Write 1 to clear this bit to 0
 * |[6]     |CRLRI2    |CRLR2 Latched Indicator Bit
 * |        |          |When PWM group input channel 2 has a rising transition, CRLR2 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if the BCn bit is 0, and can write 1 to clear this
 * |        |          |bit to 0 if the BCn bit is 1.
 * |[7]     |CFLRI2    |CFLR2 Latched Indicator Bit
 * |        |          |When PWM group input channel 2 has a falling transition, CFLR2 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if BCn bit is 0, and can write 1 to clear this bit
 * |        |          |to 0 if the BCn bit is 1.
 * |[16]    |INV3      |Channel 3 Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
 * |[17]    |CRL_IE3   |Channel 3 Rising Latch Interrupt Enable
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 3 has rising transition, Capture will issue
 * |        |          |an Interrupt.
 * |[18]    |CFL_IE3   |Channel 3 Falling Latch Interrupt Enable
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |When Enabled, if Capture detects PWM group channel 3 has falling transition, Capture will issue
 * |        |          |an Interrupt.
 * |[19]    |CAPCH3EN  |Channel 3 Capture Function Enable
 * |        |          |0 = Capture function on PWM group channel 3 Disabled.
 * |        |          |1 = Capture function on PWM group channel 3 Enabled.
 * |        |          |When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling
 * |        |          |latch).
 * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 3 Interrupt.
 * |[20]    |CAPIF3    |Channel 3 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 3 rising latch interrupt is enabled (CRL_IE3=1), a rising transition occurs
 * |        |          |at PWM group channel 3 will result in CAPIF3 to high; Similarly, a falling transition will cause
 * |        |          |CAPIF3 to be set high if PWM group channel 3 falling latch interrupt is enabled (CFL_IE3=1).
 * |        |          |Write 1 to clear this bit to 0
 * |[22]    |CRLRI3    |CRLR3 Latched Indicator Bit
 * |        |          |When PWM group input channel 3 has a rising transition, CRLR3 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if the BCn bit is 0, and can write 1 to clear this
 * |        |          |bit to 0 if the BCn bit is 1.
 * |[23]    |CFLRI3    |CFLR3 Latched Indicator Bit
 * |        |          |When PWM group input channel 3 has a falling transition, CFLR3 was latched with the value of PWM
 * |        |          |down-counter and this bit is set by hardware.
 * |        |          |Software can write 0 to clear this bit to 0 if the BCn bit is 0, and can write 1 to clear this
 * |        |          |bit to 0 if the BCn bit is 1.
 * @var PWM_T::CRLR0
 * Offset: 0x58  PWM Capture Rising Latch Register (Channel 0)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLRx     |Capture Rising Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
 * @var PWM_T::CFLR0
 * Offset: 0x5C  PWM Capture Falling Latch Register (Channel 0)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLRx     |Capture Falling Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
 * @var PWM_T::CRLR1
 * Offset: 0x60  PWM Capture Rising Latch Register (Channel 1)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLRx     |Capture Rising Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
 * @var PWM_T::CFLR1
 * Offset: 0x64  PWM Capture Falling Latch Register (Channel 1)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLRx     |Capture Falling Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
 * @var PWM_T::CRLR2
 * Offset: 0x68  PWM Capture Rising Latch Register (Channel 2)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLRx     |Capture Rising Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
 * @var PWM_T::CFLR2
 * Offset: 0x6C  PWM Capture Falling Latch Register (Channel 2)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLRx     |Capture Falling Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
 * @var PWM_T::CRLR3
 * Offset: 0x70  PWM Capture Rising Latch Register (Channel 3)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLRx     |Capture Rising Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
 * @var PWM_T::CFLR3
 * Offset: 0x74  PWM Capture Falling Latch Register (Channel 3)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLRx     |Capture Falling Latch Register
 * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
 * @var PWM_T::CAPENR
 * Offset: 0x78  PWM Capture Input 0~3 Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CINEN0    |Channel 0 Capture Input Enable
 * |        |          |0 = PWM Channel 0 capture input path Disabled.
 * |        |          |The input of PWM channel 0 capture function is always regarded as 0.
 * |        |          |1 = PWM Channel 0 capture input path Enabled.
 * |        |          |The input of PWM channel 0 capture function comes from correlative multifunction pin if GPIO
 * |        |          |multi-function is set as PWM0.
 * |[1]     |CINEN1    |Channel 1 Capture Input Enable
 * |        |          |0 = PWM Channel 1 capture input path Disabled.
 * |        |          |The input of PWM channel 1 capture function is always regarded as 0.
 * |        |          |1 = PWM Channel 1 capture input path Enabled.
 * |        |          |The input of PWM channel 1 capture function comes from correlative multifunction pin if GPIO
 * |        |          |multi-function is set as PWM1.
 * |[2]     |CINEN2    |Channel 2 Capture Input Enable
 * |        |          |0 = PWM Channel 2 capture input path Disabled.
 * |        |          |The input of PWM channel 2 capture function is always regarded as 0.
 * |        |          |1 = PWM Channel 2 capture input path Enabled.
 * |        |          |The input of PWM channel 2 capture function comes from correlative multifunction pin if GPIO
 * |        |          |multi-function is set as PWM2.
 * |[3]     |CINEN3    |Channel 3 Capture Input Enable
 * |        |          |0 = PWM Channel 3 capture input path Disabled.
 * |        |          |The input of PWM channel 3 capture function is always regarded as 0.
 * |        |          |1 = PWM Channel 3 capture input path Enabled.
 * |        |          |The input of PWM channel 3 capture function comes from correlative multifunction pin if GPIO
 * |        |          |multi-function is set as PWM3.
 * @var PWM_T::POE
 * Offset: 0x7C  PWM Output Enable for Channel 0~3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |POE0      |Channel 0 Output Enable Register
 * |        |          |0 = PWM channel 0 output to pin Disabled.
 * |        |          |1 = PWM channel 0 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
 * |[1]     |POE1      |Channel 1 Output Enable Register
 * |        |          |0 = PWM channel 1 output to pin Disabled.
 * |        |          |1 = PWM channel 1 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
 * |[2]     |POE2      |Channel 2 Output Enable Register
 * |        |          |0 = PWM channel 2 output to pin Disabled.
 * |        |          |1 = PWM channel 2 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
 * |[3]     |POE3      |Channel 3 Output Enable Register
 * |        |          |0 = PWM channel 3 output to pin Disabled.
 * |        |          |1 = PWM channel 3 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
 * @var PWM_T::TCON
 * Offset: 0x80  PWM Trigger Control for Channel 0~3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWM0TEN   |Channel 0 Center-Aligned Trigger Enable Register
 * |        |          |0 = PWM channel 0 trigger ADC function Disabled.
 * |        |          |1 = PWM channel 0 trigger ADC function Enabled.
 * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to
 * |        |          |1.
 * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
 * |[1]     |PWM1TEN   |Channel 1 Center-Aligned Trigger Enable Register
 * |        |          |0 = PWM channel 1 trigger ADC function Disabled.
 * |        |          |1 = PWM channel 1 trigger ADC function Enabled.
 * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to
 * |        |          |1.
 * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
 * |[2]     |PWM2TEN   |Channel 2 Center-Aligned Trigger Enable Register
 * |        |          |0 = PWM channel 2 trigger ADC function Disabled.
 * |        |          |1 = PWM channel 2 trigger ADC function Enabled.
 * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to
 * |        |          |1.
 * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
 * |[3]     |PWM3TEN   |Channel 3 Center-Aligned Trigger Enable Register
 * |        |          |0 = PWM channel 3 trigger ADC function Disabled.
 * |        |          |1 = PWM channel 3 trigger ADC function Enabled.
 * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to
 * |        |          |1.
 * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
 * @var PWM_T::TSTATUS
 * Offset: 0x84  PWM Trigger Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWM0TF    |Channel 0 Center-Aligned Trigger Flag
 * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up counts
 * |        |          |to CNR if PWM0TEN bit is set to 1.
 * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by
 * |        |          |PWM.
 * |        |          |Software can write 1 to clear this bit.
 * |[1]     |PWM1TF    |Channel 1 Center-Aligned Trigger Flag
 * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up count to
 * |        |          |CNR if PWM1TEN bit is set to 1.
 * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by
 * |        |          |PWM.
 * |        |          |Software can write 1 to clear this bit.
 * |[2]     |PWM2TF    |Channel 2 Center-Aligned Trigger Flag
 * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up count to
 * |        |          |CNR if PWM2TEN bit is set to 1.
 * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by
 * |        |          |PWM.
 * |        |          |Software can write 1 to clear this bit.
 * |[3]     |PWM3TF    |Channel 3 Center-Aligned Trigger Flag
 * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up count to
 * |        |          |CNR if PWM3TEN bit is set to 1.
 * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by
 * |        |          |PWM.
 * |        |          |Software can write 1 to clear this bit.
 * @var PWM_T::SYNCBUSY0
 * Offset: 0x88  PWM0 Synchronous Busy Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |S_BUSY    |PWM Synchronous Busy
 * |        |          |When software writes CNR0/CMR0/PPR or switches PWM0 operation mode (PCR[3]), PWM will have a
 * |        |          |busy time to update these values completely because PWM clock may be different from system clock
 * |        |          |domain.
 * |        |          |Software needs to check this busy status before writing CNR0/CMR0/PPR or switching PWM0
 * |        |          |operation mode (PCR[3]) to make sure previous setting has been updated completely.
 * |        |          |This bit will be set when software writes CNR0/CMR0/PPR or switches PWM0 operation mode (PCR[3])
 * |        |          |and will be cleared by hardware automatically when PWM update these value completely.
 * @var PWM_T::SYNCBUSY1
 * Offset: 0x8C  PWM1 Synchronous Busy Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |S_BUSY    |PWM Synchronous Busy
 * |        |          |When Software writes CNR1/CMR1/PPR or switches PWM1 operation mode (PCR[11]), PWM will have a
 * |        |          |busy time to update these values completely because PWM clock may be different from system clock
 * |        |          |domain.
 * |        |          |Software needs to check this busy status before writing CNR1/CMR1/PPR or switching PWM1
 * |        |          |operation mode (PCR[11]) to make sure previous setting has been updated completely.
 * |        |          |This bit will be set when software writes CNR1/CMR1/PPR or switches PWM1 operation mode
 * |        |          |(PCR[11]) and will be cleared by hardware automatically when PWM update these value completely.
 * @var PWM_T::SYNCBUSY2
 * Offset: 0x90  PWM2 Synchronous Busy Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |S_BUSY    |PWM Synchronous Busy
 * |        |          |When Software writes CNR2/CMR2/PPR or switch PWM2 operation mode (PCR[19]), PWM will have a busy
 * |        |          |time to update these values completely because PWM clock may be different from system clock
 * |        |          |domain.
 * |        |          |Software needs to check this busy status before writing CNR2/CMR2/PPR or switching PWM2
 * |        |          |operation mode (PCR[19]) to make sure previous setting has been updated completely.
 * |        |          |This bit will be set when software writes CNR2/CMR2/PPR or switch PWM2 operation mode (PCR[19])
 * |        |          |and will be cleared by hardware automatically when PWM update these value completely.
 * @var PWM_T::SYNCBUSY3
 * Offset: 0x94  PWM3 Synchronous Busy Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |S_BUSY    |PWM Synchronous Busy
 * |        |          |When Software writes CNR3/CMR3/PPR or switch PWM3 operation mode (PCR[27]), PWM will have a busy
 * |        |          |time to update these values completely because PWM clock may be different from system clock
 * |        |          |domain.
 * |        |          |Software need to check this busy status before writing CNR3/CMR3/PPR or switching PWM3 operation
 * |        |          |mode (PCR[27]) to make sure previous setting has been updated completely.
 * |        |          |This bit will be set when Software writes CNR3/CMR3/PPR or switch PWM3 operation mode (PCR[27])
 * |        |          |and will be cleared by hardware automatically when PWM update these value completely.
 */

    __IO uint32_t PPR;           /* Offset: 0x00  PWM Prescaler Register                                             */
    __IO uint32_t CSR;           /* Offset: 0x04  PWM Clock Source Divider Select Register                           */
    __IO uint32_t PCR;           /* Offset: 0x08  PWM Control Register                                               */
    __IO uint32_t CNR0;          /* Offset: 0x0C  PWM Counter Register 0                                             */
    __IO uint32_t CMR0;          /* Offset: 0x0C  PWM Counter Register 0                                             */
    __I  uint32_t PDR0;          /* Offset: 0x14  PWM Data Register 0                                                */
    __IO uint32_t CNR1;          /* Offset: 0x18  PWM Counter Register 1                                             */
    __IO uint32_t CMR1;          /* Offset: 0x1C  PWM Comparator Register 1                                          */
    __I  uint32_t PDR1;          /* Offset: 0x20  PWM Data Register 1                                                */
    __IO uint32_t CNR2;          /* Offset: 0x24  PWM Counter Register 2                                             */
    __IO uint32_t CMR2;          /* Offset: 0x28  PWM Comparator Register 2                                          */
    __I  uint32_t PDR2;          /* Offset: 0x2C  PWM Data Register 2                                                */
    __IO uint32_t CNR3;          /* Offset: 0x30  PWM Counter Register 3                                             */
    __IO uint32_t CMR3;          /* Offset: 0x34  PWM Comparator Register 3                                          */
    __I  uint32_t PDR3;          /* Offset: 0x38  PWM Data Register 3                                                */
    __IO uint32_t PBCR;          /* Offset: 0x3C  PWM Backward Compatible Register                                   */
    __IO uint32_t PIER;          /* Offset: 0x40  PWM Interrupt Enable Register                                      */
    __IO uint32_t PIIR;          /* Offset: 0x44  PWM Interrupt Indication Register                                  */
    __I  uint32_t RESERVE1[2];  
    __IO uint32_t CCR0;          /* Offset: 0x50  PWM Capture Control Register 0                                     */
    __IO uint32_t CCR2;          /* Offset: 0x54  PWM Capture Control Register 2                                     */
    __IO uint32_t CRLR0;         /* Offset: 0x58  PWM Capture Rising Latch Register (Channel 0)                      */
    __IO uint32_t CFLR0;         /* Offset: 0x5C  PWM Capture Falling Latch Register (Channel 0)                     */
    __IO uint32_t CRLR1;         /* Offset: 0x60  PWM Capture Rising Latch Register (Channel 1)                      */
    __IO uint32_t CFLR1;         /* Offset: 0x64  PWM Capture Falling Latch Register (Channel 1)                     */
    __IO uint32_t CRLR2;         /* Offset: 0x68  PWM Capture Rising Latch Register (Channel 2)                      */
    __IO uint32_t CFLR2;         /* Offset: 0x6C  PWM Capture Falling Latch Register (Channel 2)                     */
    __IO uint32_t CRLR3;         /* Offset: 0x70  PWM Capture Rising Latch Register (Channel 3)                      */
    __IO uint32_t CFLR3;         /* Offset: 0x74  PWM Capture Falling Latch Register (Channel 3)                     */
    __IO uint32_t CAPENR;        /* Offset: 0x78  PWM Capture Input 0~3 Enable Register                              */
    __IO uint32_t POE;           /* Offset: 0x7C  PWM Output Enable for Channel 0~3                                  */
    __IO uint32_t TCON;          /* Offset: 0x80  PWM Trigger Control for Channel 0~3                                */
    __IO uint32_t TSTATUS;       /* Offset: 0x84  PWM Trigger Status Register                                        */
    __IO uint32_t SYNCBUSY0;     /* Offset: 0x88  PWM0 Synchronous Busy Status Register                              */
    __IO uint32_t SYNCBUSY1;     /* Offset: 0x8C  PWM1 Synchronous Busy Status Register                              */
    __IO uint32_t SYNCBUSY2;     /* Offset: 0x90  PWM2 Synchronous Busy Status Register                              */
    __IO uint32_t SYNCBUSY3;     /* Offset: 0x94  PWM3 Synchronous Busy Status Register                              */

} PWM_T;


/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */


/* PWM PPR Bit Field Definitions */
#define PWM_PPR_DZI23_Pos                       24                                  /*!< PWM_T::PPR: DZI23 Position */
#define PWM_PPR_DZI23_Msk                       (0xFFul << PWM_PPR_DZI23_Pos)       /*!< PWM_T::PPR: DZI23 Mask */

#define PWM_PPR_DZI01_Pos                       16                                  /*!< PWM_T::PPR: DZI01 Position */
#define PWM_PPR_DZI01_Msk                       (0xFFul << PWM_PPR_DZI01_Pos)       /*!< PWM_T::PPR: DZI01 Mask */

#define PWM_PPR_CP23_Pos                        8                                   /*!< PWM_T::PPR: CP23 Position */
#define PWM_PPR_CP23_Msk                        (0xFFul << PWM_PPR_CP23_Pos)        /*!< PWM_T::PPR: CP23 Mask */

#define PWM_PPR_CP01_Pos                        0                                   /*!< PWM_T::PPR: CP01 Position */
#define PWM_PPR_CP01_Msk                        (0xFFul << PWM_PPR_CP01_Pos)        /*!< PWM_T::PPR: CP01 Mask */

/* PWM CSR Bit Field Definitions */
#define PWM_CSR_CSR3_Pos                        12                                  /*!< PWM_T::CSR: CSR3 Position */
#define PWM_CSR_CSR3_Msk                        (7ul << PWM_CSR_CSR3_Pos)           /*!< PWM_T::CSR: CSR3 Mask */

#define PWM_CSR_CSR2_Pos                        8                                   /*!< PWM_T::CSR: CSR2 Position */
#define PWM_CSR_CSR2_Msk                        (7ul << PWM_CSR_CSR2_Pos)           /*!< PWM_T::CSR: CSR2 Mask */

#define PWM_CSR_CSR1_Pos                        4                                   /*!< PWM_T::CSR: CSR1 Position */
#define PWM_CSR_CSR1_Msk                        (7ul << PWM_CSR_CSR1_Pos)           /*!< PWM_T::CSR: CSR1 Mask */

#define PWM_CSR_CSR0_Pos                        0                                   /*!< PWM_T::CSR: CSR0 Position */
#define PWM_CSR_CSR0_Msk                        (7ul << PWM_CSR_CSR0_Pos)           /*!< PWM_T::CSR: CSR0 Mask */

/* PWM PCR Bit Field Definitions */
#define PWM_PCR_PWM23TYPE_Pos                   31                                  /*!< PWM_T::PCR: PWM23TYPE Position */
#define PWM_PCR_PWM23TYPE_Msk                   (1ul << PWM_PCR_PWM23TYPE_Pos)      /*!< PWM_T::PCR: PWM23TYPE Mask */

#define PWM_PCR_PWM01TYPE_Pos                   30                                  /*!< PWM_T::PCR: PWM01TYPE Position */
#define PWM_PCR_PWM01TYPE_Msk                   (1ul << PWM_PCR_PWM01TYPE_Pos)      /*!< PWM_T::PCR: PWM01TYPE Mask */

#define PWM_PCR_CH3MOD_Pos                      27                                  /*!< PWM_T::PCR: CH3MOD Position */
#define PWM_PCR_CH3MOD_Msk                      (1ul << PWM_PCR_CH3MOD_Pos)         /*!< PWM_T::PCR: CH3MOD Mask */

#define PWM_PCR_CH3INV_Pos                      26                                  /*!< PWM_T::PCR: CH3INV Position */
#define PWM_PCR_CH3INV_Msk                      (1ul << PWM_PCR_CH3INV_Pos)         /*!< PWM_T::PCR: CH3INV Mask */

#define PWM_PCR_CH3PINV_Pos                     25                                  /*!< PWM_T::PCR: CH3PINV Position */
#define PWM_PCR_CH3PINV_Msk                     (1ul << PWM_PCR_CH3PINV_Pos)        /*!< PWM_T::PCR: CH3PINV Mask */

#define PWM_PCR_CH3EN_Pos                       24                                  /*!< PWM_T::PCR: CH3EN Position */
#define PWM_PCR_CH3EN_Msk                       (1ul << PWM_PCR_CH3EN_Pos)          /*!< PWM_T::PCR: CH3EN Mask */

#define PWM_PCR_CH2MOD_Pos                      19                                  /*!< PWM_T::PCR: CH2MOD Position */
#define PWM_PCR_CH2MOD_Msk                      (1ul << PWM_PCR_CH2MOD_Pos)         /*!< PWM_T::PCR: CH2MOD Mask */

#define PWM_PCR_CH2INV_Pos                      18                                  /*!< PWM_T::PCR: CH2INV Position */
#define PWM_PCR_CH2INV_Msk                      (1ul << PWM_PCR_CH2INV_Pos)         /*!< PWM_T::PCR: CH2INV Mask */

#define PWM_PCR_CH2PINV_Pos                     17                                  /*!< PWM_T::PCR: CH2PINV Position */
#define PWM_PCR_CH2PINV_Msk                     (1ul << PWM_PCR_CH2PINV_Pos)        /*!< PWM_T::PCR: CH2PINV Mask */

#define PWM_PCR_CH2EN_Pos                       16                                  /*!< PWM_T::PCR: CH2EN Position */
#define PWM_PCR_CH2EN_Msk                       (1ul << PWM_PCR_CH2EN_Pos)          /*!< PWM_T::PCR: CH2EN Mask */

#define PWM_PCR_CH1MOD_Pos                      11                                  /*!< PWM_T::PCR: CH1MOD Position */
#define PWM_PCR_CH1MOD_Msk                      (1ul << PWM_PCR_CH1MOD_Pos)         /*!< PWM_T::PCR: CH1MOD Mask */

#define PWM_PCR_CH1INV_Pos                      10                                  /*!< PWM_T::PCR: CH1INV Position */
#define PWM_PCR_CH1INV_Msk                      (1ul << PWM_PCR_CH1INV_Pos)         /*!< PWM_T::PCR: CH1INV Mask */

#define PWM_PCR_CH1PINV_Pos                     9                                   /*!< PWM_T::PCR: CH1PINV Position */
#define PWM_PCR_CH1PINV_Msk                     (1ul << PWM_PCR_CH1PINV_Pos)        /*!< PWM_T::PCR: CH1PINV Mask */

#define PWM_PCR_CH1EN_Pos                       8                                   /*!< PWM_T::PCR: CH1EN Position */
#define PWM_PCR_CH1EN_Msk                       (1ul << PWM_PCR_CH1EN_Pos)          /*!< PWM_T::PCR: CH1EN Mask */

#define PWM_PCR_DZEN23_Pos                      5                                   /*!< PWM_T::PCR: DZEN23 Position */
#define PWM_PCR_DZEN23_Msk                      (1ul << PWM_PCR_DZEN23_Pos)         /*!< PWM_T::PCR: DZEN23 Mask */

#define PWM_PCR_DZEN01_Pos                      4                                   /*!< PWM_T::PCR: DZEN01 Position */
#define PWM_PCR_DZEN01_Msk                      (1ul << PWM_PCR_DZEN01_Pos)         /*!< PWM_T::PCR: DZEN01 Mask */

#define PWM_PCR_CH0MOD_Pos                      3                                   /*!< PWM_T::PCR: CH0MOD Position */
#define PWM_PCR_CH0MOD_Msk                      (1ul << PWM_PCR_CH0MOD_Pos)         /*!< PWM_T::PCR: CH0MOD Mask */

#define PWM_PCR_CH0INV_Pos                      2                                   /*!< PWM_T::PCR: CH0INV Position */
#define PWM_PCR_CH0INV_Msk                      (1ul << PWM_PCR_CH0INV_Pos)         /*!< PWM_T::PCR: CH0INV Mask */

#define PWM_PCR_CH0PINV_Pos                      1                                  /*!< PWM_T::PCR: CH0PINV Position */
#define PWM_PCR_CH0PINV_Msk                     (1ul << PWM_PCR_CH0PINV_Pos)        /*!< PWM_T::PCR: CH0PINV Mask */

#define PWM_PCR_CH0EN_Pos                       0                                   /*!< PWM_T::PCR: CH0EN Position */
#define PWM_PCR_CH0EN_Msk                       (1ul << PWM_PCR_CH0EN_Pos)          /*!< PWM_T::PCR: CH0EN Mask */

/* PWM CNR Bit Field Definitions */
#define PWM_CNR_CNR_Pos                         0                                   /*!< PWM_T::CNR0: CNR Position */
#define PWM_CNR_CNR_Msk                         (0xFFFFul << PWM_CNR_CNR_Pos)       /*!< PWM_T::CNR0: CNR Mask */

/* PWM CMR Bit Field Definitions */
#define PWM_CMR_CMR_Pos                         0                                   /*!< PWM_T::CMR0: CMR Position */
#define PWM_CMR_CMR_Msk                         (0xFFFFul << PWM_CMR_CMR_Pos)       /*!< PWM_T::CMR0: CMR Mask */

/* PWM PDR Bit Field Definitions */
#define PWM_PDR_PDR_Pos                         0                                   /*!< PWM_T::PDR0: PDR Position */
#define PWM_PDR_PDR_Msk                         (0xFFFFul << PWM_PDR_PDR_Pos)       /*!< PWM_T::PDR0: PDR Mask */

/* PWM PBCR Bit Field Definitions */
#define PWM_PBCR_BCn_Pos                         0                                  /*!< PWM_T::PBCR: BCn Position */
#define PWM_PBCR_BCn_Msk                         (1ul << PWM_PBCR_BCn_Pos)          /*!< PWM_T::PBCR: BCn Mask */

/* PWM PIER Bit Field Definitions */

#define PWM_PIER_INT23TYPE_Pos                  17                                  /*!< PWM_T::PIER: INT23TYPE Position */
#define PWM_PIER_INT23TYPE_Msk                     (1ul << PWM_PIER_INT23TYPE_Pos)  /*!< PWM_T::PIER: INT23TYPE Mask */

#define PWM_PIER_INT01TYPE_Pos                  16                                  /*!< PWM_T::PIER: INT01TYPE Position */
#define PWM_PIER_INT01TYPE_Msk                     (1ul << PWM_PIER_INT01TYPE_Pos)  /*!< PWM_T::PIER: INT01TYPE Mask */

#define PWM_PIER_PWMDIE3_Pos                    11                                  /*!< PWM_T::PIER: PWMDIE3 Position */
#define PWM_PIER_PWMDIE3_Msk                     (1ul << PWM_PIER_PWMDIE3_Pos)      /*!< PWM_T::PIER: PWMDIE3 Mask */

#define PWM_PIER_PWMDIE2_Pos                    10                                  /*!< PWM_T::PIER: PWMDIE2 Position */
#define PWM_PIER_PWMDIE2_Msk                     (1ul << PWM_PIER_PWMDIE2_Pos)      /*!< PWM_T::PIER: PWMDIE2 Mask */

#define PWM_PIER_PWMDIE1_Pos                    9                                   /*!< PWM_T::PIER: PWMDIE1 Position */
#define PWM_PIER_PWMDIE1_Msk                     (1ul << PWM_PIER_PWMDIE1_Pos)      /*!< PWM_T::PIER: PWMDIE1 Mask */

#define PWM_PIER_PWMDIE0_Pos                    8                                   /*!< PWM_T::PIER: PWMDIE0 Position */
#define PWM_PIER_PWMDIE0_Msk                     (1ul << PWM_PIER_PWMDIE0_Pos)      /*!< PWM_T::PIER: PWMDIE0 Mask */

#define PWM_PIER_PWMIE3_Pos                     3                                   /*!< PWM_T::PIER: PWMIE3 Position */
#define PWM_PIER_PWMIE3_Msk                     (1ul << PWM_PIER_PWMIE3_Pos)        /*!< PWM_T::PIER: PWMIE3 Mask */

#define PWM_PIER_PWMIE2_Pos                     2                                   /*!< PWM_T::PIER: PWMIE2 Position */
#define PWM_PIER_PWMIE2_Msk                     (1ul << PWM_PIER_PWMIE2_Pos)        /*!< PWM_T::PIER: PWMIE2 Mask */

#define PWM_PIER_PWMIE1_Pos                     1                                   /*!< PWM_T::PIER: PWMIE1 Position */
#define PWM_PIER_PWMIE1_Msk                     (1ul << PWM_PIER_PWMIE1_Pos)        /*!< PWM_T::PIER: PWMIE1 Mask */

#define PWM_PIER_PWMIE0_Pos                     0                                   /*!< PWM_T::PIER: PWMIE0 Position */
#define PWM_PIER_PWMIE0_Msk                     (1ul << PWM_PIER_PWMIE0_Pos)        /*!< PWM_T::PIER: PWMIE0 Mask */

/* PWM PIIR Bit Field Definitions */
#define PWM_PIIR_PWMDIF3_Pos                    11                                  /*!< PWM_T::PIIR: PWMDIF3 Position */
#define PWM_PIIR_PWMDIF3_Msk                    (1ul << PWM_PIIR_PWMDIF3_Pos)       /*!< PWM_T::PIIR: PWMDIF3 Mask */

#define PWM_PIIR_PWMDIF2_Pos                    10                                  /*!< PWM_T::PIIR: PWMDIF2 Position */
#define PWM_PIIR_PWMDIF2_Msk                    (1ul << PWM_PIIR_PWMDIF2_Pos)       /*!< PWM_T::PIIR: PWMDIF2 Mask */

#define PWM_PIIR_PWMDIF1_Pos                    9                                   /*!< PWM_T::PIIR: PWMDIF1 Position */
#define PWM_PIIR_PWMDIF1_Msk                    (1ul << PWM_PIIR_PWMDIF1_Pos)       /*!< PWM_T::PIIR: PWMDIF1 Mask */

#define PWM_PIIR_PWMDIF0_Pos                    8                                   /*!< PWM_T::PIIR: PWMDIF0 Position */
#define PWM_PIIR_PWMDIF0_Msk                    (1ul << PWM_PIIR_PWMDIF0_Pos)       /*!< PWM_T::PIIR: PWMDIF0 Mask */

#define PWM_PIIR_PWMIF3_Pos                     3                                   /*!< PWM_T::PIIR: PWMIF3 Position */
#define PWM_PIIR_PWMIF3_Msk                     (1ul << PWM_PIIR_PWMIF3_Pos)        /*!< PWM_T::PIIR: PWMIF3 Mask */

#define PWM_PIIR_PWMIF2_Pos                     2                                   /*!< PWM_T::PIIR: PWMIF2 Position */
#define PWM_PIIR_PWMIF2_Msk                     (1ul << PWM_PIIR_PWMIF2_Pos)        /*!< PWM_T::PIIR: PWMIF2 Mask */

#define PWM_PIIR_PWMIF1_Pos                     1                                   /*!< PWM_T::PIIR: PWMIF1 Position */
#define PWM_PIIR_PWMIF1_Msk                     (1ul << PWM_PIIR_PWMIF1_Pos)        /*!< PWM_T::PIIR: PWMIF1 Mask */

#define PWM_PIIR_PWMIF0_Pos                     0                                   /*!< PWM_T::PIIR: PWMIF0 Position */
#define PWM_PIIR_PWMIF0_Msk                     (1ul << PWM_PIIR_PWMIF0_Pos)        /*!< PWM_T::PIIR: PWMIF0 Mask */

/* PWM CCR0 Bit Field Definitions */
#define PWM_CCR0_CFLRI1_Pos                     23                                  /*!< PWM_T::CCR0: CFLRI1 Position */
#define PWM_CCR0_CFLRI1_Msk                     (1ul << PWM_CCR0_CFLRI1_Pos)        /*!< PWM_T::CCR0: CFLRI1 Mask */

#define PWM_CCR0_CRLRI1_Pos                     22                                  /*!< PWM_T::CCR0: CRLRI1 Position */
#define PWM_CCR0_CRLRI1_Msk                     (1ul << PWM_CCR0_CRLRI1_Pos)        /*!< PWM_T::CCR0: CRLRI1 Mask */

#define PWM_CCR0_CAPIF1_Pos                     20                                  /*!< PWM_T::CCR0: CAPIF1 Position */
#define PWM_CCR0_CAPIF1_Msk                     (1ul << PWM_CCR0_CAPIF1_Pos)        /*!< PWM_T::CCR0: CAPIF1 Mask */

#define PWM_CCR0_CAPCH1EN_Pos                   19                                  /*!< PWM_T::CCR0: CAPCH1EN Position */
#define PWM_CCR0_CAPCH1EN_Msk                   (1ul << PWM_CCR0_CAPCH1EN_Pos)      /*!< PWM_T::CCR0: CAPCH1EN Mask */

#define PWM_CCR0_CFL_IE1_Pos                    18                                  /*!< PWM_T::CCR0: CFL_IE1 Position */
#define PWM_CCR0_CFL_IE1_Msk                    (1ul << PWM_CCR0_CFL_IE1_Pos)       /*!< PWM_T::CCR0: CFL_IE1 Mask */

#define PWM_CCR0_CRL_IE1_Pos                    17                                  /*!< PWM_T::CCR0: CRL_IE1 Position */
#define PWM_CCR0_CRL_IE1_Msk                    (1ul << PWM_CCR0_CRL_IE1_Pos)       /*!< PWM_T::CCR0: CRL_IE1 Mask */

#define PWM_CCR0_INV1_Pos                       16                                  /*!< PWM_T::CCR0: INV1 Position */
#define PWM_CCR0_INV1_Msk                       (1ul << PWM_CCR0_INV1_Pos)          /*!< PWM_T::CCR0: INV1 Mask */

#define PWM_CCR0_CFLRI0_Pos                     7                                   /*!< PWM_T::CCR0: CFLRI0 Position */
#define PWM_CCR0_CFLRI0_Msk                     (1ul << PWM_CCR0_CFLRI0_Pos)        /*!< PWM_T::CCR0: CFLRI0 Mask */

#define PWM_CCR0_CRLRI0_Pos                     6                                   /*!< PWM_T::CCR0: CRLRI0 Position */
#define PWM_CCR0_CRLRI0_Msk                     (1ul << PWM_CCR0_CRLRI0_Pos)        /*!< PWM_T::CCR0: CRLRI0 Mask */

#define PWM_CCR0_CAPIF0_Pos                     4                                   /*!< PWM_T::CCR0: CAPIF0 Position */
#define PWM_CCR0_CAPIF0_Msk                     (1ul << PWM_CCR0_CAPIF0_Pos)        /*!< PWM_T::CCR0: CAPIF0 Mask */

#define PWM_CCR0_CAPCH0EN_Pos                   3                                   /*!< PWM_T::CCR0: CAPCH0EN Position */
#define PWM_CCR0_CAPCH0EN_Msk                   (1ul << PWM_CCR0_CAPCH0EN_Pos)      /*!< PWM_T::CCR0: CAPCH0EN Mask */

#define PWM_CCR0_CFL_IE0_Pos                    2                                   /*!< PWM_T::CCR0: CFL_IE0 Position */
#define PWM_CCR0_CFL_IE0_Msk                    (1ul << PWM_CCR0_CFL_IE0_Pos)       /*!< PWM_T::CCR0: CFL_IE0 Mask */

#define PWM_CCR0_CRL_IE0_Pos                    1                                   /*!< PWM_T::CCR0: CRL_IE0 Position */
#define PWM_CCR0_CRL_IE0_Msk                    (1ul << PWM_CCR0_CRL_IE0_Pos)       /*!< PWM_T::CCR0: CRL_IE0 Mask */

#define PWM_CCR0_INV0_Pos                       0                                   /*!< PWM_T::CCR0: INV0 Position */
#define PWM_CCR0_INV0_Msk                       (1ul << PWM_CCR0_INV0_Pos)          /*!< PWM_T::CCR0: INV0 Mask */

/* PWM CCR2 Bit Field Definitions */
#define PWM_CCR2_CFLRI3_Pos                     23                                  /*!< PWM_T::CCR2: CFLRI3 Position */
#define PWM_CCR2_CFLRI3_Msk                     (1ul << PWM_CCR2_CFLRI3_Pos)        /*!< PWM_T::CCR2: CFLRI3 Mask */

#define PWM_CCR2_CRLRI3_Pos                     22                                  /*!< PWM_T::CCR2: CRLRI3 Position */
#define PWM_CCR2_CRLRI3_Msk                     (1ul << PWM_CCR2_CRLRI3_Pos)        /*!< PWM_T::CCR2: CRLRI3 Mask */

#define PWM_CCR2_CAPIF3_Pos                     20                                  /*!< PWM_T::CCR2: CAPIF3 Position */
#define PWM_CCR2_CAPIF3_Msk                     (1ul << PWM_CCR2_CAPIF3_Pos)        /*!< PWM_T::CCR2: CAPIF3 Mask */

#define PWM_CCR2_CAPCH3EN_Pos                   19                                  /*!< PWM_T::CCR2: CAPCH3EN Position */
#define PWM_CCR2_CAPCH3EN_Msk                   (1ul << PWM_CCR2_CAPCH3EN_Pos)      /*!< PWM_T::CCR2: CAPCH3EN Mask */

#define PWM_CCR2_CFL_IE3_Pos                    18                                  /*!< PWM_T::CCR2: CFL_IE3 Position */
#define PWM_CCR2_CFL_IE3_Msk                    (1ul << PWM_CCR2_CFL_IE3_Pos)       /*!< PWM_T::CCR2: CFL_IE3 Mask */

#define PWM_CCR2_CRL_IE3_Pos                    17                                  /*!< PWM_T::CCR2: CRL_IE3 Position */
#define PWM_CCR2_CRL_IE3_Msk                    (1ul << PWM_CCR2_CRL_IE3_Pos)       /*!< PWM_T::CCR2: CRL_IE3 Mask */

#define PWM_CCR2_INV3_Pos                       16                                  /*!< PWM_T::CCR2: INV3 Position */
#define PWM_CCR2_INV3_Msk                       (1ul << PWM_CCR2_INV3_Pos)          /*!< PWM_T::CCR2: INV3 Mask */

#define PWM_CCR2_CFLRI2_Pos                     7                                   /*!< PWM_T::CCR2: CFLRI2 Position */
#define PWM_CCR2_CFLRI2_Msk                     (1ul << PWM_CCR2_CFLRI2_Pos)        /*!< PWM_T::CCR2: CFLRI2 Mask */

#define PWM_CCR2_CRLRI2_Pos                     6                                   /*!< PWM_T::CCR2: CRLRI2 Position */
#define PWM_CCR2_CRLRI2_Msk                     (1ul << PWM_CCR2_CRLRI2_Pos)        /*!< PWM_T::CCR2: CRLRI2 Mask */

#define PWM_CCR2_CAPIF2_Pos                     4                                   /*!< PWM_T::CCR2: CAPIF2 Position */
#define PWM_CCR2_CAPIF2_Msk                     (1ul << PWM_CCR2_CAPIF2_Pos)        /*!< PWM_T::CCR2: CAPIF2 Mask */

#define PWM_CCR2_CAPCH2EN_Pos                   3                                   /*!< PWM_T::CCR2: CAPCH2EN Position */
#define PWM_CCR2_CAPCH2EN_Msk                   (1ul << PWM_CCR2_CAPCH2EN_Pos)      /*!< PWM_T::CCR2: CAPCH2EN Mask */

#define PWM_CCR2_CFL_IE2_Pos                    2                                   /*!< PWM_T::CCR2: CFL_IE2 Position */
#define PWM_CCR2_CFL_IE2_Msk                    (1ul << PWM_CCR2_CFL_IE2_Pos)       /*!< PWM_T::CCR2: CFL_IE2 Mask */

#define PWM_CCR2_CRL_IE2_Pos                    1                                   /*!< PWM_T::CCR2: CRL_IE2 Position */
#define PWM_CCR2_CRL_IE2_Msk                    (1ul << PWM_CCR2_CRL_IE2_Pos)       /*!< PWM_T::CCR2: CRL_IE2 Mask */

#define PWM_CCR2_INV2_Pos                       0                                   /*!< PWM_T::CCR2: INV2 Position */
#define PWM_CCR2_INV2_Msk                       (1ul << PWM_CCR2_INV2_Pos)          /*!< PWM_T::CCR2: INV2 Mask */

/* PWM CRLR Bit Field Definitions */
#define PWM_CRLR_CRLR_Pos                       0                                   /*!< PWM_T::CRLR0: CRLR Position */
#define PWM_CRLR_CRLR_Msk                       (0xFFFFul << PWM_CRLR_CRLR_Pos)     /*!< PWM_T::CRLR0: CRLR Mask */

/* PWM CFLR Bit Field Definitions */
#define PWM_CFLR_CFLR_Pos                       0                                   /*!< PWM_T::CFLR0: CFLR Position */
#define PWM_CFLR_CFLR_Msk                       (0xFFFFul << PWM_CFLR_CFLR_Pos)     /*!< PWM_T::CFLR0: CFLR Mask */

/* PWM CAPENR Bit Field Definitions */
#define PWM_CAPENR_CINEN3_Pos                   3                                   /*!< PWM_T::CAPENR: CINEN3 Position */
#define PWM_CAPENR_CINEN3_Msk                   (1ul << PWM_CAPENR_CINEN3_Pos)      /*!< PWM_T::CAPENR: CINEN3 Mask */

#define PWM_CAPENR_CINEN2_Pos                   2                                   /*!< PWM_T::CAPENR: CINEN2 Position */
#define PWM_CAPENR_CINEN2_Msk                   (1ul << PWM_CAPENR_CINEN2_Pos)      /*!< PWM_T::CAPENR: CINEN2 Mask */

#define PWM_CAPENR_CINEN1_Pos                   1                                   /*!< PWM_T::CAPENR: CINEN1 Position */
#define PWM_CAPENR_CINEN1_Msk                   (1ul << PWM_CAPENR_CINEN1_Pos)      /*!< PWM_T::CAPENR: CINEN1 Mask */

#define PWM_CAPENR_CINEN0_Pos                   0                                   /*!< PWM_T::CAPENR: CINEN0 Position */
#define PWM_CAPENR_CINEN0_Msk                   (1ul << PWM_CAPENR_CINEN0_Pos)      /*!< PWM_T::CAPENR: CINEN0 Mask */

/* PWM POE Bit Field Definitions */
#define PWM_POE_POE3_Pos                        3                                   /*!< PWM_T::POE: POE3 Position */
#define PWM_POE_POE3_Msk                        (1ul << PWM_POE_POE3_Pos)           /*!< PWM_T::POE: POE3 Mask */

#define PWM_POE_POE2_Pos                        2                                   /*!< PWM_T::POE: POE2 Position */
#define PWM_POE_POE2_Msk                        (1ul << PWM_POE_POE2_Pos)           /*!< PWM_T::POE: POE2 Mask */

#define PWM_POE_POE1_Pos                        1                                   /*!< PWM_T::POE: POE1 Position */
#define PWM_POE_POE1_Msk                        (1ul << PWM_POE_POE1_Pos)           /*!< PWM_T::POE: POE1 Mask */

#define PWM_POE_POE0_Pos                        0                                   /*!< PWM_T::POE: POE0 Position */
#define PWM_POE_POE0_Msk                        (1ul << PWM_POE_POE0_Pos)           /*!< PWM_T::POE: POE0 Mask */

/* PWM TCON Bit Field Definitions */

#define PWM_TCON_PWM3TEN_Pos                    3                                   /*!< PWM_T::TCON: PWM3TEN Position */
#define PWM_TCON_PWM3TEN_Msk                    (1ul << PWM_TCON_PWM3TEN_Pos)       /*!< PWM_T::TCON: PWM3TEN Mask */

#define PWM_TCON_PWM2TEN_Pos                    2                                   /*!< PWM_T::TCON: PWM2TEN Position */
#define PWM_TCON_PWM2TEN_Msk                    (1ul << PWM_TCON_PWM2TEN_Pos)       /*!< PWM_T::TCON: PWM2TEN Mask */

#define PWM_TCON_PWM1TEN_Pos                    1                                   /*!< PWM_T::TCON: PWM1TEN Position */
#define PWM_TCON_PWM1TEN_Msk                    (1ul << PWM_TCON_PWM1TEN_Pos)       /*!< PWM_T::TCON: PWM1TEN Mask */

#define PWM_TCON_PWM0TEN_Pos                    0                                   /*!< PWM_T::TCON: PWM0TEN Position */
#define PWM_TCON_PWM0TEN_Msk                    (1ul << PWM_TCON_PWM0TEN_Pos)       /*!< PWM_T::TCON: PWM0TEN Mask */

/* PWM TSTATUS Bit Field Definitions */

#define PWM_TSTATUS_PWM3TF_Pos                  3                                   /*!< PWM_T::TSTATUS: PWM3TF Position */
#define PWM_TSTATUS_PWM3TF_Msk                  (1ul << PWM_TSTATUS_PWM3TF_Pos)     /*!< PWM_T::TSTATUS: PWM3TF Mask */

#define PWM_TSTATUS_PWM2TF_Pos                  2                                   /*!< PWM_T::TSTATUS: PWM2TF Position */
#define PWM_TSTATUS_PWM2TF_Msk                  (1ul << PWM_TSTATUS_PWM2TF_Pos)     /*!< PWM_T::TSTATUS: PWM2TF Mask */

#define PWM_TSTATUS_PWM1TF_Pos                  1                                   /*!< PWM_T::TSTATUS: PWM1TF Position */
#define PWM_TSTATUS_PWM1TF_Msk                  (1ul << PWM_TSTATUS_PWM1TF_Pos)     /*!< PWM_T::TSTATUS: PWM1TF Mask */

#define PWM_TSTATUS_PWM0TF_Pos                  0                                   /*!< PWM_T::TSTATUS: PWM0TF Position */
#define PWM_TSTATUS_PWM0TF_Msk                  (1ul << PWM_TSTATUS_PWM0TF_Pos)     /*!< PWM_T::TSTATUS: PWM0TF Mask */

/* PWM SYNCBUSY0 Bit Field Definitions */
#define PWM_SYNCBUSY0_S_BUSY_Pos                0                                   /*!< PWM_T::SYNCBUSY0: S_BUSY Position */
#define PWM_SYNCBUSY0_S_BUSY_Msk                (1ul << PWM_SYNCBUSY0_S_BUSY_Pos)   /*!< PWM_T::SYNCBUSY0: S_BUSY Mask */

/* PWM SYNCBUSY1 Bit Field Definitions */
#define PWM_SYNCBUSY1_S_BUSY_Pos                0                                   /*!< PWM_T::SYNCBUSY1: S_BUSY Position */
#define PWM_SYNCBUSY1_S_BUSY_Msk                (1ul << PWM_SYNCBUSY1_S_BUSY_Pos)   /*!< PWM_T::SYNCBUSY1: S_BUSY Mask */

/* PWM SYNCBUSY2 Bit Field Definitions */
#define PWM_SYNCBUSY2_S_BUSY_Pos                0                                   /*!< PWM_T::SYNCBUSY2: S_BUSY Position */
#define PWM_SYNCBUSY2_S_BUSY_Msk                (1ul << PWM_SYNCBUSY2_S_BUSY_Pos)   /*!< PWM_T::SYNCBUSY2: S_BUSY Mask */

/* PWM SYNCBUSY3 Bit Field Definitions */
#define PWM_SYNCBUSY3_S_BUSY_Pos                0                                   /*!< PWM_T::SYNCBUSY3: S_BUSY Position */
#define PWM_SYNCBUSY3_S_BUSY_Msk                (1ul << PWM_SYNCBUSY3_S_BUSY_Pos)   /*!< PWM_T::SYNCBUSY3: S_BUSY Mask */
/*@}*/ /* end of group PWM_CONST */
/*@}*/ /* end of group PWM */
/**@}*/ /* end of REGISTER group */


#endif /* __PWM_REG_H__ */
