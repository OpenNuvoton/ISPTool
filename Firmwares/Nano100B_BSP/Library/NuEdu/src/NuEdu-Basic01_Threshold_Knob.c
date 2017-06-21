#include <stdio.h>
#include "NUC200Series.h"
#include "NuEdu-Basic01_Threshold_Knob.h"

void Open_Threshold_Knob(void)
{
    //Initial ACMP0 Function Pin
    SYS->GPC_MFP = (SYS->GPC_MFP & ~SYS_GPC_MFP_PC6_Msk) | SYS_GPC_MFP_PC6_CPP0;
    SYS->ALT_MFP1 &= ~SYS_ALT_MFP1_PC6_Msk;
    PC->OFFD |= GPIO_OFFD_ENABLE(6);

    SYS->GPC_MFP = (SYS->GPC_MFP & ~SYS_GPC_MFP_PC7_Msk) | SYS_GPC_MFP_PC7_CPN0;
    SYS->ALT_MFP1 &= ~SYS_ALT_MFP1_PC7_Msk;
    PC->OFFD |= GPIO_OFFD_ENABLE(7);

    /*  SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB2_Msk) | SYS_GPB_MFP_PB2_CPO0;
        SYS->ALT_MFP &= ~SYS_ALT_MFP_PB2_Msk;
    //  SYS->ALT_MFP2 &= ~SYS_ALT_MFP2_PB2_Msk;
    //  PB->OFFD |= GPIO_OFFD_ENABLE(2);
    */

    //Initial ACMP Clock Source
    SYS_UnlockReg();
//  SYS->IPRSTC2 |= SYS_IPRSTC2_ACMP_RST_Msk;
//  SYS->IPRSTC2 &= ~SYS_IPRSTC2_ACMP_RST_Msk;
    SYSCLK->APBCLK |= SYSCLK_APBCLK_ACMP_EN_Msk;
    SYS_LockReg();

    //Initial ACMP0 Peripheral
    ACMP->CMPSR = ACMP_CMPSR_CMPF0_Msk;         //Clear ACMP0 Flags
    ACMP->CMPCR[0] = ACMP_CMPCR_CMP_HYSEN_Msk | ACMP_CMPCR_CMPEN_Msk;
}

void Close_Threshold_Knob(void)
{
    //Close ACMP Function Pin
    SYS->GPC_MFP &= ~SYS_GPC_MFP_PC6_Msk;
    SYS->ALT_MFP1 &= ~SYS_ALT_MFP1_PC6_Msk;
    PC->OFFD &= ~GPIO_OFFD_ENABLE(6);

    SYS->GPC_MFP &= ~SYS_GPC_MFP_PC7_Msk;
    SYS->ALT_MFP1 &= ~SYS_ALT_MFP1_PC7_Msk;
    PC->OFFD &= ~GPIO_OFFD_ENABLE(7);

    /*  SYS->GPB_MFP &= ~SYS_GPB_MFP_PB2_Msk;
        SYS->ALT_MFP &= ~SYS_ALT_MFP_PB2_Msk;
    //  SYS->ALT_MFP2 &= ~SYS_ALT_MFP2_PB2_Msk;
    //  PB->OFFD &= ~GPIO_OFFD_ENABLE(2);
    */
    //Close ACMP Clock Source
    ACMP->CMPCR[0] &= ~ACMP_CMPCR_CMPEN_Msk;
    if(!(ACMP->CMPCR[0]&ACMP_CMPCR_CMPEN_Msk) || !(ACMP->CMPCR[1]&ACMP_CMPCR_CMPEN_Msk)) {  //Donot Cloce ACMP, Safe for other unknown ACMP device
        SYS_UnlockReg();
        SYS->IPRSTC2 |= SYS_IPRSTC2_ACMP_RST_Msk;
        SYS->IPRSTC2 &= ~SYS_IPRSTC2_ACMP_RST_Msk;
        SYSCLK->APBCLK &= ~SYSCLK_APBCLK_ACMP_EN_Msk;
        SYS_LockReg();
    }
}

uint32_t Get_Threshold_Knob(void)
{
    uint32_t ACMP0_Output_Level;

    if(ACMP->CMPSR&ACMP_CMPSR_CO0_Msk)
        ACMP0_Output_Level = 1;
    else
        ACMP0_Output_Level = 0;

    ACMP->CMPSR |= ACMP_CMPSR_CMPF0_Msk;        //Clear ACMP0 Flags

    return ACMP0_Output_Level;
}
