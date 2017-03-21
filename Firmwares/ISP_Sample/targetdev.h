#ifndef __TARGET_H__
#define __TARGET_H__

#ifdef __cplusplus
extern "C"
{
#endif

// Nuvoton MCU Peripheral Access Layer Header File
#if defined(TARGET_M051)
#include "M051Series.h"

#elif defined(TARGET_M058)
#include "M058S.h"

#elif defined(TARGET_M451)
#include "M451Series.h"

#elif defined(TARGET_M0518)
#include "M0518.h"

#elif defined(TARGET_M0519)
#include "M0519.h"

#elif defined(TARGET_M0564)
#include "M0564.h"
	
#elif defined(TARGET_MINI51)
#include "Mini51Series.h"

#elif defined(TARGET_MINI58)
#include "Mini58Series.h"

#elif defined(TARGET_NANO1X2)
#include "Nano1X2Series.h"

#elif defined(TARGET_NANO100A) || defined(TARGET_NANO100B)
// There are two different header files with the same name "Nano100Series.h".
// Library\Device\Nuvoton\Nano100ASeries\Include\Nano100Series.h
// Library\Device\Nuvoton\Nano100BSeries\Include\Nano100Series.h
#include "Nano100Series.h"

#elif defined(TARGET_NANO103)
#include "Nano103.h"

#elif defined(TARGET_NM1530)
#include "NM1530.h"

#elif defined(TARGET_NUC029FAE)
#include "NUC029FAE.h"

#elif defined(TARGET_NUC029XAN)
#include "NUC029xAN.h"

#elif defined(TARGET_NUC100)
#include "NUC100Series.h"

#elif defined(TARGET_NUC121)
#include "NUC121.h"

#elif defined(TARGET_NUC122)
#include "NUC122.h"

#elif defined(TARGET_NUC123)
#include "NUC123.h"

#elif defined(TARGET_NUC126)
#include "NUC126.h"

#elif defined(TARGET_NUC131)
#include "NUC131.h"

#elif defined(TARGET_NUC200)
#include "NUC200Series.h"

#elif defined(TARGET_NUC230_240)
#include "NUC230_240.h"

#elif defined(TARGET_NUC472_442)
#include "NUC472_442.h"

#elif defined(TARGET_M480)
#include "M480.h"

#else
#error "Nuvoton MCU Peripheral Access Layer Header File is not included."
#endif


#include "bsp_name.h"
#include "ISP_USER.h"

#ifdef __cplusplus
}
#endif

#endif //__TARGET_H__
