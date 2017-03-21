
#if defined(TARGET_NANO100A) || defined(TARGET_NANO100B) || defined(TARGET_NUC472_442)
#define DetectPin   				PB15

#elif defined(TARGET_NUC121)
#define DetectPin   				PB0

#elif defined(TARGET_NUC122) || defined(TARGET_NUC230_240)
#define DetectPin   				PA10

#elif defined(TARGET_NUC123)
#define DetectPin   				PA13

#elif defined(TARGET_NUC126)
#define DetectPin   				PD0

#elif defined(TARGET_M451) || defined(TARGET_M480)
#define DetectPin   				PB6

#endif

#ifndef DetectPin
#error "DetectPin is not defined."
#endif
