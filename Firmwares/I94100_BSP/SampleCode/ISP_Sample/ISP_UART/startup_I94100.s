;/******************************************************************************
; * @file     startup_I94100.s
; * @version  V0.10
; * $Revision: 1 $
; * $Date: 16/06/14 10:24a $ 
; * @brief   CMSIS ARM Cortex-M4 Core Device Startup File
; *
; * @note
; * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
Stack_Size      EQU     0x000001000
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     BOD_IRQHandler            ; 0: Brown Out detection
                DCD     IRC_IRQHandler            ; 1: Internal RC
                DCD     PWRWU_IRQHandler          ; 2: Power down wake up 
                DCD     SRAMF_IRQHandler          ; 3: SRAM parity check error 
                DCD     CLKF_IRQHandler           ; 4: Clock fail detected 
                DCD     Default_Handler           ; 5: Reserved
                DCD     RTC_IRQHandler            ; 6: Real Time Clock 
                DCD     Default_Handler           ; 7: Reserved
                DCD     WDT_IRQHandler            ; 8: Watchdog timer
                DCD     WWDT_IRQHandler           ; 9: Window watchdog timer
                DCD     EINT0_IRQHandler          ; 10: External Input 0
                DCD     EINT1_IRQHandler          ; 11: External Input 1
                DCD     EINT2_IRQHandler          ; 12: External Input 2
                DCD     EINT3_IRQHandler          ; 13: External Input 3
                DCD     EINT4_IRQHandler          ; 14: External Input 4
                DCD     EINT5_IRQHandler          ; 15: External Input 5
                DCD     GPA_IRQHandler            ; 16: GPIO Port A
                DCD     GPB_IRQHandler            ; 17: GPIO Port B
                DCD     GPC_IRQHandler            ; 18: GPIO Port C
                DCD     GPD_IRQHandler            ; 19: GPIO Port D
                DCD     Default_Handler           ; 20: Reserved
                DCD     Default_Handler           ; 21: Reserved
                DCD     SPI0_IRQHandler           ; 22: SPI0
                DCD     SPI1_IRQHandler           ; 23: SPI1
                DCD     Default_Handler           ; 24: Reserved
                DCD     PWM0P0_IRQHandler         ; 25: PWM0 pair 0 interrupt
                DCD     PWM0P1_IRQHandler         ; 26: PWM0 pair 1 interrupt
                DCD     PWM0P2_IRQHandler         ; 27: PWM0 pair 2 interrupt
                DCD     Default_Handler        	  ; 28: Reserved
                DCD     Default_Handler           ; 29: Reserved
                DCD     Default_Handler           ; 30: Reserved
                DCD     Default_Handler           ; 31: Reserved
                DCD     TMR0_IRQHandler           ; 32: Timer 0
                DCD     TMR1_IRQHandler           ; 33: Timer 1
                DCD     TMR2_IRQHandler           ; 34: Timer 2
                DCD     TMR3_IRQHandler           ; 35: Timer 3
                DCD     UART0_IRQHandler          ; 36: UART0
                DCD     Default_Handler           ; 37: Reserved
                DCD     I2C0_IRQHandler           ; 38: I2C0
                DCD     I2C1_IRQHandler           ; 39: I2C1
                DCD     PDMA_IRQHandler           ; 40: Peripheral DMA
                DCD     Default_Handler           ; 41: Reserved
                DCD     EADC0_IRQHandler          ; 42: EADC source 0
                DCD     EADC1_IRQHandler          ; 43: EADC source 1
                DCD     Default_Handler           ; 44: Reserved
                DCD     Default_Handler           ; 45: Reserved
                DCD     EADC2_IRQHandler          ; 46: EADC source 2
                DCD     EADC3_IRQHandler          ; 47: EADC source 3
                DCD     Default_Handler           ; 48: Reserved
                DCD     Default_Handler           ; 49: Reserved
                DCD     Default_Handler           ; 50: Reserved
                DCD     SPI2_IRQHandler           ; 51: SPI2
				DCD     DMIC_IRQHandler           ; 52: DMIC
                DCD     USBD_IRQHandler           ; 53:	USBD
                DCD     Default_Handler           ; 54: 
                DCD     Default_Handler           ; 55: 
                DCD     VAD_IRQHandler			  ; 56:	VAD
                DCD     Default_Handler           ; 57: 
                DCD     Default_Handler           ; 58: 
                DCD     Default_Handler           ; 59: 
                DCD     Default_Handler           ; 60:
                DCD     Default_Handler           ; 61:
                DCD     DPWM_IRQHandler           ; 62: DPWM
                DCD     Default_Handler           ; 63:
                DCD     Default_Handler           ; 64: 
                DCD     Default_Handler           ; 65: 
                DCD     Default_Handler           ; 66: 
                DCD     Default_Handler           ; 67:
                DCD     I2S0_IRQHandler           ; 68: I2S0

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

				LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC 
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  IRC_IRQHandler            [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  SRAMF_IRQHandler          [WEAK]
                EXPORT  CLKF_IRQHandler		      [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  WWDT_IRQHandler           [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  EINT2_IRQHandler          [WEAK]
                EXPORT  EINT3_IRQHandler          [WEAK]
                EXPORT  EINT4_IRQHandler          [WEAK]
                EXPORT  EINT5_IRQHandler          [WEAK]
                EXPORT  GPA_IRQHandler            [WEAK]
                EXPORT  GPB_IRQHandler            [WEAK]
                EXPORT  GPC_IRQHandler            [WEAK]
                EXPORT  GPD_IRQHandler            [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  PWM0P0_IRQHandler         [WEAK]
                EXPORT  PWM0P1_IRQHandler         [WEAK]
                EXPORT  PWM0P2_IRQHandler         [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
				EXPORT	EADC0_IRQHandler		  [WEAK]
				EXPORT	EADC1_IRQHandler		  [WEAK]
				EXPORT	EADC2_IRQHandler		  [WEAK]
				EXPORT	EADC3_IRQHandler		  [WEAK]
                EXPORT  SPI2_IRQHandler           [WEAK]
				EXPORT	DMIC_IRQHandler			  [WEAK]
				EXPORT	USBD_IRQHandler			  [WEAK]
				EXPORT	VAD_IRQHandler			  [WEAK]
				EXPORT	DPWM_IRQHandler			  [WEAK]
                EXPORT  I2S0_IRQHandler           [WEAK]
				

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
SRAMF_IRQHandler
CLKF_IRQHandler
RTC_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT4_IRQHandler
EINT5_IRQHandler
GPA_IRQHandler
GPB_IRQHandler
GPC_IRQHandler
GPD_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
PWM0P0_IRQHandler
PWM0P1_IRQHandler
PWM0P2_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler 
I2C0_IRQHandler 
I2C1_IRQHandler
PDMA_IRQHandler
EADC0_IRQHandler
EADC1_IRQHandler
EADC2_IRQHandler
EADC3_IRQHandler
SPI2_IRQHandler
DMIC_IRQHandler
USBD_IRQHandler
VAD_IRQHandler
DPWM_IRQHandler
I2S0_IRQHandler
                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
;/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
