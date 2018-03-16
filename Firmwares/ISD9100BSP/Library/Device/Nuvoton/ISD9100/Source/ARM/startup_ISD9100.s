;/******************************************************************************
; * @file     startup_ISD9100Series.s
; * @version  V1.00
; * $Revision: 2 $
; * $Date: 14/07/17 11:28a $ 
; * @brief    CMSIS ARM Cortex-M0 Core Device Startup File
; *
; * @note
; * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/  

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler
; External Interrupts
                DCD     BOD_IRQHandler            ; Brownout low voltage detected interrupt  
                DCD     WDT_IRQHandler            ; Watch Dog Timer interrupt  
                DCD     EINT0_IRQHandler          ; External signal interrupt from PB.14 pin
                DCD     EINT1_IRQHandler          ; External signal interrupt from PB.15 pin
                DCD     GPAB_IRQHandler           ; External interrupt from PA[15:0]/PB[15:0]
                DCD     ALC_IRQHandler            ; Automatic Level Control Interrupt
                DCD     PWM0_IRQHandler           ; PWM0 interrupt 
                DCD     Default_Handler           ; Reserved
                DCD     TMR0_IRQHandler           ; Timer 0 interrupt
                DCD     TMR1_IRQHandler           ; Timer 1 interrupt  
                DCD     Default_Handler           ; Reserved
                DCD     Default_Handler           ; Reserved
                DCD     UART0_IRQHandler          ; UART0 interrupt
                DCD     Default_Handler           ; Reserved
                DCD     SPI0_IRQHandler           ; SPI0 interrupt 
                DCD     Default_Handler           ; Reserved
                DCD     Default_Handler           ; Reserved 
                DCD     Default_Handler           ; Reserved 
                DCD     I2C0_IRQHandler           ; I2C0 interrupt 
                DCD     Default_Handler           ; Reserved
                DCD     Default_Handler           ; Reserved
                DCD     TALARM_IRQHandler         ; Temperature Alarm Interrupt
                DCD     Default_Handler           ; Reserved
				DCD     Default_Handler           ; Reserved
				DCD     Default_Handler           ; Reserved
                DCD     ACMP_IRQHandler           ; ACMP interrupt 
                DCD     PDMA_IRQHandler           ; PDMA interrupt
                DCD     I2S_IRQHandler            ; I2S interrupt
				DCD     CAPS_IRQHandler           ; Capacitive Touch Sensing Relaxation Oscillator Interrupt
                DCD     ADC_IRQHandler            ; Audio ADC interrupt
                DCD     Default_Handler           ; Reserved
                DCD     RTC_IRQHandler            ; Real time clock interrupt
                AREA    |.text|, CODE, READONLY



; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                ; Unlock Register
                LDR     R0, =0x50000100
                LDR     R1, =0x59
                STR     R1, [R0]
                LDR     R1, =0x16
                STR     R1, [R0]
                LDR     R1, =0x88
                STR     R1, [R0]

                ; Init POR
                LDR     R2, =0x50000024
                LDR     R1, =0x00005AA5
                STR     R1, [R2]

                ; Lock register
                MOVS    R1, #0
                STR     R1, [R0]

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
    IF :DEF:DEBUG_ENABLE_SEMIHOST
                MOV     R0, LR
                LSLS    R0, #29               ; Check bit 2
                BMI     SP_is_PSP             ; previous stack is PSP
                MRS     R0, MSP               ; previous stack is MSP, read MSP
                B       SP_Read_Ready
SP_is_PSP
                MRS     R0, PSP               ; Read PSP
SP_Read_Ready
                LDR     R1, [R0, #24]         ; Get previous PC
                LDRH    R3, [R1]              ; Get instruction
                LDR     R2, =0xBEAB           ; The sepcial BKPT instruction
                CMP     R3, R2                ; Test if the instruction at previous PC is BKPT
                BNE     HardFault_Handler_Ret ; Not BKPT
        
                ADDS    R1, #4                ; Skip BKPT and next line
                STR     R1, [R0, #24]         ; Save previous PC
        
                BX      LR
HardFault_Handler_Ret
    ENDIF

                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  GPAB_IRQHandler           [WEAK]
                EXPORT  ALC_IRQHandler            [WEAK]
                EXPORT  PWM0_IRQHandler           [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  TALARM_IRQHandler         [WEAK]
                EXPORT  ACMP_IRQHandler           [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
                EXPORT  I2S_IRQHandler            [WEAK]
                EXPORT  CAPS_IRQHandler           [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                
BOD_IRQHandler
WDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
GPAB_IRQHandler
ALC_IRQHandler
PWM0_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
UART0_IRQHandler
SPI0_IRQHandler
I2C0_IRQHandler
TALARM_IRQHandler
ACMP_IRQHandler
PDMA_IRQHandler
I2S_IRQHandler
CAPS_IRQHandler
ADC_IRQHandler
RTC_IRQHandler

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
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF
				
                END

;/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/