;/******************************************************************************
; * @file     startup_I91200.s
; * @version  V1.00
; * $Revision: 1 $
; * $Date: 16/12/05 11:28a $ 
; * @brief    CMSIS ARM Cortex-M0 Core Device Startup File
; *
; * @note
; * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/  

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000280

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
                DCD BOD_IRQHandler
                DCD WDT_IRQHandler
                DCD EINT0_IRQHandler
                DCD EINT1_IRQHandler
                DCD GPAB_IRQHandler
                DCD ALC_IRQHandler
                DCD PWM0_IRQHandler
                DCD Default_Handler
                DCD TMR0_IRQHandler
                DCD TMR1_IRQHandler
                DCD Default_Handler
                DCD UART1_IRQHandler
                DCD UART0_IRQHandler
				DCD SPI1_IRQHandler
                DCD SPI0_IRQHandler
                DCD DPWM_IRQHandler
                DCD Default_Handler
				DCD Default_Handler
                DCD I2C0_IRQHandler
                DCD Default_Handler
                DCD Default_Handler
                DCD CMP_IRQHandler
                DCD MAC_IRQHandler
                DCD Default_Handler
                DCD Default_Handler
                DCD SARADC_IRQHandler
                DCD PDMA_IRQHandler
                DCD I2S0_IRQHandler
                DCD CAPS_IRQHandler
                DCD SDADC_IRQHandler
                DCD Default_Handler
                DCD RTC_IRQHandler

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

                ; Disable POR to prevent repeated reset if the main voltage ramps from 0V.
				; The ramp voltage is unreliable!
                LDR     R2, =0x50000024
                LDR     R1, =0x00005AA5
                STR     R1, [R2]

                ; Lock register
                MOVS    R1, #0
                STR     R1, [R0]
				
				; Set event and execute WFE to ensure WIC is initialized.
				; Because WIC state doesn't synchronize with NVIC when M0 is in standby domain 
				SEV                  
                WFE                  

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

                EXPORT BOD_IRQHandler           [WEAK]
                EXPORT WDT_IRQHandler           [WEAK]
                EXPORT EINT0_IRQHandler         [WEAK]
                EXPORT EINT1_IRQHandler         [WEAK]
                EXPORT GPAB_IRQHandler          [WEAK]
                EXPORT ALC_IRQHandler           [WEAK]
                EXPORT PWM0_IRQHandler          [WEAK]
                EXPORT TMR0_IRQHandler          [WEAK]
                EXPORT TMR1_IRQHandler          [WEAK]
                EXPORT UART0_IRQHandler         [WEAK]
                EXPORT UART1_IRQHandler         [WEAK]
                EXPORT SPI1_IRQHandler          [WEAK]
                EXPORT SPI0_IRQHandler          [WEAK]
                EXPORT DPWM_IRQHandler          [WEAK]
                EXPORT I2C0_IRQHandler          [WEAK]
                EXPORT CMP_IRQHandler    	    [WEAK]
                EXPORT MAC_IRQHandler           [WEAK]
				EXPORT SARADC_IRQHandler		[WEAK]
                EXPORT PDMA_IRQHandler          [WEAK]
                EXPORT I2S0_IRQHandler          [WEAK]
                EXPORT CAPS_IRQHandler          [WEAK]
                EXPORT SDADC_IRQHandler         [WEAK]
                EXPORT RTC_IRQHandler           [WEAK]
	
                
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
UART1_IRQHandler
SPI1_IRQHandler
SPI0_IRQHandler  
DPWM_IRQHandler  
I2C0_IRQHandler  
CMP_IRQHandler
MAC_IRQHandler   
SARADC_IRQHandler
PDMA_IRQHandler  
I2S0_IRQHandler   
CAPS_IRQHandler  
SDADC_IRQHandler     
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

;/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/