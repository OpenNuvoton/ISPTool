;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

    
    IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000400
    ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

    IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000000
    ENDIF

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
                                                  ; maximum of 32 External Interrupts are possible
                DCD     BOD_IRQHandler  
                DCD     WDT_IRQHandler  
                DCD     EINT0_IRQHandler
                DCD     EINT1_IRQHandler
                DCD     GPG0_IRQHandler 
                DCD     GPG1_IRQHandler
                DCD     BPWM0_IRQHandler 
                DCD     EADC0_IRQHandler 
                DCD     TMR0_IRQHandler 
                DCD     TMR1_IRQHandler 
                DCD     TMR2_IRQHandler 
                DCD     TMR3_IRQHandler 
                DCD     UART0_IRQHandler
                DCD     UART1_IRQHandler
                DCD     SPI0_IRQHandler 
                DCD     SPI1_IRQHandler 
                DCD     SPI2_IRQHandler 
                DCD     MDU_IRQHandler 
                DCD     I2C0_IRQHandler 
                DCD     CKD_IRQHandler 
                DCD     Default_IRQHandler 
                DCD     EPWM0_IRQHandler
                DCD     EPWM1_IRQHandler 
                DCD     CAP0_IRQHandler  
                DCD     CAP1_IRQHandler  
                DCD     ACMP_IRQHandler 
                DCD     QEI0_IRQHandler
                DCD     QEI1_IRQHandler 
                DCD     PWRWU_IRQHandler
                DCD     EADC1_IRQHandler
                DCD     EADC2_IRQHandler  
                DCD     EADC3_IRQHandler  
                
                       
                
                AREA    |.text|, CODE, READONLY
                
                
                
; Reset Handler 
                
                ENTRY
                
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                LDR     R0, =0x50000100
                ; Unlock Register                
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
                EXPORT  GPG0_IRQHandler           [WEAK]
                EXPORT  GPG1_IRQHandler           [WEAK]
                EXPORT  BPWM0_IRQHandler          [WEAK]
                EXPORT  EADC0_IRQHandler          [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  SPI2_IRQHandler           [WEAK]
                EXPORT  MDU_IRQHandler            [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  CKD_IRQHandler            [WEAK]
                EXPORT  EPWM0_IRQHandler          [WEAK] 
                EXPORT  EPWM1_IRQHandler          [WEAK]
                EXPORT  CAP0_IRQHandler           [WEAK]
                EXPORT  CAP1_IRQHandler           [WEAK]                
                EXPORT  ACMP_IRQHandler           [WEAK]
                EXPORT  QEI0_IRQHandler           [WEAK]
                EXPORT  QEI1_IRQHandler           [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  EADC1_IRQHandler          [WEAK]
                EXPORT  EADC2_IRQHandler          [WEAK]
                EXPORT  EADC3_IRQHandler          [WEAK]                
 
Default_IRQHandler                
BOD_IRQHandler  
WDT_IRQHandler  
EINT0_IRQHandler
EINT1_IRQHandler
GPG0_IRQHandler 
GPG1_IRQHandler
BPWM0_IRQHandler 
EADC0_IRQHandler 
TMR0_IRQHandler 
TMR1_IRQHandler 
TMR2_IRQHandler 
TMR3_IRQHandler 
UART0_IRQHandler
UART1_IRQHandler
SPI0_IRQHandler 
SPI1_IRQHandler 
SPI2_IRQHandler 
MDU_IRQHandler 
I2C0_IRQHandler 
CKD_IRQHandler 
EPWM0_IRQHandler
EPWM1_IRQHandler 
CAP0_IRQHandler  
CAP1_IRQHandler  
ACMP_IRQHandler 
QEI0_IRQHandler
QEI1_IRQHandler 
PWRWU_IRQHandler
EADC1_IRQHandler
EADC2_IRQHandler  
EADC3_IRQHandler  

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
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

                END
