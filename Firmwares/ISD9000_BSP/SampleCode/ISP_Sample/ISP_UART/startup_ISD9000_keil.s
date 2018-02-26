;/******************************************************************************
; * @file     startup_ISD9000_keil.s
; * @version  V1.00
; * $Revision: 2 $
; * $Date: 16/05/25 18:28p $ 
; * @brief    CMSIS ARM Cortex-M0 Core Device Startup File
; *
; * @note
; * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/  

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size		EQU		0x00000400

				AREA	STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem		SPACE	Stack_Size
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

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; 1.Reset Handler
                DCD     NMI_Handler               ; 2.NMI Handler
                DCD     HardFault_Handler         ; 3.Hard Fault Handler
                DCD     0                         ; 4.Reserved
                DCD     0                         ; 5.Reserved
                DCD     0                         ; 6.Reserved
                DCD     0                         ; 7.Reserved
                DCD     0                         ; 8.Reserved
                DCD     0                         ; 9.Reserved
                DCD     0                         ; 10.Reserved
                DCD     SVC_Handler               ; 11.SVCall Handler
                DCD     0                         ; 12.Reserved
                DCD     0                         ; 13.Reserved
                DCD     PendSV_Handler            ; 14.PendSV Handler
                DCD     0                         ; 15.SysTick Handler
; External Interrupts
												  ;	maximum	of 32 External Interrupts are possible
				DCD		WDT_IRQHandler            ; 0
				DCD		DPWM_IRQHandler           ; 1
				DCD		ADC_IRQHandler            ; 2
				DCD		0                         ; 3
				DCD     SPIM_IRQHandler           ; 4 
				DCD		TMR0_IRQHandler           ; 5
				DCD		TMR1_IRQHandler	          ; 6
				DCD		TMR2_IRQHandler	          ; 7
				DCD		GPAB_IRQHandler	          ; 8
				DCD		SPI0_IRQHandler	          ; 9
				DCD		PWM0_IRQHandler	          ; 10
				DCD		PDMA_IRQHandler           ; 11	
				DCD		TMRF_IRQHandler           ; 12
				DCD		RTC_IRQHandler            ; 13
				DCD		PWRWU_IRQHandler          ; 14
				DCD		PWM1_IRQHandler           ; 15
				DCD		MAC_IRQHandler            ; 16
				DCD		UART0_IRQHandler          ; 17
				DCD		BOD_IRQHandler            ; 18					
				DCD		0						  ;ROM/RAM configuration, 0:  N571P032 mode ;32K ROM; 4K RAM.
			
				AREA	|.text|, CODE, READONLY



; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
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

				EXPORT	WDT_IRQHandler			  [WEAK]
				EXPORT	DPWM_IRQHandler			  [WEAK]
				EXPORT	ADC_IRQHandler			  [WEAK]
				EXPORT	SPIM_IRQHandler	     	  [WEAK]
				EXPORT	TMR0_IRQHandler			  [WEAK]
				EXPORT	TMR1_IRQHandler			  [WEAK]
				EXPORT	TMR2_IRQHandler			  [WEAK]
				EXPORT	GPAB_IRQHandler			  [WEAK]
				EXPORT	SPI0_IRQHandler			  [WEAK]
				EXPORT	PWM0_IRQHandler			  [WEAK]
				EXPORT	PDMA_IRQHandler			  [WEAK]
				EXPORT	TMRF_IRQHandler			  [WEAK]
				EXPORT	RTC_IRQHandler            [WEAK]
				EXPORT	PWRWU_IRQHandler          [WEAK]
				EXPORT	PWM1_IRQHandler			  [WEAK]
				EXPORT	MAC_IRQHandler  		  [WEAK]
				EXPORT	UART0_IRQHandler		  [WEAK]
				EXPORT	BOD_IRQHandler            [WEAK]
                
WDT_IRQHandler	
DPWM_IRQHandler	
ADC_IRQHandler
SPIM_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler	
TMR2_IRQHandler	
GPAB_IRQHandler	
SPI0_IRQHandler	          	
PWM0_IRQHandler	   
PDMA_IRQHandler           
TMRF_IRQHandler
RTC_IRQHandler
PWRWU_IRQHandler
PWM1_IRQHandler          
MAC_IRQHandler     
UART0_IRQHandler          
BOD_IRQHandler            

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

;/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/