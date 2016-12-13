;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3) ;; 8 bytes alignment

    SECTION .intvec:CODE:NOROOT(2);; 4 bytes alignment

    EXTERN  SystemInit	
    EXTERN  __iar_program_start
    PUBLIC  __vector_table

    DATA
__vector_table
    DCD     sfe(CSTACK)
    DCD     Reset_Handler

    DCD     NMI_Handler
    DCD     HardFault_Handler
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     SVC_Handler
    DCD     0
    DCD     0
    DCD     PendSV_Handler
    DCD     SysTick_Handler

    ; External Interrupts
    DCD     BOD_IRQHandler              ; Brown-out low voltage detected interrupt                 
    DCD     WDT_IRQHandler              ; Watch-dog timer interrupt                              
    DCD     EINT0_IRQHandler            ; External signal interrupt from P3.2 pin                
    DCD     EINT1_IRQHandler            ; External signal interrupt from P3.3 pin                
    DCD     GPG0_IRQHandler             ; External interrupt from GPIO group 0 (P0~P4) except P3.2 and P3.3     
    DCD     GPG1_IRQHandler          	; External interrupt from GPIO group 1 (P5~PA)    
    DCD     BPWM0_IRQHandler            ; Basic PWM0 interrupt                              
    DCD     EADC0_IRQHandler            ; EADC0 interrupt                                 
    DCD     TMR0_IRQHandler             ; Timer 0 interrupt                                      
    DCD     TMR1_IRQHandler             ; Timer 1 interrupt                                      
    DCD     TMR2_IRQHandler             ; Timer 2 interrupt                                      
    DCD     TMR3_IRQHandler             ; Timer 3 interrupt                                      
    DCD     UART0_IRQHandler            ; UART0 interrupt                                        
    DCD     UART1_IRQHandler            ; UART1 interrupt                                        
    DCD     SPI0_IRQHandler             ; SPI0 interrupt                                         
    DCD     SPI1_IRQHandler             ; SPI1 interrupt                                         
    DCD     SPI2_IRQHandler             ; SPI2 interrupt                                         
    DCD     MDU_IRQHandler              ; Motor dive unit interrupt                                        
    DCD     I2C0_IRQHandler             ; I2C interrupt                                         
    DCD     CKD_IRQHandler              ; CKD interrupt                                         
    DCD     CAN_IRQHandler              ; Default interrupt                                         
    DCD     EPWM0_IRQHandler            ; Enhanced PWM0 interrupt                                        
    DCD     EPWM1_IRQHandler            ; Enhanced PWM1 interrupt
    DCD     CAP0_IRQHandler             ; Input capture 0 interrupt                             
    DCD     CAP1_IRQHandler             ; Input capture 1 interrupt                                        
    DCD     ACMP_IRQHandler             ; Analog Comparator 0 or 1, or OP Amplifier digital output interrupt
    DCD     QEI0_IRQHandler             ; QEI0 interrupt  
    DCD     QEI1_IRQHandler             ; QEI1 interrupt
    DCD     PWRWU_IRQHandler            ; Clock controller interrupt for chip wake up from power-down state
    DCD     EADC1_IRQHandler            ; EADC1 interrupt                                          
    DCD     EADC2_IRQHandler            ; EADC2 interrupt 
    DCD     EADC3_IRQHandler            ; EADC3 interrupt 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
    THUMB
    PUBWEAK Reset_Handler   
    SECTION .text:CODE:REORDER(2)       ; 4 bytes alignment
Reset_Handler
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

        LDR      R0, =SystemInit
        BLX      R0
        LDR      R0, =__iar_program_start
        BX       R0

    PUBWEAK HardFault_Handler
    PUBWEAK NMI_Handler       
    PUBWEAK SVC_Handler       
    PUBWEAK PendSV_Handler    
    PUBWEAK SysTick_Handler   
    PUBWEAK BOD_IRQHandler  
    PUBWEAK WDT_IRQHandler  
    PUBWEAK EINT0_IRQHandler
    PUBWEAK EINT1_IRQHandler
    PUBWEAK GPG0_IRQHandler 
    PUBWEAK GPG1_IRQHandler 
    PUBWEAK BPWM0_IRQHandler 
    PUBWEAK EADC0_IRQHandler 
    PUBWEAK TMR0_IRQHandler 
    PUBWEAK TMR1_IRQHandler 
    PUBWEAK TMR2_IRQHandler 
    PUBWEAK TMR3_IRQHandler 
    PUBWEAK UART0_IRQHandler
    PUBWEAK UART1_IRQHandler
    PUBWEAK SPI0_IRQHandler 
    PUBWEAK SPI1_IRQHandler 
    PUBWEAK SPI2_IRQHandler 
    PUBWEAK MDU_IRQHandler 
    PUBWEAK I2C0_IRQHandler 
    PUBWEAK CKD_IRQHandler 
    PUBWEAK CAN_IRQHandler
    PUBWEAK EPWM0_IRQHandler
    PUBWEAK EPWM1_IRQHandler 
    PUBWEAK CAP0_IRQHandler  
    PUBWEAK CAP1_IRQHandler  
    PUBWEAK ACMP_IRQHandler 
    PUBWEAK QEI0_IRQHandler
    PUBWEAK QEI1_IRQHandler 
    PUBWEAK PWRWU_IRQHandler
    PUBWEAK EADC1_IRQHandler
    PUBWEAK EADC2_IRQHandler  
    PUBWEAK EADC3_IRQHandler   
    SECTION .text:CODE:REORDER(2)

HardFault_Handler 
NMI_Handler       
SVC_Handler       
PendSV_Handler    
SysTick_Handler   
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
CAN_IRQHandler
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
Default_IRQHandler
    
    END

