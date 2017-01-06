;/**************************************************************************//**
; * @file     startup_M051Series.s
; * @version  V2.00
; * $Revision: 4 $
; * $Date: 16/06/28 2:13p $ 
; * @brief    M051 Series Startup Source File for IAR Platform
; *
; * @note
; * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
; *
; ******************************************************************************/

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
    DCD     BOD_IRQHandler              ; Brownout low voltage detected interrupt                 
    DCD     WDT_IRQHandler              ; Watch Dog Timer interrupt                              
    DCD     EINT024_IRQHandler          ; External interrupt from PA[0]/PC[0]/PD[2]/PE[0]/PE[4] pin               
    DCD     EINT135_IRQHandler          ; External interrupt from PB[0]/PD[0]/PD[3]/PE[5]/PF[0] pin            
    DCD     GPAB_IRQHandler             ; External interrupt from PA[15:0]/PB[15:0]    
    DCD     GPCDEF_IRQHandler           ; External interrupt from PC[15:0]/PD[15:0]/PE[13:0]/PF[7:0]    
    DCD     PWM0_IRQHandler             ; PWM0 interrupt                                 
    DCD     PWM1_IRQHandler             ; PWM1 interrupt                                 
    DCD     TMR0_IRQHandler             ; Timer 0 interrupt                                      
    DCD     TMR1_IRQHandler             ; Timer 1 interrupt                                      
    DCD     TMR2_IRQHandler             ; Timer 2 interrupt                                      
    DCD     TMR3_IRQHandler             ; Timer 3 interrupt                                      
    DCD     UART02_IRQHandler           ; UART0 and UART2 interrupt                                        
    DCD     UART1_IRQHandler            ; UART1 interrupt                                        
    DCD     SPI0_IRQHandler             ; SPI0 interrupt                                         
    DCD     SPI1_IRQHandler             ; SPI1 interrupt                                         
    DCD     Default_Handler             ; SPI2 interrupt                                         
    DCD     Default_Handler             ; SPI3 interrupt                                         
    DCD     I2C0_IRQHandler             ; I2C0 interrupt                                         
    DCD     I2C1_IRQHandler             ; I2C1 interrupt                                        
    DCD     Default_Handler             ; Reserved                                        
    DCD     Default_Handler             ; Reserved                                         
    DCD     USCI_IRQHandler             ; USCI interrupt
    DCD     USBD_IRQHandler             ; USBD interrupt
    DCD     SC01_IRCHandler             ; SC0 and SC1 interrupt
    DCD     ACMP01_IRQHandler           ; ACMP0/1 interrupt
    DCD     PDMA_IRQHandler             ; PDMA interrupt
    DCD     Default_Handler             ; Reserved
    DCD     PWRWU_IRQHandler            ; Clock controller interrupt for chip wake up from power-
    DCD     ADC_IRQHandler              ; ADC interrupt                                          
    DCD     CLKDIRC_IRQHandler          ; Clock fail detect and IRC TRIM interrupt
    DCD     RTC_IRQHandler              ; Real time clock interrupt                              

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

        ; Disable NMI (Assign to reserved IRQ)
        LDR     R2, =0x50000380
        LDR     R1, =0x0000001F
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
    PUBWEAK EINT024_IRQHandler 
    PUBWEAK EINT135_IRQHandler 
    PUBWEAK GPAB_IRQHandler  
    PUBWEAK GPCDEF_IRQHandler
    PUBWEAK PWM0_IRQHandler 
    PUBWEAK PWM1_IRQHandler 
    PUBWEAK TMR0_IRQHandler 
    PUBWEAK TMR1_IRQHandler 
    PUBWEAK TMR2_IRQHandler 
    PUBWEAK TMR3_IRQHandler 
    PUBWEAK UART02_IRQHandler
    PUBWEAK UART1_IRQHandler
    PUBWEAK SPI0_IRQHandler
    PUBWEAK SPI1_IRQHandler
    PUBWEAK I2C0_IRQHandler
    PUBWEAK I2C1_IRQHandler
	PUBWEAK USCI_IRQHandler
    PUBWEAK USBD_IRQHandler
	PUBWEAK SC01_IRCHandler
    PUBWEAK ACMP01_IRQHandler 
    PUBWEAK PDMA_IRQHandler
    PUBWEAK PWRWU_IRQHandler  
    PUBWEAK ADC_IRQHandler
    PUBWEAK CLKDIRC_IRQHandler
    PUBWEAK RTC_IRQHandler  
    SECTION .text:CODE:REORDER(2)
HardFault_Handler 
NMI_Handler       
SVC_Handler       
PendSV_Handler    
SysTick_Handler   
BOD_IRQHandler   
WDT_IRQHandler   
EINT024_IRQHandler 
EINT135_IRQHandler 
GPAB_IRQHandler  
GPCDEF_IRQHandler
PWM0_IRQHandler  
PWM1_IRQHandler  
TMR0_IRQHandler  
TMR1_IRQHandler  
TMR2_IRQHandler  
TMR3_IRQHandler  
UART02_IRQHandler 
UART1_IRQHandler 
SPI0_IRQHandler  
SPI1_IRQHandler   
I2C0_IRQHandler
I2C1_IRQHandler
USCI_IRQHandler
USBD_IRQHandler
SC01_IRCHandler
ACMP01_IRQHandler
PDMA_IRQHandler
PWRWU_IRQHandler
ADC_IRQHandler
CLKDIRC_IRQHandler
RTC_IRQHandler
Default_Handler
    B Default_Handler         

    
    END

