;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

CLK_BA_base      EQU 0x50000200
PWRCON			 EQU 0x00
AHBCLK			 EQU 0x04
APBCLK			 EQU 0x08
CLKSEL0			 EQU 0x10
CLKSEL1			 EQU 0x14
CLKDIV			 EQU 0x18
PLLCON			 EQU 0x20
TEST_S			 EQU 0x30

CLK_BA_APBCLK    EQU 0x50000208

;// Define clock enable registers

ADC_COMP_CLK     EQU 0x50000208
ADC_enable		 EQU 0x10000000
COMP_enable      EQU 0x40000000

PDMA_CLK         EQU 0x50000204
PDMA_enable      EQU 0x00000003

;;  bit 0  CPU_EN
;;	bit 1  PDMA_EN

;// Define COMP registers base
COMP_base        EQU  0x400D0000
CMP1CR           EQU  0x00
CMP2CR           EQU  0x04
CMPSR            EQU  0x08

;// Define ADC registers base
ADC_base         EQU  0x400E0000
ADDR0            EQU  0x00
ADDR1            EQU  0x04
ADDR2            EQU  0x08
ADDR3            EQU  0x0c
ADDR4            EQU  0x10
ADDR5            EQU  0x14
ADDR6            EQU  0x18
ADDR7            EQU  0x1c
ADCR             EQU  0x20
ADCHER           EQU  0x24
ADCMPR0          EQU  0x28
ADCMPR1          EQU  0x2c
ADSR             EQU  0x30
ADCALR           EQU  0x34
ADCFCR           EQU  0x38
ADCALD           EQU  0x3c

;// Pattern Table
pattern_55555555 EQU  0x55555555
pattern_aaaaaaaa EQU  0xaaaaaaaa
pattern_00005555 EQU  0x00005555
pattern_0000aaaa EQU  0x0000aaaa
pattern_05550515 EQU  0x05550515
pattern_0aaa0a2a EQU  0x0aaa0a2a

;// Define PDMA regsiter base
PDMA_BA_ch0_base        EQU  0x50008000
PDMA_BA_ch1_base        EQU  0x50008100
PDMA_BA_ch2_base        EQU  0x50008200
PDMA_BA_ch3_base        EQU  0x50008300
PDMA_BA_ch4_base        EQU  0x50008400
PDMA_BA_ch5_base        EQU  0x50008500
PDMA_BA_ch6_base        EQU  0x50008600
PDMA_BA_ch7_base        EQU  0x50008700

PDMA_BA_GCR             EQU 0x50008F00
PDMA_BA_GCR_base        EQU 0x50008F00

PDMA_GCRCSR		 EQU  0X00
PDMA_PDSSR2		 EQU  0X04
PDMA_PDSSR1		 EQU  0X08  ;; PDMA channel select   0x77000000
PDMA_GCRISR		 EQU  0X0C

PDMA_GLOBAL_enable      EQU 0x0000FF00

PDMA_CSR         EQU  0X00
PDMA_SAR         EQU  0X04
PDMA_DAR         EQU  0X08
PDMA_BCR         EQU  0X0C
PDMA_CSAR        EQU  0X14
PDMA_CDAR        EQU  0X18
PDMA_CBSR        EQU  0X1C
PDMA_IER         EQU  0X20
PDMA_ISR         EQU  0X24
PDMA_CTCSR       EQU  0X28
PDMA_SASOCR		 EQU  0X2C
PDMA_DASOCR      EQU  0X30
PDMA_SBUF0       EQU  0X80
PDMA_SBUF1       EQU  0X84
PDMA_SBUF2       EQU  0X88
PDMA_SBUF3       EQU  0X8C

;// Define VIC control register
VIC_base         EQU  0xFFFF0000
VIC_SCR15        EQU  0x003c
VIC_SVR15        EQU  0x00bc
VIC_SCR16        EQU  0x0040
VIC_SVR16        EQU  0x00c0
VIC_SCR30        EQU  0x0078
VIC_SVR30        EQU  0x00f8
VIC_MECR         EQU  0x0318
VIC_MDCR         EQU  0x031c
VIC_EOSCR        EQU  0x0130

;//==================================
INT_BA_base      EQU  0x50000300

;// Parameter table
ADC_PDMA_CFG     EQU  0x00002980
ADC_PDMA_DST     EQU  0xC0000000
ADC_PDMA_SRC     EQU  0xE0024200
ADC_PDMA_TCBL    EQU  0x00030008

;//==================================

GPIO_base        EQU  0x50004000
GPIOB_PMD		 EQU  0x0040
GPIOB_OFFD		 EQU  0x0044
GPIOB_DOUT		 EQU  0x0048
GPIOB_DMASK		 EQU  0x004C
GPIOB_PIN		 EQU  0x0050
GPIOB_DBEN		 EQU  0x0054
GPIOB_IMD		 EQU  0x0058
GPIOB_IEN		 EQU  0x005C
GPIOB_ISRC		 EQU  0x0060

;//==================================

GCR_base         EQU  0x50000000
GPB_MFP          EQU  0x0034

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


Stack_Size      EQU     0x00000400

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

__Vectors       DCD     __initial_sp            ; Top of Stack
                DCD     Reset_Handler           ; Reset Handler
                DCD     NMI_Handler             ; NMI Handler
                DCD     HardFault_Handler       ; Hard Fault Handler
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     SVC_Handler             ; SVCall Handler
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     PendSV_Handler          ; PendSV Handler
                DCD     SysTick_Handler         ; SysTick Handler

                ; External Interrupts
                ; maximum of 32 External Interrupts are possible
                DCD     BOD_IRQHandler          ;  0: Brown-Out low voltage detected interrupt
                DCD     WDT_IRQHandler          ;  1: Watchdog Timer interrupt
                DCD     USCI0_IRQHandler        ;  2: USCI0 interrupt
                DCD     USCI1_IRQHandler        ;  3: USCI1 interrupt
                DCD     GPABCD_IRQHandler       ;  4: External interrupt from GPA ~ GPD pins
                DCD     EPWM_IRQHandler         ;  5: EPWM interrupt
                DCD     BRAKE0_IRQHandler       ;  6: EPWM brake interrupt from PWM0 or PWM_BRAKE pin
                DCD     BRAKE1_IRQHandler       ;  7: EPWM brake interrupt from PWM1
                DCD     BPWM0_IRQHandler        ;  8: BPWM0 interrupt
                DCD     BPWM1_IRQHandler        ;  9: BPWM1 interrupt
                DCD     Default_Handler         ; 10: Reserved
                DCD     Default_Handler         ; 11: Reserved
                DCD     Default_Handler         ; 12: Reserved
                DCD     Default_Handler         ; 13: Reserved
                DCD     Default_Handler         ; 14: Reserved
                DCD     ECAP_IRQHandler         ; 15: Enhanced Input Capture interrupt
                DCD     CCAP_IRQHandler         ; 16: Continues Input Capture interrupt
                DCD     Default_Handler         ; 17: Reserved
                DCD     Default_Handler         ; 18: Reserved
                DCD     Default_Handler         ; 19: Reserved
                DCD     Default_Handler         ; 20: Reserved
                DCD     HIRCTRIM_IRQHandler     ; 21: HIRC TRIM interrupt
                DCD     TMR0_IRQHandler         ; 22: Timer 0 interrupt
                DCD     TMR1_IRQHandler         ; 23: Timer 1 interrupt
                DCD     Default_Handler         ; 24: Reserved
                DCD     Default_Handler         ; 25: Reserved
                DCD     ACMP_IRQHandler         ; 26: Analog Comparator 0 or Comparator 1 interrupt
                DCD     Default_Handler         ; 27: Reserved
                DCD     PWRWU_IRQHandler        ; 28: Chip wake-up from Power-down state interrupt
                DCD     EADC0_IRQHandler        ; 29: EADC0 interrupt
                DCD     EADC1_IRQHandler        ; 30: EADC1 interrupt
                DCD     EADCWCMP_IRQHandler     ; 31: EADC Window Compare interrupt

                AREA    |.text|, CODE, READONLY

; Reset Handler

                ENTRY

Reset_Handler   PROC
                EXPORT  Reset_Handler           [WEAK]
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
                EXPORT  NMI_Handler             [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler       [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler             [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler          [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler         [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler          [WEAK]
                EXPORT  WDT_IRQHandler          [WEAK]
                EXPORT  USCI0_IRQHandler        [WEAK]
                EXPORT  USCI1_IRQHandler        [WEAK]
                EXPORT  GPABCD_IRQHandler       [WEAK]
                EXPORT  EPWM_IRQHandler         [WEAK]
                EXPORT  BRAKE0_IRQHandler       [WEAK]
                EXPORT  BRAKE1_IRQHandler       [WEAK]
                EXPORT  BPWM0_IRQHandler        [WEAK]
                EXPORT  BPWM1_IRQHandler        [WEAK]
                EXPORT  ECAP_IRQHandler         [WEAK]
                EXPORT  CCAP_IRQHandler         [WEAK]
                EXPORT  HIRCTRIM_IRQHandler     [WEAK]
                EXPORT  TMR0_IRQHandler         [WEAK]
                EXPORT  TMR1_IRQHandler         [WEAK]
                EXPORT  ACMP_IRQHandler         [WEAK]
                EXPORT  PWRWU_IRQHandler        [WEAK]
                EXPORT  EADC0_IRQHandler        [WEAK]
                EXPORT  EADC1_IRQHandler        [WEAK]
                EXPORT  EADCWCMP_IRQHandler     [WEAK]


BOD_IRQHandler
WDT_IRQHandler
USCI0_IRQHandler
USCI1_IRQHandler
GPABCD_IRQHandler
EPWM_IRQHandler
BRAKE0_IRQHandler
BRAKE1_IRQHandler
BPWM0_IRQHandler
BPWM1_IRQHandler
ECAP_IRQHandler
CCAP_IRQHandler
HIRCTRIM_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
ACMP_IRQHandler
PWRWU_IRQHandler
EADC0_IRQHandler
EADC1_IRQHandler
EADCWCMP_IRQHandler

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
