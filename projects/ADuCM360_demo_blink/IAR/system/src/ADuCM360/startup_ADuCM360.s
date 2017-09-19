;/**************************************************************************//**
; * @file     startup_ADuCM360.s
; * @brief    CMSIS Cortex-M3 Core Device Startup File for
; *           Device ADuCM360
; * @version  V3.10
; * @date     23. November 2012
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2012 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;
;   Portions Copyright (c) 2017 Analog Devices, Inc.
;   ---------------------------------------------------------------------------*/

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     Nmi_Handler                                 ; The NMI handler        
        DCD     Fault_Handler                              ; The hard fault handler 
        DCD     MemManage_Handler                     ; The MPU fault handler  
        DCD     BusFault_Handler                      ; The bus fault handler  
        DCD     UsageFault_Handler                    ; The usage fault handler
        DCD     0                                     ; Reserved               
        DCD     0                                     ; Reserved               
        DCD     0                                     ; Reserved               
        DCD     0                                     ; Reserved               
        DCD     SVC_Handler                           ; SVCall handler         
        DCD     DebugMon_Handler                      ; Debug monitor handler  
        DCD     0                                     ; Reserved               
        DCD     PendSV_Handler                        ; The PendSV handler     
        DCD     SysTick_Handler                       ; The SysTick handler    


              ; External Interrupts
	        DCD     WakeUp_Int_Handler        ; Wake Up Timer              [ 0]
	        DCD     Ext_Int0_Handler          ; External Interrupt 0       [ 1]
	        DCD     Ext_Int1_Handler          ; External Interrupt 1       [ 2]
	        DCD     Ext_Int2_Handler          ; External Interrupt 2       [ 3]
	        DCD     Ext_Int3_Handler          ; External Interrupt 3       [ 4]
	        DCD     Ext_Int4_Handler          ; External Interrupt 4       [ 5]
	        DCD     Ext_Int5_Handler          ; External Interrupt 5       [ 6]
	        DCD     Ext_Int6_Handler          ; External Interrupt 6       [ 7]
	        DCD     Ext_Int7_Handler          ; External Interrupt 7       [ 8]
	        DCD     WDog_Tmr_Int_Handler      ; Watchdog timer handler     [ 9]
	        DCD     0                         ;                            [10]
	        DCD     GP_Tmr0_Int_Handler       ; General purpose timer 0    [11]
	        DCD     GP_Tmr1_Int_Handler       ; General purpose timer 1    [12]
	        DCD     ADC0_Int_Handler          ; ADC0 Interrupt             [13]
	        DCD     ADC1_Int_Handler          ; ADC1 Interrupt             [14]
	        DCD     SINC2_Int_Handler         ; SINC2 Interrupt            [15]
	        DCD     Flsh_Int_Handler          ; Flash Interrupt            [16]
	        DCD     UART_Int_Handler          ; UART0                      [17]
	        DCD     SPI0_Int_Handler          ; SPI 0                      [18]
	        DCD     SPI1_Int_Handler          ; SPI 1                      [19]
	        DCD     I2C0_Slave_Int_Handler    ; I2C0 Slave                 [20]
	        DCD     I2C0_Master_Int_Handler   ; I2C0 Master                [21]
	        DCD     DMA_Err_Int_Handler       ; DMA Error interrupt        [22]
	        DCD     DMA_SPI1_TX_Int_Handler   ; DMA SPI1 TX                [23]
	        DCD     DMA_SPI1_RX_Int_Handler   ; DMA SPI1 RX                [24]
	        DCD     DMA_UART_TX_Int_Handler   ; DMA UART TX                [25]
	        DCD     DMA_UART_RX_Int_Handler   ; DMA UART RX                [26]
	        DCD     DMA_I2C0_STX_Int_Handler  ; DMA I2C0 Slave TX          [27]
	        DCD     DMA_I2C0_SRX_Int_Handler  ; DMA I2C0 Slave RX          [28]
	        DCD     DMA_I2C0_MTX_Int_Handler  ; DMA I2C0 Master TX         [29]
	        DCD     DMA_I2C0_MRX_Int_Handler  ; DMA I2C0 Master RX         [30]
	        DCD     DMA_DAC_Out_Int_Handler   ; DMA DAC out                [31]
	        DCD     DMA_ADC0_Int_Handler      ; DMA ADC0                   [32]
	        DCD     DMA_ADC1_Int_Handler      ; DMA ADC1                   [33]
	        DCD     DMA_SINC2_Int_Handler     ; SINC2                      [34]
	        DCD     PWMTRIP_Int_Handler       ; PWMTRIP                    [35]
	        DCD     PWM0_Int_Handler          ; PWM0                       [36]
	        DCD     PWM1_Int_Handler          ; PWM1                       [37]
	        DCD     PWM2_Int_Handler          ; PWM2                       [38]
	        DCD     0                         ;                            [39]

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
Reset_Handler
            IMPORT  SystemInit
            LDR         R0, =SystemInit
            BLX         R0
            LDR         R0, =__iar_program_start
            BX          R0
            
	    PUBWEAK SysTick_Handler
	    PUBWEAK Nmi_Handler
	    PUBWEAK Fault_Handler
	    PUBWEAK MemManage_Handler
	    PUBWEAK BusFault_Handler
	    PUBWEAK UsageFault_Handler
	    PUBWEAK SVC_Handler
	    PUBWEAK DebugMon_Handler
	    PUBWEAK PendSV_Handler
	    PUBWEAK WakeUp_Int_Handler 
            PUBWEAK Ext_Int0_Handler        
            PUBWEAK Ext_Int1_Handler        
            PUBWEAK Ext_Int2_Handler        
            PUBWEAK Ext_Int3_Handler        
            PUBWEAK Ext_Int4_Handler        
            PUBWEAK Ext_Int5_Handler        
            PUBWEAK Ext_Int6_Handler        
            PUBWEAK Ext_Int7_Handler        
            PUBWEAK WDog_Tmr_Int_Handler    
            PUBWEAK GP_Tmr0_Int_Handler     
            PUBWEAK GP_Tmr1_Int_Handler     
            PUBWEAK ADC0_Int_Handler        
            PUBWEAK ADC1_Int_Handler        
            PUBWEAK SINC2_Int_Handler       
            PUBWEAK Flsh_Int_Handler        
            PUBWEAK UART_Int_Handler        
            PUBWEAK SPI0_Int_Handler        
            PUBWEAK SPI1_Int_Handler        
            PUBWEAK I2C0_Slave_Int_Handler  
            PUBWEAK I2C0_Master_Int_Handler 
            PUBWEAK DMA_Err_Int_Handler     
            PUBWEAK DMA_SPI1_TX_Int_Handler 
            PUBWEAK DMA_SPI1_RX_Int_Handler 
            PUBWEAK DMA_UART_TX_Int_Handler 
            PUBWEAK DMA_UART_RX_Int_Handler 
            PUBWEAK DMA_I2C0_STX_Int_Handler
            PUBWEAK DMA_I2C0_SRX_Int_Handler
            PUBWEAK DMA_I2C0_MTX_Int_Handler
            PUBWEAK DMA_I2C0_MRX_Int_Handler
            PUBWEAK DMA_DAC_Out_Int_Handler 
            PUBWEAK DMA_ADC0_Int_Handler    
            PUBWEAK DMA_ADC1_Int_Handler    
            PUBWEAK DMA_SINC2_Int_Handler   
            PUBWEAK PWMTRIP_Int_Handler     
            PUBWEAK PWM0_Int_Handler        
            PUBWEAK PWM1_Int_Handler        
            PUBWEAK PWM2_Int_Handler        
	    PUBWEAK UnUsed_Handler



        THUMB
        SECTION .text:CODE:REORDER:NOROOT(1)
Nmi_Handler
Fault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler
WakeUp_Int_Handler      
Ext_Int0_Handler        
Ext_Int1_Handler        
Ext_Int2_Handler        
Ext_Int3_Handler        
Ext_Int4_Handler        
Ext_Int5_Handler        
Ext_Int6_Handler        
Ext_Int7_Handler        
WDog_Tmr_Int_Handler    
GP_Tmr0_Int_Handler     
GP_Tmr1_Int_Handler     
ADC0_Int_Handler        
ADC1_Int_Handler        
SINC2_Int_Handler       
Flsh_Int_Handler        
UART_Int_Handler        
SPI0_Int_Handler        
SPI1_Int_Handler        
I2C0_Slave_Int_Handler  
I2C0_Master_Int_Handler 
DMA_Err_Int_Handler     
DMA_SPI1_TX_Int_Handler 
DMA_SPI1_RX_Int_Handler 
DMA_UART_TX_Int_Handler 
DMA_UART_RX_Int_Handler 
DMA_I2C0_STX_Int_Handler
DMA_I2C0_SRX_Int_Handler
DMA_I2C0_MTX_Int_Handler
DMA_I2C0_MRX_Int_Handler
DMA_DAC_Out_Int_Handler 
DMA_ADC0_Int_Handler    
DMA_ADC1_Int_Handler    
DMA_SINC2_Int_Handler   
PWMTRIP_Int_Handler     
PWM0_Int_Handler        
PWM1_Int_Handler        
PWM2_Int_Handler        
UnUsed_Handler
        B UnUsed_Handler

        END
