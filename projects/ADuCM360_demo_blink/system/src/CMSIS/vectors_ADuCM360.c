//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu
//

// ----------------------------------------------------------------------------

#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

void __attribute__((weak))
Default_Handler(void);

// Forward declaration of the specific IRQ handlers. These are aliased
// to the Default_Handler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
// TODO: Rename this and add the actual routines here.

//void __attribute__ ((weak, alias ("Default_Handler"))) WakeUp_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int0_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int1_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int2_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int3_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int4_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int5_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int6_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Ext_Int7_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) Watchdog_Int_Handler(void);
void __attribute__ ((weak, alias ("Default_Handler"))) GP_Tmr0_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) GP_Tmr1_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) ADC0_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) ADC1_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) SINC2_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) UART_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) SPI0_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) SPI1_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) I2C0_Slave_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) PWMTRIP_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) PWM0_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) PWM1_Int_Handler(void);
//void __attribute__ ((weak, alias ("Default_Handler"))) PWM2_Int_Handler(void);

// ----------------------------------------------------------------------------

extern unsigned int _estack;

typedef void
(* const pHandler)(void);

// ----------------------------------------------------------------------------

// The vector table.
// This relies on the linker script to place at correct location in memory.

__attribute__ ((section(".isr_vector")))
pHandler __isr_vectors[] =
  { //
    (pHandler) &_estack,                          // The initial stack pointer
        Reset_Handler,                            // The reset handler

        NMI_Handler,                              // The NMI handler
        HardFault_Handler,                        // The hard fault handler

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
        MemManage_Handler,                        // The MPU fault handler
        BusFault_Handler,// The bus fault handler
        UsageFault_Handler,// The usage fault handler
#else
        0, 0, 0,				  // Reserved
#endif
        0,                                        // Reserved
        0,                                        // Reserved
        0,                                        // Reserved
        0,                                        // Reserved
        SVC_Handler,                              // SVCall handler
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
        DebugMon_Handler,                         // Debug monitor handler
#else
        0,					  // Reserved
#endif
        0,                                        // Reserved
        PendSV_Handler,                           // The PendSV handler
        SysTick_Handler,                          // The SysTick handler

        // ----------------------------------------------------------------------
        // ADuCM360 vectors
        Default_Handler,                  		  // [00] WakeUp_Int_Handler
        Default_Handler,                  		  // [01]
        Default_Handler,                  		  // [02]
        Default_Handler,                  		  // [03]
        Default_Handler,                  		  // [04]
        Default_Handler,                  		  // [05]
        Default_Handler,                  		  // [06]
        Default_Handler,                  		  // [07]
        Default_Handler,                  		  // [08]
        Default_Handler,                  		  // [09]
        Default_Handler,                  		  // [10]
		GP_Tmr0_Int_Handler,               		  // [11] //GP_Tmr0_Int_Handler
        Default_Handler,                  		  // [12]
        Default_Handler,                  		  // [13]
        Default_Handler,                  		  // [14] ADC 1
        Default_Handler,                  		  // [15]
        Default_Handler,                  		  // [16]
        Default_Handler,                  		  // [17]
        Default_Handler,                  		  // [18]
        Default_Handler,                  		  // [19]
        Default_Handler,                  		  // [20]
        Default_Handler,               		  	  // [21]
        Default_Handler,                  		  // [22]
        Default_Handler,                  		  // [23]
        Default_Handler,                  		  // [24]
        Default_Handler,                  		  // [25]
        Default_Handler,                  		  // [26]
        Default_Handler,                  		  // [27]
        Default_Handler,                  		  // [28]
        Default_Handler,                  		  // [29]

    // TODO: rename and add more vectors here
    };

// ----------------------------------------------------------------------------

// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.

void __attribute__ ((section(".after_vectors")))
Default_Handler(void)
{
  while (1)
    {
    }
}

// ----------------------------------------------------------------------------
