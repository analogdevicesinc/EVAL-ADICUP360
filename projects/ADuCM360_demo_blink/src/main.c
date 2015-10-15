//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include "diag/Trace.h"
#include "blink.h"
#include "Timer.h"


// ----------------------------------------------------------------------------
//
// Print a greeting message on the trace device and enter a loop
// to count seconds.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// ----------------------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

const unsigned use_irq = 1;

void GP_Tmr0_Int_Handler(void)
{
   Blink_Process();
   /* Clears current Timer interrupt */
   GptClrInt(pADI_TM0, TSTA_TMOUT);
}

int main (int argc, char *argv[])
{
   /* Initialize GPIO */
   Blink_Init();

   if (use_irq) {
      /* Initialize the general purpose timer0 */
      GptLd(pADI_TM0, 63);                                  // Set timeout period for 0.5 seconds
      GptCfg(pADI_TM0, TCON_CLK_LFOSC, TCON_PRE_DIV256, TCON_MOD_PERIODIC | TCON_ENABLE);
      NVIC_EnableIRQ(TIMER0_IRQn);      // Enable Timer0 IRQ

   } else {
      /* Configure the system's tick interrupt */
      timer_start ();
   }

   /* Main Program Loop */
   while (1) {
      if(!use_irq) {
         /* Configure blinking interval */
         timer_sleep(BLINK_TIME * TIMER_FREQUENCY_HZ);
         Blink_Process();
      }
   }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
