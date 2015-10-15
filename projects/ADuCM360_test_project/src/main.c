//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include "diag/Trace.h"
#include "Timer.h"
#include <stdio.h>
#include <I2cLib.h>
#include <UrtLib.h>

#include "Test.h"
#include "Test_config.h"
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

/************************* Functions Definitions ******************************/

/**
  @brief UART Interrupt Handler

  @return none

**/
void UART_Int_Handler (void)
{
   if (UrtIntSta(pADI_UART) & COMIIR_NINT) {
      return;
   }

}

/**
  @brief I2C0 Interrupt Handler

  @return none

**/
void I2C0_Master_Int_Handler(void)
{
   if((I2cSta(MASTER)& I2CMSTA_TCOMP_SET) == I2CMSTA_TCOMP_SET) {

      I2cMCfg(0, I2CMCON_IENCMP | I2CMCON_IENTX_EN | I2CMCON_IENRX_EN, I2CMCON_LOOPBACK_EN | I2CMCON_MAS_EN);
   }

}


/**
  @brief Main function

**/
int main (int argc, char *argv[])
{

   Test_Init();    /* Init Test project */

   timer_start();   /* Start timer */


   while(1) {

      if(FUNCTION_TO_TEST == GPIO) {
         Test_Port();   /* Test ports */

      } else {
         Test_Periph();   /* Test peripherals */
      }
   }

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
