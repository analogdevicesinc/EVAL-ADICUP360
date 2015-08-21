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
#include <GptLib.h>
#include <UrtLib.h>

#include "Communication.h"
#include "Cli.h"
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

/*Interrupt handler for UART */
void UART_Int_Handler ()
{
    Cli_Int_Handler();
}

int main (int argc, char* argv[])
{

	UART_Init(B9600, COMLCR_WLS_8BITS);  /* UART initialization */

	timer_start();


	while(1)
	{

		Cli_Process();    /* Command interpreter */
	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
