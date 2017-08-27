//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include "Communication.h"
#include "Timer.h"
#include "CN0398.h"

#include "UrtLib.h"


CN0398 cn0398;
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the NONE output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#ifdef  __cplusplus
extern "C" {
#endif
void UART_Int_Handler (void)
 {

   volatile unsigned char ucCOMIID0 = 0;

   ucCOMIID0 = UrtIntSta(pADI_UART);   // Read UART Interrupt ID register
   if ((ucCOMIID0 & 0x2) == 0x2)       // Transmit buffer empty
   {
      ucTxBufferEmpty = 1;
   }
   if ((ucCOMIID0 & 0x4) == 0x4)       // Receive byte
   {
         Rx_char = UART.ReadChar();
         read_ch = true;
   }
 }


#ifdef __cplusplus
} //extern "C"
#endif


int
main(int argc, char* argv[])
{
  
  timer.start();

  cn0398.setup();
  cn0398.init();

  timer.sleep(500);
  printf("Initialization complete!\r\n");
  printf("\n");

  printf("Do you want to perform pH calibration [y/N]?\n");
  char response;
  scanf("%c",&response);
  printf("\n");

  if(response == 'y' || response == 'Y'){

     cn0398.calibrate_ph();

  } else{

          cn0398.use_nernst = true;
          printf("Do you want to load default calibration?[y/N]. If not[N], the Nernst equation will be used.\n");
          char response;
          scanf("%c",&response);
          if(response == 'y' || response == 'Y')
          {
             cn0398.use_nernst = false;
          }
  }

  printf("\n");

  // Infinite loop
  while (1)
    {

        timer.sleep(DISPLAY_REFRESH);
        cn0398.set_data();
        cn0398.display_data();

	  
    }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
