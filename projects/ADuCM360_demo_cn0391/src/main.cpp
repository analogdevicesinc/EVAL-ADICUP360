/*!
 *****************************************************************************
 * @file:    main.cpp
 * @brief:   Main source file
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <stdio.h>

#include <stdlib.h>


#include "ADuCM360.h"
#include "UrtLib.h"

#include "Communication.h"
#include "Timer.h"
#include "AD7124.h"
#include "CN0391.h"


AD7124 Ad7124;
CN0391 cn0391;

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
      ucWaitForUart = 0;
   }
 }


#ifdef __cplusplus
} //extern "C"
#endif

int main(int argc, char *argv[])
{
   SystemCoreClockUpdate();
   timer.start();

   UART.Init();

   cn0391.init();

#if(USE_RTD_CALIBRATION == YES)
   cn0391.calibration(RTD_CHANNEL);
   printf("RTD channel calibration completed!\n");
#else
   printf("Calibration for RTD channel is disabled.\n");
#endif
   printf("\n");

#if(USE_TH_CALIBRATION == YES)
   cn0391.calibration(TH_CHANNEL);
   printf("TC channel calibration completed!\n");
#else
   printf("Calibration for TC channel is disabled.\n");
#endif
   printf("\n");

	while(1)
	{
	      timer.sleep(DISPLAY_REFRESH);
	      cn0391.set_data();
	      cn0391.display_data();
	}
	return 0;
}
#pragma GCC diagnostic pop
