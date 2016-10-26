/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/


/***************************** Include Files **********************************/

#include <stdio.h>
#include <stdlib.h>

#include "diag/Trace.h"
#include "Timer.h"

#include "Communication.h"
#include "CN0394.h"


#include <ADuCM360.h>

#include "ADC.h"

#include <AdcLib.h>
#include "IntLib.h"
#include "UrtLib.h"



// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/**
   @brief Interrupt handler for UART

   @return none

**/
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


void ADC0_Int_Handler(void)
{

  adc0_data = AdcRd(pADI_ADC0);

  if(calibFlag0 == false){

      if(write0 == 4)
         write0 = 0;

      adc0[write0] = adc0_data;
      write0++;
   }
}

/**
   @brief Interrupt handler for ADC1

   @return none

**/
void ADC1_Int_Handler(void)
{
   adc1_data = AdcRd(pADI_ADC1);

   if(calibFlag1 == false){

      if(write1 == 4)
         write1 = 0;

      adc1[write1] = adc1_data;
      write1++;
   }

}

/**
   @brief The main application function

   @return the function contains infinite loop and never returns

**/
int main(int argc, char *argv[])
{

   timer_start();

   UART_Init(B19200, COMLCR_WLS_8BITS);

   CN0394_Init();

   timer_sleep(500);

#if(USE_RTD_CALIBRATION == YES)
   CN0394_Calibration(RTD_CHANNEL);
   printf("RTD channel calibration completed!\n");
#else
   printf("Calibration for RTD channel is disabled.\n");
#endif
   printf("\n");

   timer_sleep(1);

#if(USE_TH_CALIBRATION == YES)
   CN0394_Calibration(TH_CHANNEL);
   printf("TC channel calibration completed!\n");
#else
   printf("Calibration for TC channel is disabled.\n");
#endif
   printf("\n");

   while (1) {

         timer_sleep(DISPLAY_REFRESH);

         CN0394_SetData();

         CN0394_DisplayData();
   }

}

#pragma GCC diagnostic pop


