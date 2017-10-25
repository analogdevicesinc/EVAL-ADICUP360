/*!
 *****************************************************************************
 * @file:    main.c
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

/***************************** Include Files **********************************/

#include <stdio.h>

#include "Timer.h"

#include "AD7091R.h"
#include "CN0337.h"
#include "Communication.h"


/* Sample pragmas to cope with warnings. Please note the related line at
  the end of this function, used to pop the compiler diagnostics status. */
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

   CN0337_Interrupt();

}

/**
   @brief The main application function

   @return the function contains infinite loop and never returns

**/
int main(int argc, char *argv[])
{
   uint16_t adcValue;
   float temp, voltage, r;


   /* Initialization part */
   AD7091R_Init();
   CN0337_Init();

   timer_start();             /* Start the System Tick Timer. */

   /* Infinite loop */
   while (1) {

      adcValue = AD7091R_Scan();         /* Read ADC output value */

      if(adcValue >= ADC_RESOLUTION) {             /* Check if ADC output value corresponds to 12 bits resolution */

         adcValue = voltage = r = temp = INVALID_DATA;         /* Set invalid value if ADC output value is bigger */

      } else {

         voltage = AD7091R_ConvertToVolts(adcValue, VREF);        /* Convert ADC output value to voltage */

         r = CN0337_CalculateResistance(adcValue, voltage);        /* Calculate RTD resistance */

         temp = CN0337_CalculateTemp(r);                  /* Calculate RTD temperature */
      }

      CN0337_WriteData(temp, r, voltage, adcValue);            /* Write data to UART */

   }
}

#pragma GCC diagnostic pop

/* End Of File */
