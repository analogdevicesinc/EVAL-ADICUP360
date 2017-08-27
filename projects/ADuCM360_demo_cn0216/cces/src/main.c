/**
******************************************************************************
*   @file     main.c
*   @brief    main program file for the ADuCM360_demo_cn0216_ardz
*   @version  V0.1
*   @author   ADI
*   @date     November 2015
*   @par Revision History:
*  - V0.1, November 2015: initial version.
*
*******************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
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


// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include <ADUCM360.h>
#include <UrtLib.h>

#include "CN0216.h"
#include "AD7791.h"
#include "Communication.h"

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

/**
  @brief Looks to see if there is anything available in the UART.

  @return none
**/
void UART_Int_Handler (void)
{

   unsigned short  status;
   char c;

   status = UrtIntSta(pADI_UART);                      /* Check UART status */

   if (status & COMIIR_NINT) {
      return;   /* Check if UART is busy */
   }

   switch (status & COMIIR_STA_MSK) {                  /* Check what command to execute */
   case COMIIR_STA_RXBUFFULL:                         /* Check if UART register is available to be read */

      UART_ReadChar(&c);                             /* Read character from UART */

      if( c == 13) {                            /* Check if read character is ENTER */
         uart_cmd = UART_TRUE;                        /* Set flag */
      }

      break;


   case COMIIR_STA_TXBUFEMPTY:                      /* Check if UART register is available to be written */

      if (uart_tcnt) {                              /* Check uart counter */

         uart_tbusy = UART_TRUE;                    /* UART is busy with writing*/

         uart_tcnt--;                               /* Decrease  uart counter */

         UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);          /* Write character to UART */

         if (uart_tpos == UART_TX_BUFFER_SIZE) {                         /* Check if TX buffer is full */
            uart_tpos = 0;                      /* Reset buffer counter  */
         }

      } else {

         uart_tbusy = UART_FALSE;                  /* UART is no longer busy with writing */
      }

      break;

   default:
      ;
   }

}

int main (int argc, char *argv[])
{
   /* Program variables */
   uint32_t ui32AdcValue;
   float fVoltage, fWeight;

   /* Start the System Tick Timer. */
   timer_start();

   /* Initialize AD7791 */
   AD7791_Init();
   /* Initialize CN0216 application */
   CN0216_Init();

   UART_WriteString("\r\nPress the <ENTER> key to measure and display the weigh scale data.\r\n ");

   // Infinite loop
   while (1) {

      if (uart_cmd == UART_TRUE) {

         /* Read data register */
         ui32AdcValue = AD7791_DataScan();

         /* Convert ADC codes into input voltage to the converter */
         fVoltage = AD7791_ConvertToVolts(ui32AdcValue);

         /* Calculate unknown weight using ADC reading and grams per LSB number */
         fWeight = CN0216_WeightCalculation (ui32AdcValue);


         if ((fWeight > (CAL_WEIGHT + (CAL_WEIGHT * 0.025))) || (fWeight < (-(CAL_WEIGHT * 0.025)))) {
            CN0216_Printf("\r\nWeight being measured is out of range for this calibration weight.");
            CN0216_Printf("\r\nPlease measure a weight within the calibration boundaries.\r\n ");

         } else {
            CN0216_Printf("\r\nADC Data Register Value = %#08x", ui32AdcValue);     /* Send valid data register read value */
            CN0216_Printf("\r\nADC Input Voltage input = %f V", fVoltage);           /* Send valid voltage input value */
            CN0216_Printf("\r\nSensor Input Weight = %f grams", fWeight);                         /* Send valid grams value */
         }

         UART_WriteString("\r\n");

         uart_cmd = UART_FALSE;
      }

   }

}

#pragma GCC diagnostic pop

